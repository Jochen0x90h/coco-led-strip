#include "LedStrip_UART_DMA.hpp"
//#include <coco/debug.hpp>


namespace {
#include "bitTable_UART.hpp"
}

namespace coco {

// LedStrip_UART_DMA

LedStrip_UART_DMA::LedStrip_UART_DMA(Loop_Queue &loop, gpio::Config txPin, const usart::Info &uartInfo,
    const dma::Info &dmaInfo, uint32_t brr, int resetCount)
    : BufferDevice(State::READY)
    , loop(loop)
    , txPin(txPin)
    , uart(uartInfo.usart), uartIrq(uartInfo.irq)
    , resetCount(resetCount)
{
    //gpio::configureOutput(gpio::PA(15), false);

    // enable clocks (note two cycles wait time until peripherals can be accessed, see STM32G4 reference manual section 7.2.17)
    uartInfo.rcc.enableClock();
    dmaInfo.rcc.enableClock();

    // configure UART TX pin (mode gets set to alternate function in start())
    //gpio::configure(txPin, false, gpio::Config::OUTPUT | gpio::Config::SPEED_HIGH);

    // configure UART TX pin (mode gets set to output during reset time)
    gpio::setOutput(txPin, false);
    gpio::configureAlternate(txPin);

    // initialize UART
    auto uart = uartInfo.usart;
    nvic::setPriority(this->uartIrq, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // set baud rate (minimum is 8)
    uart->BRR = ((brr & ~7) << 1) | (brr & 7);

    // set configuration registers
    uart->CR3 = USART_CR3_DMAT; // TX DMA mode
#ifndef STM32F4 // STM32F4 does not support inverting, therefore the output is inverted (use e.g. 74hct1g04 to also level shift to 5V)
    uart->CR2 = USART_CR2_DATAINV // start and stop bits of UART are inverted, therefore also invert data
        | (extract(txPin, gpio::Config::INVERT) ? 0 : USART_CR2_TXINV); // invert output according to INVERT flag
#endif
    uart->CR1 = USART_CR1_OVER8 // 8x oversampling
#ifdef USART_CR1_M1
        | USART_CR1_M1 // 7 bit
#endif
        | USART_CR1_UE; // enable UART

    // initialize TX DMA channel
    this->dmaStatus = dmaInfo.status();
    this->dmaChannel = dmaInfo.channel();
#ifdef STM32F4
    this->dmaChannel.setPeripheralAddress(&uart->DR);
#else
    this->dmaChannel.setPeripheralAddress(&uart->TDR);
#endif

    // map DMA to UART TX
    uartInfo.mapTx(dmaInfo);

    // enable transmitter
    auto cr1 = uart->CR1;
    uart->CR1 = cr1 | USART_CR1_TE;
#ifndef STM32F4 // STM32F4 does not support inverting, therefore the output is inverted (use e.g. inverting level shifter from 3.3V to 5V)
    while ((uart->ISR & USART_ISR_TEACK) == 0);
#endif

    //debug::setRed();

    /*uart->TDR = 0x05;
    while (true) {

    }*/

    nvic::enable(dmaInfo.irq);
}

LedStrip_UART_DMA::~LedStrip_UART_DMA() {
}

int LedStrip_UART_DMA::getBufferCount() {
    return this->buffers.count();
}

LedStrip_UART_DMA::BufferBase &LedStrip_UART_DMA::getBuffer(int index) {
    return this->buffers.get(index);
}

void LedStrip_UART_DMA::handle() {
    auto uart = this->uart;
    auto dmaChannel = this->dmaChannel;

    // disable DMA
    dmaChannel.disable();

    // clear interrupt flag
    this->dmaStatus.clear(dma::Status::Flags::TRANSFER_COMPLETE);

    switch (this->phase) {
    case Phase::COPY:
        {
            //gpio::setOutput(gpio::PA(15), true);

            // source data
            uint8_t *src = this->data;
            uint8_t *end = std::min(src + LED_BUFFER_SIZE, this->end);

            // destination
            uint32_t *dst = this->buffer;

            // set DMA pointer
            dmaChannel.setMemoryAddress(dst);

            // copy/convert
            for (; src < end; src += 3, dst += 2) {
                int a = src[0];
                int b = src[1];
                int c = src[2];

                dst[0] = bitTable[a >> 2]
                    | (bitTable[((a & 3) << 4) | b >> 4] << 16);
                dst[1] = bitTable[((b & 15) << 2) | c >> 6]
                    | (bitTable[c & 63] << 16);
            }
            //gpio::setOutput(gpio::PA(15), false);

            // set DMA count
            dmaChannel.setCount(uintptr_t(dst) - uintptr_t(this->buffer));

            // check if more source data to transfer
            if (end < this->end) {
                // enable DMA
                dmaChannel.enable(dma::Channel::Config::TX
                    | dma::Channel::Config::TRANSFER_COMPLETE_INTERRUPT);

                // advance source data pointer
                this->data = end;

                // stay in copy phase
                break;
            }

            // enable DMA (without transfer complete interrupt, we use UART transmission complete interrupt instead)
            dmaChannel.enable(dma::Channel::Config::TX);

            // enable UART transmission complete interrupt (TC flag gets cleared automatically by new data)
            uart->CR1 = uart->CR1 | USART_CR1_TCIE;

            // go to reset phase
            this->phase = Phase::RESET;
        }
        break;
    case Phase::RESET:
        {
            // set tx pin to output, state is low
            gpio::setMode(this->txPin, gpio::Mode::OUTPUT);

            // clear first byte of buffer (not necessary as TX pin is permanently low)
            //this->buffer[0] = 0;

            // dummy DMA transfer to measure reset time
            dmaChannel.setMemoryAddress(this->buffer);
            dmaChannel.setCount(this->resetCount);
            dmaChannel.enable(dma::Channel::Config::MEMORY_TO_PERIPHERAL);

            // enable transmission complete interrupt (TC flag gets cleared automatically by new data)
            uart->CR1 = uart->CR1 | USART_CR1_TCIE;

            // go to finished phase
            this->phase = Phase::FINISHED;
        }
        break;
    case Phase::FINISHED:
        {
            this->phase = Phase::STOPPED;

            this->transfers.pop(
                [this](BufferBase &buffer) {
                    // push finished transfer buffer to event loop so that BufferBase::handle() gets called from the event loop
                    this->loop.push(buffer);
                    return true;
                },
                [](BufferBase &next) {
                    // start next transfer if there is one
                    next.start();
                }
            );
        }
        break;
    default:
        ;
    }
}


// BufferBase

LedStrip_UART_DMA::BufferBase::BufferBase(uint8_t *data, int capacity, LedStrip_UART_DMA &device)
    : coco::Buffer(data, capacity, device.st.state), device(device)
{
    device.buffers.add(*this);
}

LedStrip_UART_DMA::BufferBase::~BufferBase() {
}

bool LedStrip_UART_DMA::BufferBase::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if WRITE flag is set
    assert((op & Op::WRITE) != 0);

    auto &device = this->device;

    // add to list of pending transfers and start immediately if list was empty
    if (device.transfers.push(nvic::Guard(device.uartIrq), *this))
        start();

    // set state
    setBusy();

    return true;
}

bool LedStrip_UART_DMA::BufferBase::cancel() {
    if (this->st.state != State::BUSY)
        return false;
    auto &device = this->device;

    // remove from pending transfers if not yet started, otherwise complete normally
    if (device.transfers.remove(nvic::Guard(device.uartIrq), *this, false))
        setReady(0);

    return true;
}

void LedStrip_UART_DMA::BufferBase::start() {
    auto &device = this->device;

    // set data
    device.data = this->p.data;
    device.end = this->p.data + this->p.size;

    // connect tx pin to UART
    gpio::setMode(device.txPin, gpio::Mode::ALTERNATE);

    // start
    device.phase = Phase::COPY;
    device.handle();
}

void LedStrip_UART_DMA::BufferBase::handle() {
    setReady();
}

} // namespace coco
