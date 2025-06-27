#include "LedStrip_UART_DMA.hpp"
//#include <coco/debug.hpp>


namespace {
// include generated table, see generator subdirectory
#include "bitTable_UART.hpp"
}

namespace coco {

// LedStrip_UART_DMA

LedStrip_UART_DMA::LedStrip_UART_DMA(Loop_Queue &loop, gpio::Config txPin, const UartInfo &uartInfo,
    const dma::Info &dmaInfo, uint32_t brr, int resetCount)
    : BufferDevice(State::READY)
    , loop(loop)
    , txPin(txPin)
    , uartIrq(uartInfo.irq)
    , resetCount(resetCount)
{
    // debug signal, (Nucleo board: CN9 1)
    //gpio::enableOutput(gpio::PC5 | gpio::Config::SPEED_HIGH, false);

    // configure UART TX pin (mode is set to alternate when data is sent and to output during reset time)
    gpio::setOutput(txPin, false);
    gpio::enableAlternate(txPin);

    // initialize UART
    auto config = usart::Config::OVER_8
#ifdef HAVE_USART_INVERT_RX_TX
        // invert output if requested. If TXINV is not supported, the output is always inverted (use e.g. 74hct1g04 to invert)
        | ((txPin & gpio::Config::INVERT) != 0 ? usart::Config::NONE : usart::Config::INVERT_TX)
#endif
        ;

#ifdef HAVE_USART_DATA_7
    auto format = usart::Format::DATA_7 | usart::Format::STOP_1 | usart::Format::LSB_FIRST;
#else
    auto format = usart::Format::DATA_8 | usart::Format::STOP_1 | usart::Format::LSB_FIRST;
#endif

    auto uart = this->uart = uartInfo.enableClock()
        .initBRR8x(brr)
        .enable(config, format,
            usart::Function::TX, // enable transmitter
            usart::Interrupt::NONE,
            usart::DmaRequest::TX); // enable TX DMA request

    // initialize TX DMA channel
    this->dmaChannel = dmaInfo.enableClock<DmaChannel>()
        .setDestinationAddress(&uart.txRegister());

    // map DMA to UART TX
    uartInfo.mapTx(dmaInfo);

    nvic::setPriority(uartInfo.irq, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()
    nvic::setPriority(dmaInfo.irq, nvic::Priority::MEDIUM);
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
    dmaChannel.clearStatus(dma::Status::TRANSFER_COMPLETE);

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
            dmaChannel.setSourceAddress(dst);

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
                dmaChannel.enable(dma::Config::TRANSFER_COMPLETE_INTERRUPT);

                // advance source data pointer
                this->data = end;

                // stay in copy phase
                break;
            }

            // copy data finished  (application buffer is not accessed any more)

            // notify the application that the buffer is finished (next buffer can be started only after reset time)
            this->transfers.pop(
                [this](BufferBase &buffer) {
                    // push finished transfer buffer to event loop so that BufferBase::handle() gets called from the event loop
                    this->loop.push(buffer);
                    return true;
                }
            );

            // enable DMA (without transfer complete interrupt, we use UART transmission complete interrupt instead
            // because we want to disable the tx pin after the last bit was sent)
            dmaChannel.enable();

            // enable UART transmission complete interrupt (TC flag gets cleared automatically by the new data)
            uart->CR1 = uart->CR1 | USART_CR1_TCIE;

            // go to reset phase
            this->phase = Phase::RESET;
        }
        break;
    case Phase::RESET:
        {
            // set tx pin low (configure as output instead of UART TX)
            gpio::setMode(this->txPin, gpio::Mode::OUTPUT);

            // debug signal
            //gpio::setOutput(gpio::PC5, true);

            // clear first byte of buffer (not necessary as TX pin is permanently low)
            //this->buffer[0] = 0;

            // dummy DMA transfer to measure reset time (new data also clears TC flag of UART)
            dmaChannel.setSourceAddress(this->buffer)
                .setCount(this->resetCount)
                .enable(dma::Config::DEFAULT, dma::Increment::NONE);

            // transmission complete interrupt stays enabled

            // go to finished phase
            this->phase = Phase::FINISHED;
        }
        break;
    case Phase::FINISHED:
        {
            // debug signal
            //gpio::setOutput(gpio::PC5, false);

            // disable transmission complete interrupt
            uart->CR1 = uart->CR1 & ~(USART_CR1_TCIE);

            this->phase = Phase::STOPPED;

            // start next buffer if there is one
            this->transfers.visitFirst(
                [](BufferBase &next) {
                    // start next transfer if there is one
                    next.start();
                }
            );
        }
        break;
    case Phase::STOPPED:
        // nothing to do
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
    {
        // DMA irq does not need to be disabled because changes to the transfers queue are not done during DMA irq
        nvic::Guard guard(device.uartIrq);
        if (device.transfers.push(*this) && device.phase == Phase::STOPPED)
            start();
    }

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
