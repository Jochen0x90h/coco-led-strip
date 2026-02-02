#include "LedStrip_I2S.hpp"
#include <coco/debug.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>
#include <coco/platform/gpio.hpp>


namespace {
// include generated table, see generator subdirectory
#include "bitTable_I2S.hpp"
}

namespace coco {

LedStrip_I2S::LedStrip_I2S(Loop_Queue &loop, gpio::Config sckPin, gpio::Config lrckPin, gpio::Config dataPin,
    int bitTime, int resetTime)
    : BufferDevice(State::READY)
    , loop_(loop)
{
    // debug start indicator pin
    //gpio::enableOutput(gpio::Config::P0_19, false);

    // configure I2S pins
    gpio::enableAlternate(dataPin);
    auto i2s = NRF_I2S;
    //i2s->PSEL.MCK = DISCONNECTED;
    i2s->PSEL.SCK = gpio::getPinPortIndex(sckPin);
    i2s->PSEL.LRCK = gpio::getPinPortIndex(lrckPin);
    //i2s->PSEL.SDIN = DISCONNECTED;
    i2s->PSEL.SDOUT = gpio::getPinPortIndex(dataPin);

    // initialize I2S
    i2s->CONFIG.MODE = N(I2S_CONFIG_MODE_MODE, Master);
    i2s->CONFIG.RXEN = 0;
    i2s->CONFIG.TXEN = N(I2S_CONFIG_TXEN_TXEN, Enabled);
    i2s->CONFIG.MCKEN = N(I2S_CONFIG_MCKEN_MCKEN, Enabled);
    i2s->CONFIG.RATIO = N(I2S_CONFIG_RATIO_RATIO, 48X);
    i2s->CONFIG.SWIDTH = N(I2S_CONFIG_SWIDTH_SWIDTH, 24Bit);
    //i2s->CONFIG.ALIGN = N(I2S_CONFIG_ALIGN_ALIGN, Left);
    i2s->CONFIG.FORMAT = N(I2S_CONFIG_FORMAT_FORMAT, Aligned);
    i2s->CONFIG.CHANNELS = N(I2S_CONFIG_CHANNELS_CHANNELS, Stereo);

    // https://devzone.nordicsemi.com/f/nordic-q-a/391/uart-baudrate-register-values
    //int value = (int64_t(bitRate.value * 3) << 32) / 32000000;
    int value = (int64_t(3000) << 32) / int(bitTime * 32);
    i2s->CONFIG.MCKFREQ = (value + 0x800) & 0xFFFFF000;
    //i2s->CONFIG.MCKFREQ = N(I2S_CONFIG_MCKFREQ_MCKFREQ, 32MDIV16);

    i2s->RXTXD.MAXCNT = LED_BUFFER_SIZE;
    i2s->INTENSET = N(I2S_INTENSET_TXPTRUPD, Set);// | N(I2S_INTENSET_STOPPED, Set);
    i2s->ENABLE = N(I2S_ENABLE_ENABLE, Enabled);

    // calc reset time in number of words
    //int actualFreq = int64_t(i2s->CONFIG.MCKFREQ) * 32000000 >> 32;
    int wordFreq = int64_t(i2s->CONFIG.MCKFREQ) * 1333333 >> 32; // frequency of 24 bit words, e.g. 41kHz
    resetWords_ = (wordFreq * resetTime) / 1000000 + 1;
}

LedStrip_I2S::~LedStrip_I2S() {
}

int LedStrip_I2S::getBufferCount() {
    return buffers_.count();
}

LedStrip_I2S::BufferBase &LedStrip_I2S::getBuffer(int index) {
    return buffers_.get(index);
}

void LedStrip_I2S::handle() {
    auto i2s = NRF_I2S;
    switch (phase_) {
    case Phase::COPY:
        {
            // get LED buffer fill size (already occupied part of the buffer)
            int size = size_;

            // source data
            uint8_t *begin = data_;
            uint8_t *src = begin;
            uint8_t *end2 = src + (LED_BUFFER_SIZE - size);
            uint8_t *end = std::min(end2, end_);

            // destination buffer
            int offset = offset_;
            volatile uint32_t *dst = buffer_ + offset;
            uintptr_t ptr = uintptr_t(dst);
            dst += size;

            // copy/convert
            for (; src != end; ++src, ++dst) {
                *dst = bitTable[*src];
            }

            // check if LED buffer is full
            if (end == end2) {
                // advance source data pointer
                data_ = end;

                // set DMA pointer to LED buffer
                i2s->TXD.PTR = ptr;

                // toggle and reset LED buffer
                offset_ = offset ^ LED_BUFFER_SIZE;
                size_ = 0;

                // stay in copy phase
                break;
            }

            // copy data finished (application buffer is not accessed any more)
            //gpio::setOutput(gpio::Config::P0_19, true);

            // notify the application that the buffer is finished (next buffer can be started only after reset time)
            transfers_.pop(
                [this](BufferBase &buffer) {
                    // push finished transfer buffer to event loop so that BufferBase::handle() gets called from the event loop
                    loop_.push(buffer);
                    return true;
                }
            );

            // update LED buffer fill size
            size_ = size + (end - begin);

            // go to reset phase
            phase_ = Phase::RESET;
        }
        // fall through
    case Phase::RESET:
        {
            // get buffer size (already occupied part of the buffer) and number of free words
            int size = size_;
            int free = LED_BUFFER_SIZE - size;

            // number of words to clear
            int count = resetCount_;
            int toClear = std::min(count, free);

            // destination
            int offset = offset_;
            volatile uint32_t *dst = buffer_ + offset;
            uintptr_t ptr = uintptr_t(dst);
            dst += size;
            volatile uint32_t *end = dst + toClear;

            // clear
            for (; dst != end; ++dst) {
                *dst = 0;
            }

            // check if buffer is full
            if (toClear == free) {
                // decrease resetCount
                resetCount_ = count - toClear;

                // set DMA pointer
                i2s->TXD.PTR = ptr;

                // toggle and reset buffer
                offset_ = offset ^ LED_BUFFER_SIZE;
                size_ = 0;

                // stay in reset phase
                break;
            }

            // reset finished: update buffer fill size
            size_ = size + toClear;

            // start next buffer if there is one
            transfers_.visitFirst(
                [](BufferBase &next) {
                    // start next transfer if there is one
                    next.start();
                }
            );
        }

        // break if a new buffer was started
        if (phase_ != Phase::RESET)
            break;

        // go to idle phase
        phase_ = Phase::IDLE;

        // fall through
    case Phase::IDLE:
        // let I2S run for some more cycles
        if (--idleCount_ >= 0) {
            // get buffer size (already occupied part of the buffer)
            int size = size_;

            // destination
            int offset = offset_;
            volatile uint32_t *dst = buffer_ + offset;
            uintptr_t ptr = uintptr_t(dst);
            volatile uint32_t *end = dst + LED_BUFFER_SIZE;
            dst += size;

            // clear
            for (; dst != end; ++dst) {
                *dst = 0;//0xffffff;
            }

            // set DMA pointer
            i2s->TXD.PTR = ptr;

            // toggle and reset buffer
            offset_ = offset ^ LED_BUFFER_SIZE;
            size_ = 0;
        } else {
            // stop I2S
            i2s->TASKS_STOP = TRIGGER;
            phase_ = Phase::STOPPED;
        }
        break;
    case Phase::STOPPED:
        // nothing to do
        ;
    }
}


// BufferBase

LedStrip_I2S::BufferBase::BufferBase(uint8_t *data, int capacity, LedStrip_I2S &device)
    : coco::Buffer(data, capacity, device.st.state), device_(device)
{
    device.buffers_.add(*this);
}

LedStrip_I2S::BufferBase::~BufferBase() {
}

bool LedStrip_I2S::BufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if WRITE flag is set
    assert((op & Op::WRITE) != 0);

    auto &device = device_;

    // add to list of pending transfers and start immediately if list was empty and not in RESET phase
    {
        nvic::Guard guard(I2S_IRQn);
        if (device.transfers_.push(*this) && device.phase_ >= Phase::IDLE)
            start();
    }

    // set state
    setBusy();

    return true;
}

bool LedStrip_I2S::BufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = device_;

    // remove from pending transfers if not yet started, otherwise complete normally
    if (device.transfers_.remove(nvic::Guard(I2S_IRQn), *this, false)) {
        // cancel succeeded: set buffer ready again
        // resume application code, therefore interrupt should be enabled at this point
        setReady(0);
    }

    return true;
}

void LedStrip_I2S::BufferBase::start() {
    //gpio::setOutput(gpio::Config::P0_19, false);

    auto &device = device_;
    auto i2s = NRF_I2S;

    // set data
    device.data_ = data_;
    device.end_ = data_ + size_;

    // set reset count (enlarge so that at least one buffer gets filled)
    device.resetCount_ = std::max(device.resetWords_, LED_BUFFER_SIZE - int(size_));

    // set idle count
    device.idleCount_ = 3;

    // get curren phase
    auto ph = device.phase_;

    // start with copy phase
    device.phase_ = Phase::COPY;

    // copy first data when STOPPED (called from start(op)) or RESET (called from handle() for next buffer)
    if (ph != Phase::IDLE)
        device.handle();

    // start I2S when STOPPED (called from start(op) after a pause)
    if (ph == Phase::STOPPED)
        i2s->TASKS_START = TRIGGER;
}

void LedStrip_I2S::BufferBase::handle() {
    setReady();
}

} // namespace coco
