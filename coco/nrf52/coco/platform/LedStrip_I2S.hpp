#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Frequency.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @brief Implementation of LED strip interface on nrf52 using I2S.
/// Can be used for RGB or RGBW LED strips with WS2812 style data transfer.
///
/// Reference manual:
///   https://docs.nordicsemi.com/bundle/ps_nrf52840/page/i2s.html
/// Resources:
///   I2S
class LedStrip_I2S : public BufferDevice {
protected:
    LedStrip_I2S(Loop_Queue &loop, gpio::Config sckPin, gpio::Config lrckPin, gpio::Config dataPin, int bitTime, int resetTime);
public:
    /// @brief Constructor
    /// @param loop event loop
    /// @param sckPin i2s sck pin needs to be an unused pin
    /// @param lrckPin i2s lrck pin needs to be an unused pin
    /// @param dataPin pin that transmits data to the LED strip
    /// @param bitTime bit time T where T0H is T/3 and T1H is 2T/3, e.g. T = 1125ns -> T0H = 375ns, T1H = 750ns
    /// @param resetTime reset time in us, e.g. 20μs
    LedStrip_I2S(Loop_Queue &loop, gpio::Config sckPin, gpio::Config lrckPin, gpio::Config dataPin,
        Nanoseconds<> bitTime, Microseconds<> resetTime)
        : LedStrip_I2S(loop, sckPin, lrckPin, dataPin, bitTime.value, resetTime.value) {}

    ~LedStrip_I2S() override;


    // internal buffer base class, derives from IntrusiveListNode for the list of active transfers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class LedStrip_I2S;
    public:
        /// @param Constructor
        /// @param data data of the buffer
        /// @param capacity capacity of the buffer in bytes
        /// @param device led strip device to attach to
        BufferBase(uint8_t *data, int capacity, LedStrip_I2S &device);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void start();
        void handle() override;

        LedStrip_I2S &device_;
    };

    /// @brief Buffer for transferring data to LED strip.
    /// @tparam C capacity of buffer in bytes
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(LedStrip_I2S &device) : BufferBase(data_, C, device) {}

    protected:
        alignas(4) uint8_t data_[C];
    };


    // BufferDevice methods
    int getBufferCount();
    BufferBase &getBuffer(int index);

    /// @brief I2S interrupt handler, needs to be called from global I2S interrupt handler
    ///
    void I2S_IRQHandler() {
        // check if tx pointer has been read
        if (NRF_I2S->EVENTS_TXPTRUPD) {
            NRF_I2S->EVENTS_TXPTRUPD = 0;
            handle();
        }
    }

protected:
    void handle();

    Loop_Queue &loop_;

    // list of buffers
    IntrusiveList<BufferBase> buffers_;

    // list of active transfers
    InterruptQueue<BufferBase> transfers_;

    // data to transfer
    uint8_t *data_;
    uint8_t *end_;

    // reset after data
    int resetWords_;
    int resetCount_;

    // number of idle buffers to send when no new data arrives
    int idleCount_;

    // buffer for 2 x 16 LEDs
    static constexpr int LED_BUFFER_SIZE = 16 * 3;
    volatile uint32_t buffer_[2 * LED_BUFFER_SIZE];
    int offset_ = 0;
    int size_ = 0;

    enum class Phase {
        // copy data into the LED buffer
        COPY,

        // reset LEDs (by sending zeros for the specified Treset time)
        RESET,

        // continue sending zeros for a short time before stopping I2S
        IDLE,

        // nothing to do, I2S is stopped
        STOPPED
    };
    Phase phase_ = Phase::STOPPED;
};

} // namespace coco
