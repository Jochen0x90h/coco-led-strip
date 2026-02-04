#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Frequency.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/uart.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @brief Implementation of LED strip interface on stm32 using USARTx, UARTx or LPUARTx.
/// Can be used for RGB or RGBW LED strips with WS2812 style data transfer.
/// Note that the output is inverted for STM32F4, use e.g. 74hct1g04 to invert and also level shift to 5V
///
/// Reference manual:
///   f0:
///     https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
///       USART: Section 27
///       DMA: Section 10, Table 29
///       Code Examples: Section A.19
///   g4:
///     https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
///       USART: Section 37
///       DMA: Section 12
///       DMAMUX: Section 13
/// Data sheet:
///   f0:
///     https://www.st.com/resource/en/datasheet/stm32f042f6.pdf
///       Alternate Functions: Section 4, Tables 14-16, Page 37
///     https://www.st.com/resource/en/datasheet/dm00039193.pdf
///       Alternate Functions: Section 4, Tables 14+15, Page 37
///   g4:
///     https://www.st.com/resource/en/datasheet/stm32g431rb.pdf
///       Alternate Functions: Section 4.11, Table 13, Page 61
/// Resources:
///   USART or UART
///   DMA
class LedStrip_UART_DMA : public BufferDevice {
protected:
    using UartInfo = uart::Info<uart::Feature::BAUD_RATE | uart::Feature::ASYNC_MODE>;

    /// @brief Internal constructor
    /// @param loop Event loop
    /// @param txPin Transmit (TX) pin and alternative function (see data sheet) that transmits data to the LED strip, supports INVERT flag
    /// @param usartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channel to use
    /// @param brr Contents of baud rate register
    /// @param resetCount Number of bytes to send during reset time, e.g. 20μs / T = 20μs / 1125ns = 18.6667 -> 19 bytes
    LedStrip_UART_DMA(Loop_Queue &loop, gpio::Config txPin,
        const UartInfo &uartInfo, const dma::Info<> &dmaInfo, uint32_t brr, int resetCount);
public:
    /// @brief Constructor
    /// @param loop Event loop
    /// @param txPin Transmit (TX) pin and alternative function (see data sheet) that transmits data to the LED strip, supports INVERT flag
    /// @param usartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channel to use
    /// @param clock Peripheral clock frequency (APB1_CLOCK or APB2_CLOCK depending on USART)
    /// @param bitTime Bit time T where T0H is T/3 and T1H is 2T/3, e.g. T = 1125ns -> T0H = 375ns, T1H = 750ns
    /// @param resetTime Reset time in us, e.g. 20μs
    LedStrip_UART_DMA(Loop_Queue &loop, gpio::Config txPin, const UartInfo &uartInfo, const dma::Info<> &dmaInfo,
        Kilohertz<> clock, Nanoseconds<> bitTime, Microseconds<> resetTime)
        : LedStrip_UART_DMA(loop, txPin, uartInfo, dmaInfo,
        std::max(int(clock * bitTime / 3) + 1, 8), int(clock / ((int(clock * bitTime / 3) + 1) * 9) * resetTime)) {}

    ~LedStrip_UART_DMA() override;


    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class LedStrip_UART_DMA;
    public:
        /// @brief Constructor
        /// @param data data of the buffer
        /// @param capacity capacity of the buffer in bytes
        /// @param channel channel to attach to
        BufferBase(uint8_t *data, int capacity, LedStrip_UART_DMA &device);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void start();
        void handle() override;

        LedStrip_UART_DMA &device_;
    };

    /// @brief Buffer for transferring data over UART.
    /// @tparam C capacity of buffer in bytes
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(LedStrip_UART_DMA &device) : BufferBase(data_, C, device) {}

    protected:
        alignas(4) uint8_t data_[((C + 2) / 3) * 3];
    };


    // BufferDevice methods
    int getBufferCount() override;
    BufferBase &getBuffer(int index) override;

    /// @brief UART interrupt handler, needs to be called from global USART/UART interrupt handler (e.g. USART1_IRQHandler() for uart::USART1_INFO on STM32G4)
    ///
    void UART_IRQHandler() {
        // check if transmission has completed
        if ((uart_.status() & uart::Status::TX_COMPLETE) != 0)
            handle();
    }

    /// @brief DMA interrupt handler, needs to be called from DMA channel interrupt handler (e.g. DMA1_Channel1_IRQHandler() for dma::DMA1_CH1_INFO on STM32G4)
    ///
    void DMA_IRQHandler() {
        // check if receive has completed
        if ((dmaChannel_.status() & dma::Status::TRANSFER_COMPLETE) != 0)
            handle();
    }

protected:
    void handle();

    Loop_Queue &loop_;

    gpio::Config txPin_;

    // uart
    UartInfo::Instance uart_;
    int uartIrq_;

    // dma
    using DmaChannel = dma::Channel<dma::Mode::TX8>;
    DmaChannel dmaChannel_;

    // list of buffers
    IntrusiveList<BufferBase> buffers_;

    // list of active transfers
    InterruptQueue<BufferBase> transfers_;

    // data to transfer
    uint8_t *data_;
    uint8_t *end_;

    // reset after data
    int resetCount_;

    // buffer for 2 x 16 LEDs
    static constexpr int LED_BUFFER_SIZE = 16 * 3;
    volatile uint32_t buffer_[(LED_BUFFER_SIZE * 4) / 3 / 2]; // need 4 x uint16_t for one LED which are 3 bytes

    enum class Phase {
        // copy data into the LED buffer
        COPY,

        // reset LEDs (by sending zeros for the specified Treset time)
        RESET,

        // notify the main application that a buffer has finished
        FINISHED,

        // nothing to do
        STOPPED
    };
    Phase phase_ = Phase::STOPPED;
};

} // namespace coco
