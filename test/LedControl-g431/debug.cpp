#include "config.hpp"
#include <coco/Time.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/usart.hpp>


namespace coco {
namespace debug {

const auto txPin = gpio::PC10 | gpio::AF7;
#define UART_INFO usart::USART3_INFO
constexpr auto uartClock = USART3_CLOCK;
constexpr auto uartConfig = usart::Config::DEFAULT;
constexpr int baudRate = 115200;

void init() {
    // initialize debug LEDs
	// no debug LEDs

    // initialize UART for debug output to virtual COM port
    gpio::configureAlternate(txPin);
    UART_INFO.init()
        .configure(uartConfig, USART_CR1_FIFOEN)
        .setBaudRate(uartClock, baudRate)
        .enableTx();
}

void set(uint32_t bits, uint32_t function) {
}

void sleep(Microseconds<> time) {
	int64_t count = int64_t(23) * time.value;
	for (int64_t i = 0; i < count; ++i) {
		__NOP();
	}
}

void write(const char *message, int length) {
    auto uart = UART_INFO.usart;

    for (int i = 0; i < length; ++i) {
        // wait until fifo has space
        while ((uart->ISR & USART_ISR_TXE_TXFNF) == 0);

        // send a character
        uart->TDR = message[i];
    }
}

} // debug
} // coco
