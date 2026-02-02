#include "config.hpp"
#include <coco/Time.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/uart.hpp>


namespace coco {
namespace debug {

constexpr auto redPin = gpio::PB1 | gpio::Config::INVERT;
constexpr auto greenPin = gpio::PB2 | gpio::Config::INVERT;
constexpr auto bluePin = gpio::PB11 | gpio::Config::INVERT;

const auto txPin = gpio::PC10 | gpio::AF7;
#define UART_INFO uart::USART3_INFO
constexpr auto uartClock = USART3_CLOCK;
constexpr auto uartConfig = uart::Config::ENABLE_FIFO;
constexpr auto uartFormat = uart::Format::DEFAULT;
constexpr auto baudRate = 115200Hz;

void init() {
    // initialize debug LEDs
    gpio::enableOutput(redPin, false);
    gpio::enableOutput(greenPin, false);
    gpio::enableOutput(bluePin, false);

    // initialize UART for debug output to virtual COM port
    UART_INFO.enableClock()
        .enable(UART_INFO.enableRxTxPins(gpio::NONE, txPin, uartConfig), uartFormat, uartClock, baudRate)
        .startTx();
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
    auto uart = UART_INFO.instance();

    for (int i = 0; i < length; ++i) {
        // wait until fifo has space
        uart.waitTx();

        // send a character
        uart.tx(message[i]);
    }
}

} // debug
} // coco
