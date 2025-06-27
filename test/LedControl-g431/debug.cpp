#include "config.hpp"
#include <coco/Time.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/usart.hpp>


namespace coco {
namespace debug {

constexpr auto redPin = gpio::PB1 | gpio::Config::INVERT;
constexpr auto greenPin = gpio::PB2 | gpio::Config::INVERT;
constexpr auto bluePin = gpio::PB11 | gpio::Config::INVERT;

const auto txPin = gpio::PC10 | gpio::AF7;
#define UART_INFO usart::USART3_INFO
constexpr auto uartClock = USART3_CLOCK;
constexpr auto uartConfig = usart::Config::ENABLE_FIFO;
constexpr auto uartFormat = usart::Format::DEFAULT;
constexpr int baudRate = 115200;

void init() {
    // initialize debug LEDs
    //gpio::enableOutput(redPin, false);
    //gpio::enableOutput(greenPin, false);
    //gpio::enableOutput(bluePin, false);

    // initialize UART for debug output to virtual COM port
    gpio::enableAlternate(txPin);
    UART_INFO.enableClock()
        .initBaudRate(uartConfig, uartClock, baudRate)
        .enable(uartConfig, uartFormat, usart::Function::TX);
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
    auto uart = UART_INFO.registers;

    for (int i = 0; i < length; ++i) {
        // wait until tx fifo has space
        while ((uart->ISR & USART_ISR_TXE_TXFNF) == 0);

        // send a character
        uart->TDR = message[i];
    }
}

} // debug
} // coco
