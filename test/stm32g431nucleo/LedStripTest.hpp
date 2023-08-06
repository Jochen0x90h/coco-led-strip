#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/LedStrip_UART_DMA.hpp>
#include <coco/board/config.hpp>
#include <coco/debug.hpp>


using namespace coco;

constexpr int LEDSTRIP_LENGTH = 300;

// drivers for LedStripTest
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK};

	using LedStrip = LedStrip_UART_DMA;
	LedStrip ledStrip{loop,
		//gpio::Config::PA9 | gpio::Config::AF7 | gpio::Config::SPEED_HIGH, // USART1 TX (CN5 1)
		gpio::Config::PC4 | gpio::Config::AF7 | gpio::Config::SPEED_HIGH, // USART1 TX (CN9 2)
		usart::USART1_INFO,
		dma::DMA1_CH1_INFO,
		USART1_CLOCK,
		1125ns, // bit time T
		75us}; // reset time
	LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer1{ledStrip};
	LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer2{ledStrip};
};

Drivers drivers;

extern "C" {
void USART1_IRQHandler() {
	drivers.ledStrip.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
	drivers.ledStrip.DMA_IRQHandler();
}
}
