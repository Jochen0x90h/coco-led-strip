#pragma once

#include "config.hpp"
#include <coco/platform/LedStrip_UART_DMA.hpp>
#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/vref.hpp>
#include <coco/platform/opamp.hpp>
#include <coco/platform/dac.hpp>
#include <coco/debug.hpp>


using namespace coco;
using namespace coco::literals;

constexpr int LEDSTRIP_LENGTH = 300;

// drivers for Test
struct Drivers {
	Loop_TIM2 loop{APB1_TIMER_CLOCK};

	// LED strip
	using LedStrip = LedStrip_UART_DMA;
	LedStrip ledStrip{loop,
		gpio::Config::PE0 | gpio::Config::AF7 | gpio::Config::SPEED_HIGH, // USART1 TX (RS4xx_TX1)
		usart::USART1_INFO,
		dma::DMA1_CH1_INFO,
		USART1_CLOCK,
		1125ns, // bit time T
		50us}; // reset time
	LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer1{ledStrip};
	LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer2{ledStrip};

	Drivers() {
		// set RS4xx_DE1 high
		gpio::configureOutput(gpio::Config::PE3, true);

		// enable reference voltage
		vref::configure(vref::Config::INTERNAL_2V048);

		// enable opamps for DAC3
		opamp::configure(OPAMP3, opamp::Config::OUT_PIN | opamp::Config::OPAMP3_INP_DAC3_CH2 | opamp::Config::INM_FOLLOWER);

		// configure voltage
		dac::DAC3_INFO.rcc.enableClock();
		gpio::configureAnalog(gpio::Config::PB11);
		DAC3->MCR = ((int(AHB_CLOCK) - 1) / 80000000 << DAC_MCR_HFSEL_Pos) | DAC_MCR_MODE2_1 | DAC_MCR_MODE2_0; // internal connection to opamp
		DAC3->CR = DAC_CR_EN2;
		debug::sleep(1us);
		DAC3->DHR12R2 = 2048; // 16V
		//DAC3->DHR12R2 = 3300; // 7V

		// enable voltage
		debug::sleep(10us);
		gpio::configureOutput(gpio::Config::PF2, true);
	}
};

Drivers drivers;

extern "C" {

// LED strip
void USART1_IRQHandler() {
	drivers.ledStrip.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
	drivers.ledStrip.DMA_IRQHandler();
}
}
