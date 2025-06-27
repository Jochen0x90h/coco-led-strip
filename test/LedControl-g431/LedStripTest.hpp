#pragma once

#include "config.hpp"
#include <coco/platform/LedStrip_UART_DMA.hpp>
//#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Loop_SysTick.hpp>
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
    //Loop_TIM2 loop{APB1_TIMER_CLOCK};
    Loop_SysTick loop{AHB_CLOCK, Loop_SysTick::Mode::POLL};

    // LED strip
    using LedStrip = LedStrip_UART_DMA;
    LedStrip ledStrip{loop,
        gpio::PC4 | gpio::AF7 | gpio::Config::SPEED_HIGH, // USART1 TX (RS485_TX)
        usart::USART1_INFO,
        dma::DMA1_CH1_INFO,
        USART1_CLOCK,
        1125ns, // bit time T
        75us}; // reset time
    LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer1{ledStrip};
    LedStrip::Buffer<LEDSTRIP_LENGTH * 3> buffer2{ledStrip};

    Drivers() {
        // set RS485_DE high
        gpio::enableOutput(gpio::PB0, true);

        // enable reference voltage
        //vref::enable(vref::Config::INTERNAL_2V048);
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
