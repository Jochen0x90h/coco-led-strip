#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/LedStrip_I2S.hpp>
#include <coco/board/config.hpp>


using namespace coco;

constexpr int LEDSTRIP_LENGTH = 300;

// drivers for LedStripTest
struct Drivers {
	Loop_RTC0 loop;
	LedStrip_I2S ledStrip{loop,
		gpio::Config::P1_14, // SCK (not needed)
		gpio::Config::P0_2,//gpio::P1(15), // LRCK (not needed)
		gpio::Config::P0_3, // data
		1125ns, // bit time T
		75us}; // reset time
	LedStrip_I2S::Buffer<LEDSTRIP_LENGTH * 3> buffer1{ledStrip};
	LedStrip_I2S::Buffer<LEDSTRIP_LENGTH * 3> buffer2{ledStrip};
};

Drivers drivers;

extern "C" {
void I2S_IRQHandler() {
	drivers.ledStrip.I2S_IRQHandler();
}
}
