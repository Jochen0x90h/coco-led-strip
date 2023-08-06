#pragma once

#include <coco/platform/LedStrip_cout.hpp>


using namespace coco;

constexpr int LEDSTRIP_LENGTH = 300;

// drivers for LedStripTest
struct Drivers {
	Loop_native loop;
	LedStrip_cout ledStrip{loop};
	LedStrip_cout::Buffer buffer1{LEDSTRIP_LENGTH, ledStrip};
	LedStrip_cout::Buffer buffer2{LEDSTRIP_LENGTH, ledStrip};
};

Drivers drivers;
