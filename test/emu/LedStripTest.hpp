#pragma once

#include <coco/platform/LedStrip_emu.hpp>


using namespace coco;

constexpr int LEDSTRIP_LENGTH = 300;

// drivers for LedStripTest
struct Drivers {
	Loop_emu loop;
	LedStrip_emu ledStrip{loop};
	LedStrip_emu::Buffer buffer1{LEDSTRIP_LENGTH, ledStrip};
	LedStrip_emu::Buffer buffer2{LEDSTRIP_LENGTH, ledStrip};
};

Drivers drivers;
