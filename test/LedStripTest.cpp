#include <LedStripTest.hpp>
#include <coco/debug.hpp>


using namespace coco;

/*
    Test for LED strip.

    You can use the WLED power calculator to calulate the power supply of LED strips: https://wled-calculator.github.io/
*/

#ifdef RGB
struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
#endif

#ifdef WS2812
struct Color {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};
#endif


struct SingleBufferStrip {
    // number of LEDs
    int count;

    // single buffer
    Buffer &buffer;

    SingleBufferStrip(Buffer &buffer) : buffer(buffer) {
        this->count = buffer.capacity() / sizeof(Color);
    }

    int size() {return this->count;}

    // get array of colors, only valid until show() is called
    Array<Color> array() {
        return {this->buffer.pointer<Color>(), this->count};
    }

    [[nodiscard]] Awaitable<Buffer::Events> show() {
        // write buffer
        return this->buffer.write(this->count * sizeof(Color));
    }
};

struct DoubleBufferStrip {
    // number of LEDs
    int count;

    // double buffer
    Buffer *buffers[2];
    Buffer *buffer;

    DoubleBufferStrip(Buffer &buffer1, Buffer &buffer2) : buffers{&buffer1, &buffer2} {
        this->count = std::min(buffer1.capacity(), buffer2.capacity()) / sizeof(Color);
        this->buffer = &buffer1;
    }

    int size() {return this->count;}

    // get array of colors, only valid until show() is called
    Array<Color> array() {
        return {this->buffer->pointer<Color>(), this->count};
    }

    [[nodiscard]] Awaitable<Buffer::Events> show() {
        // start writeing buffer
        this->buffer->startWrite(this->count * sizeof(Color));

        // toggle buffer
        this->buffer = this->buffer == this->buffers[0] ? this->buffers[1] : this->buffers[0];

        // wait until other buffer is finished
        return this->buffer->untilReadyOrDisabled();
    }
};

// test correct timing, bit order and RGB order
template <typename S>
Coroutine testRgb(Loop &loop, S &strip) {
    while (true) {
        // get time in milliseconds
        auto time = int((loop.now() - Loop::Time(0)) / 5ms);

        uint8_t fade = (time & 256) ? 255 - time : time;

        auto leds = strip.array();
        int count = leds.size();
        for (int i = 0; i < count; ++i) {
            Color &color = leds[i];

            color.r = color.g = color.b = 0;
            int x = i & 3;
            if (x == 0) {
                color.r = fade;
            }
            if (x == 1) {
                color.g = fade;
            }
            if (x == 2) {
                color.b = fade;
            }
        }
        co_await strip.show();
        //debug::out << "show\n";
    }
}

// moving color ramps
template <typename S>
Coroutine colorRamps(Loop &loop, S &strip) {
    while (true) {
        // get time in milliseconds
        auto time = int((loop.now() - Loop::Time(0)) / 1ms);
        int rOffset = time / 32;
        int gOffset = time * 2 / 32;
        int bOffset = time * 3 / 32;


        auto leds = strip.array();
        int count = leds.size();
        for (int i = 0; i < count; ++i) {
            Color &color = leds[i];

            int rx = i + rOffset;
            color.r = (rx & 256) ? 255 - rx : rx;

            int gx = i + gOffset;
            color.g = (gx & 256) ? 255 - gx : gx;

            int bx = i + bOffset;
            color.b = (bx & 256) ? 255 - bx : bx;
        }
        co_await strip.show();
        //debug::out << "show\n";
    }
}


int main() {
    debug::out << "LedStripTest\n";

    //SingleBufferStrip strip(drivers.buffer);
    DoubleBufferStrip strip(drivers.buffer1, drivers.buffer2);

    // test correct assignment of RGB channels
    // LEDs should fade on/off up in this order R, G, B, off, R, G, B, off ...
    testRgb(drivers.loop, strip);

    //colorRamps(drivers.loop, strip);

    drivers.loop.run();
    return 0;
}
