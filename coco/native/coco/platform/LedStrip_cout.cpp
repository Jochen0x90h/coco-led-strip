#include "LedStrip_cout.hpp"
//#include <coco/Color.hpp>
#include <iostream>


namespace coco {

LedStrip_cout::LedStrip_cout(Loop_native &loop)
    : BufferDevice(State::READY)
    , loop_(loop), callback_(makeCallback<LedStrip_cout, &LedStrip_cout::handle>(this))
{
}

LedStrip_cout::~LedStrip_cout() {
}

int LedStrip_cout::getBufferCount() {
    return buffers_.count();
}

LedStrip_cout::Buffer &LedStrip_cout::getBuffer(int index) {
    return buffers_.get(index);
}

void LedStrip_cout::handle() {
    transfers_.pop([this](auto &buffer) {
        // https://stackoverflow.com/questions/30097953/ascii-art-sorting-an-array-of-ascii-characters-by-brightness-levels-c-c
        static const char lookup[] = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@@";
        const int size = std::size(lookup) - 2;

        int count = buffer.size_ / 3;
        Color *colors = (Color*)buffer.data_;
        for (int i = 0; i < count; ++i) {
            Color color = colors[i];
            int intensity = int((0.30f * color.r + 0.59f * color.g + 0.11f * color.b) / 255.0f * size);
            char ch = lookup[intensity];
            std::cout << ch;
        }
        std::cout << std::endl;
        buffer.setSuccess();
        buffer.setReady();

        // check if there are more buffers in the list
        if (!transfers_.empty())
            loop_.invoke(callback_);
    });
}


// Buffer

LedStrip_cout::Buffer::Buffer(int length, LedStrip_cout &device)
    : coco::Buffer(new uint8_t[length * 3], length * 3, Device::State::READY)
    , device_(device)
{
    device.buffers_.add(*this);
}

LedStrip_cout::Buffer::~Buffer() {
    delete [] data_;
}

bool LedStrip_cout::Buffer::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    // add buffer to list of transfers and let event loop call LedStrip_cout::handle() when the first was added
    if (device_.transfers_.push(*this))
        device_.loop_.invoke(device_.callback_);

    // set state
    setBusy();

    return true;
}

bool LedStrip_cout::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    device_.transfers_.remove(*this);
    setError(std::errc::operation_canceled);
    setReady();

    return true;
}

} // namespace coco
