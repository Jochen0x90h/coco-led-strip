#include "LedStrip_cout.hpp"
//#include <coco/Color.hpp>
#include <iostream>


namespace coco {

LedStrip_cout::LedStrip_cout(Loop_native &loop)
    : BufferDevice(State::READY)
    , loop(loop), callback(makeCallback<LedStrip_cout, &LedStrip_cout::handle>(this))
{
}

LedStrip_cout::~LedStrip_cout() {
}

int LedStrip_cout::getBufferCount() {
    return this->buffers.count();
}

LedStrip_cout::Buffer &LedStrip_cout::getBuffer(int index) {
    return this->buffers.get(index);
}

void LedStrip_cout::handle() {
    auto buffer = this->transfers.pop();
    if (buffer != nullptr) {
        // https://stackoverflow.com/questions/30097953/ascii-art-sorting-an-array-of-ascii-characters-by-brightness-levels-c-c
        static const char lookup[] = " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@@";
        const int size = std::size(lookup) - 2;

        int count = buffer->size_ / 3;
        Color *colors = (Color*)buffer->data_;
        for (int i = 0; i < count; ++i) {
            Color color = colors[i];
            int intensity = int((0.30f * color.r + 0.59f * color.g + 0.11f * color.b) / 255.0f * size);
            char ch = lookup[intensity];
            std::cout << ch;
        }
        std::cout << std::endl;
        buffer->setReady();

        // check if there are more buffers in the list
        if (!this->transfers.empty())
            this->loop.invoke(this->callback);
    }
}


// Buffer

LedStrip_cout::Buffer::Buffer(int length, LedStrip_cout &device)
    : coco::Buffer(new uint8_t[length * 3], length * 3, Device::State::READY)
    , device(device)
{
    device.buffers.add(*this);
}

LedStrip_cout::Buffer::~Buffer() {
    delete [] this->data_;
}

bool LedStrip_cout::Buffer::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if WRITE flag is set
    assert((op & Op::WRITE) != 0);

    // add buffer to list of transfers and let event loop call LedStrip_cout::handle() when the first was added
    if (this->device.transfers.push(*this))
        this->device.loop.invoke(this->device.callback);

    // set state
    setBusy();

    return true;
}

bool LedStrip_cout::Buffer::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    this->device.transfers.remove(*this);
    setReady(0);
    return true;
}

} // namespace coco
