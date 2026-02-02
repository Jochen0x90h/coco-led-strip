#include "LedStrip_emu.hpp"
#include "GuiLedStrip.hpp"


namespace coco {

LedStrip_emu::LedStrip_emu(Loop_emu &loop)
    : BufferDevice(State::READY)
    , loop(loop)
{
    loop.guiHandlers.add(*this);
}

LedStrip_emu::~LedStrip_emu() {
}

int LedStrip_emu::getBufferCount() {
    return this->buffers.count();
}

LedStrip_emu::Buffer &LedStrip_emu::getBuffer(int index) {
    return this->buffers.get(index);
}

void LedStrip_emu::handle(Gui &gui) {
    auto buffer = this->transfers.pop();
    if (buffer != nullptr) {
        gui.draw<GuiLedStrip>(buffer->data_, buffer->size_ / 3);
        buffer->setReady();
    } else {
        // draw emulated LED strip with previous content
        gui.draw<GuiLedStrip>();
    }
}


// Buffer

LedStrip_emu::Buffer::Buffer(int length, LedStrip_emu &device)
    : coco::Buffer(new uint8_t[length * 3], length * 3, device.st.state)
    , device(device)
{
    device.buffers.add(*this);
}

LedStrip_emu::Buffer::~Buffer() {
    delete [] this->data_;
}

bool LedStrip_emu::Buffer::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if WRITE flag is set
    assert((op & Op::WRITE) != 0);

    // add buffer to list of transfers. No need to start first transfer as LedStrip_emu::handle() gets called periodically
    this->device.transfers.push(*this);

    // set state
    setBusy();

    return true;
}

bool LedStrip_emu::Buffer::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    setReady(0);
    return true;
}

} // namespace coco
