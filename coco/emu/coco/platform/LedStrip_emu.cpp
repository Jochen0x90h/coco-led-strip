#include "LedStrip_emu.hpp"
#include "GuiLedStrip.hpp"


namespace coco {

LedStrip_emu::LedStrip_emu(Loop_emu &loop)
    : BufferDevice(State::READY)
    , loop_(loop)
{
    loop.guiHandlers.add(*this);
}

LedStrip_emu::~LedStrip_emu() {
}

int LedStrip_emu::getBufferCount() {
    return buffers_.count();
}

LedStrip_emu::Buffer &LedStrip_emu::getBuffer(int index) {
    return buffers_.get(index);
}

void LedStrip_emu::handle(Gui &gui) {
    auto result = transfers_.pop([&gui](auto &buffer) {
        gui.draw<GuiLedStrip>(buffer.data_, buffer.size_ / 3);
        buffer.setSuccess();
        buffer.setReady();
    });
    if (!result) {
        // no buffer: draw emulated LED strip with previous content
        gui.draw<GuiLedStrip>();
    }
}


// Buffer

LedStrip_emu::Buffer::Buffer(int length, LedStrip_emu &device)
    : coco::Buffer(new uint8_t[length * 3], length * 3, device.state_)
    , device_(device)
{
    device.buffers_.add(*this);
}

LedStrip_emu::Buffer::~Buffer() {
    delete [] data_;
}

bool LedStrip_emu::Buffer::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    // add buffer to list of transfers. No need to start first transfer as LedStrip_emu::handle() gets called periodically
    device_.transfers_.push(*this);

    // set state
    setBusy();

    return true;
}

bool LedStrip_emu::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    device_.transfers_.remove(*this);
    setError(std::errc::operation_canceled);
    setReady();

    return true;
}

} // namespace coco
