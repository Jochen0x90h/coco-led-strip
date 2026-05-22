#include "LedStrip_emu.hpp"
#include "GuiLedStrip.hpp"


namespace coco {

LedStrip_emu::LedStrip_emu(Loop_emu &loop)
    : BufferDevice(State::READY)
    , loop_(loop)
{
    // ensure that strip is initially black
    data_.assign(3, 0);

    // add to gui handlers of event loop
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

void LedStrip_emu::onGui(Gui &gui) {
    transfers_.pop([this, &gui](auto &buffer) {
        // get data
        uint8_t *data = buffer.data_;
        int size = buffer.size_;
        data_.assign(data, data + size);

        buffer.setSuccess();
        buffer.setReady();
    });

    // draw led strip
    gui.draw<GuiLedStrip>(data_.data(), data_.size() / 3);
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
