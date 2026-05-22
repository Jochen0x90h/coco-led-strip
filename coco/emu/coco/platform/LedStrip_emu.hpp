#include <coco/BufferDevice.hpp>
#include <coco/IntrusiveQueue.hpp>
#include <coco/platform/Loop_emu.hpp>
#include <string>


namespace coco {

/// @brief Implementation of a LED strip emulator that shows the LED strip on the emulator gui.
///
class LedStrip_emu : public BufferDevice, public Loop_emu::GuiHandler {
public:
    LedStrip_emu(Loop_emu &loop);
    ~LedStrip_emu() override;

    /// @brief Buffer for transferring data to a LED strip.
    ///
    class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveQueueNode {
        friend class LedStrip_emu;
    public:
        /// @brief Constructor.
        /// @param length length of emulated LED strip, i.e. number of RGB triples
        /// @param device emulator device
        Buffer(int length, LedStrip_emu &device);
        ~Buffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:

        LedStrip_emu &device_;
    };


    // BufferDevice methods
    int getBufferCount() override;
    Buffer &getBuffer(int index) override;

protected:
    void onGui(Gui &gui) override;

    Loop_native &loop_;

    // list of buffers
    IntrusiveList<Buffer> buffers_;

    // list of active transfers
    IntrusiveQueue<Buffer> transfers_;

    std::vector<uint8_t> data_;
};

} // namespace coco
