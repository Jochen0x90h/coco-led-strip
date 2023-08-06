#include <coco/BufferDevice.hpp>
#include <coco/IntrusiveQueue.hpp>
#include <coco/platform/Loop_native.hpp>
#include <string>


namespace coco {

/**
 * Implementation of a LED strip emulator that shows the LED strip on the console using std::cout
 */
class LedStrip_cout : public BufferDevice {
public:
    LedStrip_cout(Loop_native &loop);
    ~LedStrip_cout() override;

    /**
     * Buffer for transferring data to a LED strip
     */
    class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveQueueNode {
        friend class LedStrip_cout;
    public:
        /**
         * Constructor
         * @param length length of emulated LED strip, i.e. number of RGB triples
         * @param loop event loop
         */
        Buffer(int length, LedStrip_cout &device);
        ~Buffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:

        LedStrip_cout &device;
    };


    // BufferDevice methods
    int getBufferCount() override;
    Buffer &getBuffer(int index) override;

protected:
    void handle();

    struct Color {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    Loop_native &loop;
    TimedTask<Callback> callback;

    // list of buffers
    IntrusiveList<Buffer> buffers;

    // list of active transfers
    IntrusiveQueue<Buffer> transfers;
};

} // namespace coco
