#include <coco/BufferImpl.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/IntrusiveQueue.hpp>
#include <coco/platform/Loop_emu.hpp>
#include <string>


namespace coco {

/**
 * Implementation of a LED strip emulator that shows the LED strip on the emulator gui
 */
class LedStrip_emu : public BufferDevice, public Loop_emu::GuiHandler {
public:
	LedStrip_emu(Loop_emu &loop);
	~LedStrip_emu() override;

	/**
	 * Buffer for transferring data to a LED strip
	 */
	class Buffer : public IntrusiveListNode, public IntrusiveQueueNode, public BufferImpl {
		friend class LedStrip_emu;
	public:
		/**
		 * Constructor
		 * @param length length of emulated LED strip, i.e. number of RGB triples
		 * @param device emulator device
		 */
		Buffer(int length, LedStrip_emu &device);
		~Buffer() override;

		// Buffer methods
		bool start(Op op) override;
		bool cancel() override;

	protected:

		LedStrip_emu &device;
	};

	// Device methods
    StateTasks<const State, Events> &getStateTasks() override;

	// BufferDevice methods
	int getBufferCount() override;
	Buffer &getBuffer(int index) override;

protected:
	void handle(Gui &gui) override;

	Loop_native &loop;

    // device state
    StateTasks<Device::State, Device::Events> st = Device::State::READY;

	// list of buffers
	IntrusiveList<Buffer> buffers;

	// list of active transfers
	IntrusiveQueue<Buffer> transfers;
};

} // namespace coco
