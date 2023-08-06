#pragma once

#include <coco/platform/Gui.hpp>


namespace coco {

/**
 * LED strip on the emulator gui.
 * Usage: gui.draw<GuiLedStrip>(buffer, length);
 */
class GuiLedStrip : public Gui::Renderer {
public:
    GuiLedStrip();

    float2 draw(float2 position);
    float2 draw(float2 position, const uint8_t *buffer, int length);

protected:
    GLuint texture;
};

} // namespace coco
