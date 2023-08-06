#include "GuiLedStrip.hpp"


namespace coco {

GuiLedStrip::GuiLedStrip()
	: Renderer("#version 330\n"
		"uniform sampler2D tex;\n"
		"in vec2 xy;\n"
		"out vec4 pixel;\n"
		"void main() {\n"
			"vec4 color = texture(tex, xy);\n"
			// use sqrt(color) to simulate linear led's on screen with gamma = 2.0
			//"vec4 linear = sqrt(color);\n"
			"vec4 linear = color;\n"
			"vec4 graph = max(1.0 - abs(1.0-color + 0.05 - xy.y * 2.0) * 20, 0);\n"
			// show as graph and color strip
			"pixel = graph + (xy.y > 0.6 ? linear : vec4(0));\n"
		"}\n")
{
	this->texture = Gui::createTexture(GL_LINEAR);
}

float2 GuiLedStrip::draw(float2 position) {
	const float2 size = {0.96f, 0.2f};

	setState(position, size);
	glBindTexture(GL_TEXTURE_2D, this->texture);
	drawAndResetState();
	glBindTexture(GL_TEXTURE_2D, 0);

	return size;
}

float2 GuiLedStrip::draw(float2 position, const uint8_t *buffer, int length) {
	const float2 size = {0.96f, 0.2f};

	setState(position, size);
	glBindTexture(GL_TEXTURE_2D, this->texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, length, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, buffer);
	drawAndResetState();
	glBindTexture(GL_TEXTURE_2D, 0);

	return size;
}

} // namespace coco
