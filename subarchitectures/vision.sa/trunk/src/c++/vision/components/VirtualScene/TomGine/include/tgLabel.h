 /**
 * @file tgLabel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Quad patch with texture created by copying text from frame buffer.
 * @namespace TomGine
 */
 
#ifndef _TG_LABEL_H_
#define _TG_LABEL_H_
 
#include <FTGL/ftgl.h>

#include "tgFont.h"
#include "tgTexture.h"

namespace TomGine{

class tgLabel
{
private:
	float m_width;
	float m_height;
	
	tgFont* m_font;
	tgTexture m_texture;

public:
	/** @brief Constructor of tgLabel
	*	@param w width of label in pixels
	* @param h height of label in pixels
	* @param ttf_filename file name of true type font (ttf) to use for text */
	tgLabel(float w, float h, const char* ttf_filename);
	~tgLabel();
	
	void Create(const char* text);
	void Draw();
	
};

}

#endif
