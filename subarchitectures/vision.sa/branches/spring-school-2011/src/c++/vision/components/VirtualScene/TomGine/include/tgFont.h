 /**
 * @file tgFont.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @link file:///usr/share/doc/libftgl-dev/html/index.html FTGL User Guide
 * @brief Main file of rendering engine 'TomGine'.
 * @namespace TomGine
 */
 
#ifndef _TG_FONT_H_
#define _TG_FONT_H_
 
#include <FTGL/ftgl.h>

namespace TomGine{

class tgFont
{
private:
	FTFont* m_font;

public:
	tgFont(const char* ttf_filename);
	~tgFont();
	
	void Print(const char* text, int size, int pos_x, int pos_y);

};

}

#endif
