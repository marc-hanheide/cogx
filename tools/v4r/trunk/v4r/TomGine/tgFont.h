 /**
 * @file tgFont.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @link file:///usr/share/doc/libftgl-dev/html/index.html FTGL User Guide
 */
 
#ifndef _TG_FONT_H_
#define _TG_FONT_H_

#include "tgSingleton.h"
 
#ifdef USE_FTGL_FONT
#include <FTGL/ftgl.h>
#endif

namespace TomGine{

#define g_font tgFont::GetInstance()

/** @brief Drawing fonts with OpenGL using FTGL. */
class tgFont
{
public:
	friend class tgSingleton <tgFont>;
	static tgFont* GetInstance(){
		return tgSingleton <tgFont>::GetInstance ();
	}

private:
#ifdef USE_FTGL_FONT
	FTFont* m_font;
#endif
protected:
	/** @brief Create the FTGL font. */
	tgFont();
	/** @brief Destroy the FTGL font. */
	~tgFont();

public:
	/** @brief Print a text in an OpenGL window.
	 *  @param text		The text to print as character array.
	 *  @param size		The size of the font in points.
	 *  @param x,y		The position of the font in image space.
	 *  @param r,g,b	The color of the font.
	 *  @param a		Transparency of the font.	 */
	void Print(	const char* text,
				int size, int x, int y,
				float r=1.0f, float g=1.0f, float b=1.0f, float a=1.0f) const;

};

}

#endif
