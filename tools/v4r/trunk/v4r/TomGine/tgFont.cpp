

#include "tgFont.h"
#include "headers.h"
#include <stdexcept>
#include <stdio.h>

using namespace TomGine;

tgFont::tgFont(){
#ifdef USE_FTGL_FONT
	m_font = new FTBitmapFont(TTF_FONT);	// FTPixmapFont(TTF_FONT) -> better, but buggy
	if(m_font->Error()){
		char err[64];
		sprintf(err, "[tgFont::tgFont()] Error cannot create font '%s'", TTF_FONT);
		throw std::runtime_error(err);
	}
#else
	printf("[tgFont::tgFont] Warning: ftgl fonts disabled. USE_FTGL_FONT not defined\n");
#endif
}

tgFont::~tgFont(){
#ifdef USE_FTGL_FONT
	if(m_font)
		delete(m_font);
#endif
}

void tgFont::Print(	const char* text, int size,
					int x, int y,
					float r, float g, float b, float a) const
{
#ifdef USE_FTGL_FONT
	glColor4f(r,g,b,a);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_font->FaceSize(size);
	//m_font->CharMap(ft_encoding_unicode);
	m_font->Render(text, -1, FTPoint(x,y));
	glDisable(GL_BLEND);
#endif
}
