

#include "tgFont.h"
#include <stdexcept>

using namespace TomGine;

tgFont::tgFont(const char* ttf_filename){
	m_font = new FTGLPixmapFont(ttf_filename);
	if(m_font->Error())
		throw std::runtime_error("[tgFont::tgFont()] Cannot create Font");
}

tgFont::~tgFont(){
	if(m_font)
		delete(m_font);
}

void tgFont::Print(const char* text, int size, int pos_x, int pos_y){
	m_font->FaceSize(size);
	m_font->Render(text, -1, FTPoint(pos_x,pos_y));
}