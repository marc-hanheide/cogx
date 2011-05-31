

#include "TomGine/tgFont.h"
#include "TomGine/headers.h"
#include <stdexcept>
#include <stdio.h>

using namespace TomGine;

tgFont::tgFont(){
#ifdef USE_FTGL_FONT
	m_font = new FTPixmapFont(TTF_FONT);
	if(m_font->Error()){
		char err[64];
		sprintf(err, "[tgFont::tgFont()] Cannot create font '%s'", TTF_FONT);
		throw std::runtime_error(err);
	}
#else
	printf("[tgFont::tgFont] Warning: ftgl fonts disabled. USE_FTGL_FONT not defined\n");
#endif
	printf("tgFont::tgFont() THIS SHOULD NOT HAPPEN TOO OFTEN!!!\n");
}

tgFont::~tgFont(){
	printf("tgFont::~tgFont() THIS SHOULD NOT HAPPEN!!!\n");
#ifdef USE_FTGL_FONT

	if(m_font)
		delete(m_font);
#endif
}

void tgFont::Print(const char* text, int size, int pos_x, int pos_y){
#ifdef USE_FTGL_FONT
m_font = new FTPixmapFont(TTF_FONT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	
printf("tgFont::Print A\n");
	if(m_font->Error()){
		printf("tgFont::Print m_font->Error()\n");
		return;
	}
printf("tgFont::Print B\n");
	
	m_font->FaceSize(size);
	//m_font->CharMap(ft_encoding_unicode);
	m_font->Render(text, -1, FTPoint(pos_x,pos_y));
	glDisable(GL_BLEND);
//delete(m_font);
#endif
}

void tgFont::Print(const char* text, int size, int pos_x, int pos_y, float x, float y, float z, float a){
#ifdef USE_FTGL_FONT
// 	glColor4f(x,y,z,a);
// 	glEnable(GL_BLEND);
// 	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 	m_font->FaceSize(size);
// 	//m_font->CharMap(ft_encoding_unicode);
// 	m_font->Render(text, -1, FTPoint(pos_x,pos_y));
// 	glDisable(GL_BLEND);
#endif
}
