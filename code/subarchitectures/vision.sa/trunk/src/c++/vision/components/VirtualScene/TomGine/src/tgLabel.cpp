

#include "tgLabel.h"
#include <stdexcept>

using namespace TomGine;

tgLabel::tgLabel(float w, float h, const char* ttf_filename){
	m_font = new tgFont(ttf_filename);
	m_width = w;
	m_height = h;
}

tgLabel::~tgLabel(){
	delete(m_font);
}

void tgLabel::Create(const char* text){
	
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	
// 	glClear(GL_COLOR_BUFFER_BIT);
	
	m_font->Print(text, 16, 2, 5);
	
	m_texture.CopyTexImage2D(64, 20);
	
	glDepthMask(1);
	glEnable(GL_DEPTH_TEST);
}

void tgLabel::Draw(){
	float w=m_width;
	float h=m_height;
	
	glEnable(GL_TEXTURE_2D);
	m_texture.Bind();
	
	glColor3f(1.0,0.0,0.0);
	glBegin(GL_QUADS);
		glTexCoord2f(0.0,0.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f(0.0, 0.0, 0.0);
		glTexCoord2f(1.0,0.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f(  w, 0.0, 0.0);
		glTexCoord2f(1.0,1.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f(  w,   h, 0.0);
		glTexCoord2f(0.0,1.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f(0.0,   h, 0.0);
	glEnd();
	
	glDisable(GL_TEXTURE_2D);
}

