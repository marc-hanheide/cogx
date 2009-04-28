
#include "OpenGLControl.h"

OpenGLControl::OpenGLControl(){
	m_clearcolor = 0.0;
	m_cleardepth = 1000.0;
}

void OpenGLControl::Init(){
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
}

void OpenGLControl::SetClearOptions(float clearcolor, float cleardepth){
	m_clearcolor = clearcolor;
	m_cleardepth = cleardepth;
}

void OpenGLControl::RenderSettings(bool color, bool depth){
	glDepthMask(depth);
	glColorMask(color,color,color,color);
}

void OpenGLControl::ClearBuffers(bool color, bool depth){
	
	if(color){
		glClearColor(m_clearcolor, m_clearcolor, m_clearcolor, m_clearcolor);
		glClear(GL_COLOR_BUFFER_BIT);
	}
	
	if(depth){
		glClearDepth(m_cleardepth);
		glClear(GL_DEPTH_BUFFER_BIT);
	}
}