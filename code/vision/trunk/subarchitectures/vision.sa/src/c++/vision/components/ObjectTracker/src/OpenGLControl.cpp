
#include "OpenGLControl.h"

OpenGLControl::OpenGLControl(){
	m_clearcolor = 1.0;
	m_cleardepth = 1000.0;
}

bool OpenGLControl::Init(){
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	//glDisable(GL_CULL_FACE);
	
	const GLubyte *str;
	int glOcclusionQueryNVAvailable;
	
	str = glGetString(GL_EXTENSIONS);
	glOcclusionQueryNVAvailable = (strstr((const char *)str, "GL_NV_occlusion_query") != NULL);
	if(!glOcclusionQueryNVAvailable){
		printf("[OpenGLControl] Error OpenGL extension 'GL_NV_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!\n");
		return false;
	}
		
	return true;
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