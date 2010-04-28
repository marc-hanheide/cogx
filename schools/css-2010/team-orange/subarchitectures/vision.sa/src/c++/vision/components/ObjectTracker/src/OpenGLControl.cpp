
#include "OpenGLControl.h"

OpenGLControl::OpenGLControl(){
	m_clearcolor = 0.0;
	m_cleardepth = 1000.0;
}

bool OpenGLControl::Init(){
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
		
	const GLubyte *str;
	int glOcclusionQueryAvailable;
	
	// Check for OpenGL and GLSL
	
	// Check for Extension
	str = glGetString(GL_EXTENSIONS);
	glOcclusionQueryAvailable = (strstr((const char *)str, "GL_ARB_occlusion_query") != NULL);
	if(!glOcclusionQueryAvailable){ 
		printf("[OpenGLControl] Error OpenGL extension 'GL_ARB_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!\n");
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
