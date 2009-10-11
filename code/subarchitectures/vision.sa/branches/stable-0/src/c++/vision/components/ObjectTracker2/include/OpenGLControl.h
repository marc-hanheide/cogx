
#ifndef __OPENGL_CONTROL_H__
#define __OPENGL_CONTROL_H__

#include <stdio.h>
#include <GL/gl.h>
#include <string.h>

class OpenGLControl
{
private:
	float m_clearcolor;
	float m_cleardepth;
	
public:
	OpenGLControl();
	
	bool Init();
	void SetClearOptions(float clearcolor, float cleardepth);
	
	void RenderSettings(bool color, bool depth);
	void ClearBuffers(bool color, bool depth);

};

#endif