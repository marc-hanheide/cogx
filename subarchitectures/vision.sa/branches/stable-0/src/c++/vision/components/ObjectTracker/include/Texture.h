
#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include <stdio.h>
#include <opencv/highgui.h>
#include <GL/gl.h>
#include <GL/glu.h>

class Texture
{
private:
	GLuint m_texture_id;
	int m_width;
	int m_height;
	

public:
	Texture();
	~Texture();
	
	bool load(unsigned char* image_data, int width, int height);
	bool load(const char* filename);
	
	void bind(int stage=0);
	void copyTexImage2D(int width, int height);		// copy frame buffer pixels to texture
	
	inline GLuint getTextureID(){ return m_texture_id; }
	inline int getWidth(){ return m_width; }
	inline int getHeight(){ return m_height; }
	
	
};

#endif