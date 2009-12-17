
#ifndef TG_TEXTURE
#define TG_TEXTURE

#include <GL/gl.h>
#include <highgui.h>

class tgTexture
{
private:
	GLuint m_texture_id;
	int m_width;
	int m_height;
	

public:
	tgTexture();
	~tgTexture();
	
	bool load(unsigned char* image_data, int width, int height);
	bool load(const char* filename);
	
	void bind(int stage=0);
	void unbind();

	void copyTexImage2D(int width, int height);
	
	inline GLuint getTextureID(){ return m_texture_id; }
	inline int getWidth(){ return m_width; }
	inline int getHeight(){ return m_height; }
	
};

#endif