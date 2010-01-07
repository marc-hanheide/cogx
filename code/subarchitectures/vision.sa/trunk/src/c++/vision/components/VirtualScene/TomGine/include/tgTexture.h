
#ifndef TG_TEXTURE
#define TG_TEXTURE

#include <GL/gl.h>
//#include <highgui.h>

class tgTexture
{
private:
	GLuint m_texture_id;
	int m_width;
	int m_height;
	

public:
	tgTexture();
	~tgTexture();
	
	bool Load(unsigned char* image_data, int width, int height);
	//bool Load(const char* filename);
	
	void Bind(int stage=0);
	void Unbind();

	void CopyTexImage2D(int width, int height);
	
	inline GLuint GetTextureID(){ return m_texture_id; }
	inline int GetWidth(){ return m_width; }
	inline int GetHeight(){ return m_height; }
	
};

#endif