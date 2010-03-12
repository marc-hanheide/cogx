
#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include "headers.h"

namespace Tracking{

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
	bool save(const char* filename);
	
	void bind(int stage=0);
	void copyTexImage2D(int width, int height);		// copy frame buffer pixels to texture
	void copyTexImage2D(int x, int y, int width, int height);
	void copyFromTexture(Texture* tex);
	void copyFromTexture(Texture* tex, int x, int y, int w, int h);
	
	inline GLuint getTextureID(){ return m_texture_id; }
	inline int getWidth(){ return m_width; }
	inline int getHeight(){ return m_height; }
	
	
};

} // namespace Tracking

#endif