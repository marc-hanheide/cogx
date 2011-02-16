 /**
 * @file tgTexture.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Class for managing textures (load from file, bind, copy, ...).
 */
 
#ifndef TG_TEXTURE
#define TG_TEXTURE

#include <GL/gl.h>

namespace TomGine{

/**
* @brief Class tgTexture
*/
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

} // namespace TomGine

#endif