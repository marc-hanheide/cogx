
#include "Texture.h"

Texture::Texture(){
	glGenTextures(1, &m_texture_id);
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

Texture::~Texture(){
	glDeleteTextures(1, &m_texture_id);
}

bool Texture::load(unsigned char* image_data, int width, int height){
	m_width = width;
    m_height = height;
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
    return true;
}

bool Texture::load(const char* filename){
	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
	cvConvertImage(img, img, CV_CVTIMG_SWAP_RB);
	return load((unsigned char*)img->imageData, img->width, img->height);
}

void Texture::bind(int stage){
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
}


void Texture::copyTexImage2D(int width, int height){
	m_width = width;
	m_height = height;
	bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, m_width, m_height, 0);	
}

