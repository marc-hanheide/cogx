
#include "tgTexture.h"

using namespace TomGine;

tgTexture::tgTexture(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

tgTexture::~tgTexture(){
	glDeleteTextures(1, &m_texture_id);
}

bool tgTexture::Load(unsigned char* image_data, int width, int height){
	m_width = width;
	m_height = height;
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
	return true;
}

/* bool tgTexture::Load(const char* filename){
	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
	cvConvertImage(img, img, CV_CVTIMG_SWAP_RB);
	bool b = Load((unsigned char*)img->imageData, img->width, img->height);
	cvReleaseImage(&img);
	return b;
} */

void tgTexture::Bind(int stage){
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glEnable(GL_TEXTURE_2D);
}

void tgTexture::Unbind(){
	glDisable(GL_TEXTURE_2D);
}

void tgTexture::CopyTexImage2D(int width, int height){
	m_width = width;
	m_height = height;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, m_width, m_height, 0);	
}
