
#include "Texture.h"
#include "Resources.h"

using namespace Tracking;

Texture::Texture(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
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
	glTexImage2D(GL_TEXTURE_2D, 0, 3, m_width, m_height, 0, GL_BGR, GL_UNSIGNED_BYTE, image_data);
	return true;
}

bool Texture::load(const char* filename){
	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
	return load((unsigned char*)img->imageData, img->width, img->height);
}

bool Texture::save(const char* filename){
	bind();
	IplImage* img = cvCreateImage ( cvSize ( m_width, m_height ), IPL_DEPTH_8U, 3 );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, img->imageData);
	cvConvertImage(img, img, CV_CVTIMG_SWAP_RB);
	cvSaveImage(filename, img);
	cvReleaseImage(&img);
}

void Texture::bind(int stage){
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glActiveTexture(GL_TEXTURE0);
}


void Texture::copyTexImage2D(int width, int height){
	copyTexImage2D(0, 0, width, height);	
}

void Texture::copyTexImage2D(int x, int y, int width, int height){
	m_width = width;
	m_height = height;
	bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, x, y, m_width, m_height, 0);	
}

void Texture::copyFromTexture(Texture* tex){
	g_Resources->GetImageProcessor()->render(tex);
	copyTexImage2D(tex->getWidth(), tex->getHeight());
}

void Texture::copyFromTexture(Texture* tex, int x, int y, int w, int h){
	g_Resources->GetImageProcessor()->render(tex);
	copyTexImage2D(x, y, w, h);
}

