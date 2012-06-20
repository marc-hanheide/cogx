
#include "ImageProcessor.h"

using namespace Tracking;


// Load and compile shaders and set parameters
bool ImageProcessor::initShader(){
	int id;
	float w = (float)m_width;
	float h = (float)m_height;
	float hi,lo;
	float sq2 = 1.0f/sqrt(2.0f);
	
	// offsets of neighbouring pixels in texture coordinates
	GLfloat offX[9] = { -1.0/w, 0.0, 1.0/w,
											-1.0/w, 0.0, 1.0/w,
											-1.0/w, 0.0, 1.0/w };
	GLfloat offY[9] = {  1.0/h, 1.0/h,  1.0/h,
												0.0,   0.0,    0.0,
											-1.0/h,-1.0/h, -1.0/h };
	GLfloat dist[9] = { sq2, 1.0, sq2,
											1.0, 0.0, 1.0,
											sq2, 1.0, sq2 };
	GLfloat kernel[25] = {	2,  4,  5,  4, 2,
							4,  9, 12,  9, 4,
							5, 12, 15, 12, 5,
							4,  9, 12,  9, 4,
							2,  4,  5,  4, 2 };
	hi = 10.0/22; // = sqrt((3+10+3)^2 + (3+10+3)^2) = 22.6
	lo = 3.0/22; 
	GLfloat sobelX[9] = {   -lo, 	0.0, 	lo,
													-hi, 	0.0, 	hi,
													-lo,  	0.0, 	lo };
	// dont modify structure of sobelY -> division in sobel.frag
	GLfloat sobelY[9] = {    lo,	hi,     lo,
														0.0,	0.0,	0.0,
													-lo,   -hi,	    -lo };
	
	// Gauss shader
	id = g_Resources->AddShader("gauss", NULL, "gauss.frag");
	m_shadeGauss = g_Resources->GetShader(id);
	m_shadeGauss->bind();
	m_shadeGauss->setUniform( "kernel", 25, kernel );
	m_shadeGauss->setUniform( "width", w);
	m_shadeGauss->setUniform( "height", h);
	m_shadeGauss->unbind();
	
	// Sobel shader
	id = g_Resources->AddShader("sobel", NULL, "sobel.frag");
	m_shadeSobel = g_Resources->GetShader(id);
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSobel->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelX", mat3(sobelX), GL_FALSE );
	m_shadeSobel->setUniform( "mSobelY", mat3(sobelY), GL_FALSE );
	m_shadeSobel->setUniform( "fThreshold", float(SOBEL_THRESHOLD) );
	m_shadeSobel->unbind();
	
	// Thinning shader
	id = g_Resources->AddShader("thinning", NULL, "thinning.frag");
	m_shadeThinning = g_Resources->GetShader(id);
	m_shadeThinning->bind();
	m_shadeThinning->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeThinning->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeThinning->setUniform( "fThreshold", float(THINNING_THRESHOLD) );
	m_shadeThinning->unbind();
	
	// Spreading shader
	id = g_Resources->AddShader("spreading", NULL, "spreading.frag");
	m_shadeSpreading = g_Resources->GetShader(id);
	m_shadeSpreading->bind();
	m_shadeSpreading->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeSpreading->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeSpreading->setUniform( "mDistance", mat3(dist), GL_FALSE );
	m_shadeSpreading->setUniform( "fThreshold", float(SPREADING_THRESHOLD) );
	m_shadeSpreading->setUniform( "fDistScale", float(DISTANCE_SCALING) );
	m_shadeSpreading->unbind();
	
	return true;
}

// Display list for normal images (non-rectificating TexCoords)
bool ImageProcessor::dlImage(){
    float x = float(m_width)/2.0;
    float y = float(m_height)/2.0;

    glBegin(GL_QUADS);
		glColor3f(1.0,1.0,1.0);
        glTexCoord2f(0,0); glVertex3f(-x,-y, 0.0);
        glTexCoord2f(1,0); glVertex3f( x,-y, 0.0);
        glTexCoord2f(1,1); glVertex3f( x, y, 0.0);
        glTexCoord2f(0,1); glVertex3f(-x, y, 0.0);
    glEnd();
    
    return true;
}

bool ImageProcessor::dlImage(int x, int y, int w, int h){

	glBegin(GL_QUADS);
	glColor3f(1.0,1.0,1.0);
			glTexCoord2f(0,0); glVertex3f(x,   y, 0.0);
			glTexCoord2f(1,0); glVertex3f(x+w, y, 0.0);
			glTexCoord2f(1,1); glVertex3f(x+w, y+h, 0.0);
			glTexCoord2f(0,1); glVertex3f(x,   y+h, 0.0);
	glEnd();
	
	return true;
}

// Display list for flipping image upside down
bool ImageProcessor::dlFlipUpsideDown(){
    float x = float(m_width)/2.0;
    float y = float(m_height)/2.0;
    
    glBegin(GL_QUADS);
		glTexCoord2f(0,1); glVertex3f(-x,-y, 0.0);
        glTexCoord2f(1,1); glVertex3f( x,-y, 0.0);
        glTexCoord2f(1,0); glVertex3f( x, y, 0.0);
        glTexCoord2f(0,0); glVertex3f(-x, y, 0.0);
    glEnd();
    
    return true;
}

// Display list for rectificating image (rectificating TexCoords)
bool ImageProcessor::dlRectification(){
    int i,j,n=10;
    double x,y;
    int w = m_width;
    int h = m_height;
    
    // Lens rectificated texture coordinates
    glBegin(GL_QUADS);
        for (i=0;i<w;i+=n) {
            for (j=0;j<h;j+=n) {
                transform(i,j,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f(i-w/2.0,j-h/2.0,0.0);
                
                transform(i+n,j,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f((i+n)-w/2.0,j-h/2.0,0.0);
                
                transform(i+n,j+n,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f((i+n)-w/2.0,(j+n)-h/2.0,0.0);
                
                transform(i,j+n,&x,&y);
                glTexCoord2f(x,y);
                glVertex3f(i-w/2.0,(j+n)-h/2.0,0.0);
            }
        } 
    glEnd();    
	return true;
}

// Gets rectificated texture coordinates ix, iy for pixel at position i,j
bool ImageProcessor::transform(int i,int j,double *ix,double *iy){
    float a,b,c,d;
    double x,y,xnew,ynew;
    double r,theta,rnew,thetanew;
    int w = m_width;
    int h = m_height;

    x = i / (w/2.0) - 1;
    y = j / (h/2.0) - 1;
    r = sqrt(x*x+y*y);
    theta = atan2(y,x);
    
    switch (m_lensMode) {
    case NONE:
        xnew = x;
        ynew = y;
        break;
    case BARREL:
        a = -0.005;
        b = 0.0;
        c = 0.0;
        d = 1.0;
        rnew = (a*r*r*r + b*r*r + c*r + d) * r;
        thetanew = theta;
        xnew = rnew * cos(thetanew);
        ynew = rnew * sin(thetanew);
        break;
    }
    *ix = (xnew + 1) / 2.0;
    *iy = (ynew + 1) / 2.0;
    
    return true;
}

ImageProcessor::ImageProcessor(){
	m_shadeSobel = 0;
	m_shadeThinning = 0;
	m_shadeSpreading = 0;
}

ImageProcessor::~ImageProcessor(){
	glDeleteLists(m_dlRect, 1);
	glDeleteLists(m_dlImage, 1);
	glDeleteLists(m_dlUpsideDown, 1);
}

// Set functions
void ImageProcessor::setCamOrtho(){ 
	m_cam_ortho.Activate();
}

// *** Image Processing functions ***
void ImageProcessor::flipUpsideDown(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlUpsideDown);
		result->copyTexImage2D(source->getWidth(), source->getHeight());
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::copy(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::rectification(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlRect);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::gauss(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	m_shadeGauss->bind();
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
    m_shadeGauss->unbind();
}

void ImageProcessor::sobel(Texture* source, Texture* result, float threshold, bool normalise){
	m_cam_ortho.Activate();
	m_shadeSobel->bind();
	m_shadeSobel->setUniform( "fThreshold", threshold );
	m_shadeSobel->setUniform( "norm", normalise);
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
  m_shadeSobel->unbind();
}

void ImageProcessor::thinning(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		m_shadeThinning->bind();
			glCallList(m_dlImage);
		m_shadeThinning->unbind();
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::spreading(Texture* source, Texture* result){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		m_shadeSpreading->bind();
		glCallList(m_dlImage);
		m_shadeSpreading->unbind();
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);;
}

void ImageProcessor::render(Texture* tex){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		tex->bind();
		glCallList(m_dlImage);
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::render(Texture* tex, int x, int y, int w, int h){
	m_cam_ortho.Activate();
	glEnable(GL_TEXTURE_2D);
		tex->bind();
		dlImage(x, y, w, h);
	glDisable(GL_TEXTURE_2D);
}

// Main initialisation function
bool ImageProcessor::init(int w, int h){
    
	m_width = w;
	m_height = h;
	
	// Initialise camera
	m_cam_ortho.Set(	0.0, 0.0, 1.0,
										0.0, 0.0, 0.0,
										0.0, 1.0, 0.0,
										45, w, h,
										0.1, 10.0,
										GL_ORTHO);
	
	// Initialize shaders
	if(!initShader())
		return false;
    
    
	// Setup display lists
  m_dlRect = glGenLists(1);
	m_lensMode = BARREL;
	glNewList(m_dlRect, GL_COMPILE);
  	dlRectification();
	glEndList();
	
  m_dlImage = glGenLists(1);
	glNewList(m_dlImage, GL_COMPILE);
  	dlImage();
	glEndList();
	
	m_dlUpsideDown = glGenLists(1);
	glNewList(m_dlUpsideDown, GL_COMPILE);
		dlFlipUpsideDown();
	glEndList();
	
	return true;
}

