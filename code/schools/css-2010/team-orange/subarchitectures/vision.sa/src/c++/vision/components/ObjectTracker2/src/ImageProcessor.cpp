
#include "ImageProcessor.h"

ImageProcessor::ImageProcessor(){
	m_shadeGauss = 0;
    m_shadeSobel = 0;
    m_shadeThinning = 0;
    m_shadeSpreading = 0;
    m_shadeSpreadingUniform = 0;
    
    int id;
    if((id = g_Resources->AddCamera("cam_orthoGL")) == -1)
		exit(1);
	m_cam_orthoGL = g_Resources->GetCamera(id);
}

ImageProcessor::~ImageProcessor(){
    glDeleteLists(m_dlRect, 1);
    glDeleteLists(m_dlImage, 1);
    glDeleteLists(m_dlUpsideDown, 1);
}

// Load and compile shaders and set parameters
bool ImageProcessor::initShader(){
	int id;
    float w = (float)m_width;
    float h = (float)m_height;
    float hi,lo;
    float sq2 = 1/sqrt(2);
    
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
    hi = 10.0/32.0;
    lo = 3.0/32.0; 
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
    
    // Spreading shader
    id = g_Resources->AddShader("spreadingUniform", NULL, "spreadinguniform.frag");
    m_shadeSpreadingUniform = g_Resources->GetShader(id);
    m_shadeSpreadingUniform->bind();
    m_shadeSpreadingUniform->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
    m_shadeSpreadingUniform->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
    m_shadeSpreadingUniform->setUniform( "mDistance", mat3(dist), GL_FALSE );
    m_shadeSpreadingUniform->setUniform( "fThreshold", float(0.0) );
    m_shadeSpreadingUniform->setUniform( "fDistScale", float(DISTANCE_SCALING) );
    m_shadeSpreading->unbind();
    
    id = g_Resources->AddShader("summation", NULL, "summation.frag");
    m_shadeSummation = g_Resources->GetShader(id);
    m_shadeSummation->bind();
    m_shadeSummation->setUniform("num_pixels", int(255));
    m_shadeSummation->unbind();
    
    return true;
}

// Display list for normal images (non-rectificating TexCoords)
bool ImageProcessor::dlImage(){
    float x = float(m_width)/2.0;
    float y = float(m_height)/2.0;
    
    glBegin(GL_QUADS);
        glTexCoord2f(0,0); glVertex3f(-x,-y, 0.0);
        glTexCoord2f(1,0); glVertex3f( x,-y, 0.0);
        glTexCoord2f(1,1); glVertex3f( x, y, 0.0);
        glTexCoord2f(0,1); glVertex3f(-x, y, 0.0);
    glEnd();
    
    return true;
}

// Display list for normal images (non-rectificating TexCoords)
bool ImageProcessor::dlQuad(int exp){
    float a = float(pow(2,exp))*0.5;
    
    glBegin(GL_QUADS);
        glTexCoord2f(0,0); glVertex3f(-a,-a, 0.0);
        glTexCoord2f(1,0); glVertex3f( a,-a, 0.0);
        glTexCoord2f(1,1); glVertex3f( a, a, 0.0);
        glTexCoord2f(0,1); glVertex3f(-a, a, 0.0);
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

void ImageProcessor::callDisplayList(){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		glCallList(m_dlImage);
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::callQuadList(int exp){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		glCallList(m_dlQuad[exp]);
	glDisable(GL_TEXTURE_2D);
}

// *** Image Processing functions ***

void ImageProcessor::flipUpsideDown(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlUpsideDown);
		result->copyTexImage2D(source->getWidth(), source->getHeight());
	glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::copy(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::rectification(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlRect);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::gauss(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	m_shadeGauss->bind();
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
    m_shadeGauss->unbind();
}

void ImageProcessor::sobel(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	m_shadeSobel->bind();
    glEnable(GL_TEXTURE_2D);
		source->bind();
		glCallList(m_dlImage);
    	result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
    m_shadeSobel->unbind();
}

void ImageProcessor::thinning(Texture* source, Texture* result){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		source->bind();
		m_shadeThinning->bind();
			glCallList(m_dlImage);
		m_shadeThinning->unbind();
		result->copyTexImage2D(source->getWidth(), source->getHeight());
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::spreading(Texture* source, Texture* result, int num_passes){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
	m_shadeSpreading->bind();
	source->bind();
		for(int i=0; i<num_passes; i++){
			glCallList(m_dlImage);
			result->copyTexImage2D(source->getWidth(), source->getHeight());
		}
	m_shadeSpreading->unbind();
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::spreadingUniform(Texture* source, Texture* result, int num_passes){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
	m_shadeSpreadingUniform->bind();
	source->bind();
		for(int i=0; i<num_passes; i++){
			glCallList(m_dlImage);
			result->copyTexImage2D(source->getWidth(), source->getHeight());
		}
	m_shadeSpreadingUniform->unbind();
    glDisable(GL_TEXTURE_2D);
}

void ImageProcessor::render(Texture* tex){
	m_cam_ortho->Activate();
	glEnable(GL_TEXTURE_2D);
		tex->bind();
		glCallList(m_dlImage);
	glDisable(GL_TEXTURE_2D);
}

// calculates total luminance of framebuffer or image (given by source = slower)
DoubleList ImageProcessor::luminance(Texture* result, int start_LOD, int end_LOD, Texture* source){	
	int exp = start_LOD;
	float c;
	int a = pow(2,exp);
	int b = pow(2,exp-1);
	unsigned int elod = pow(2,end_LOD);
	unsigned int slod = pow(2,start_LOD);
	int w2 = int(float(m_glWidth) * 0.5);
	int h2 = int(float(m_glHeight) * 0.5);
	
	vector<unsigned char> vbSum;
	vbSum.resize(4*elod*elod,0);
	DoubleList vfResult;
	vfResult.resize(elod*elod,0.0);
	
	if(slod > m_glWidth || slod > m_glHeight){
		printf("[ImageProcessor::luminance] Warning LOD bigger than gl-buffer size!");
	}
	
	m_cam_orthoGL->Activate();
	glEnable(GL_TEXTURE_2D);
	
		// source == NULL means image is already in the framebuffer
		if(source!=NULL){
			// Copy source image to texture with size 2^n x 2^n
			source->bind();
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			if(a > source->getWidth()){
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			}else{
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			}
			glCallList(m_dlQuad[exp]);
		}
		
		// setup texture for processing
		result->bind();
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		
		// downsample images
		m_shadeSummation->bind();
			m_shadeSummation->setUniform("first", 1);							// first run (shader converts colors to internal format)
			glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w2-b, h2-b, a, a, 0);	// copy framebuffer to texture
			exp--;			// downscale qaud (=2^exp)
			a = pow(2,exp);	// get sidelength of new quad
			b = a*0.5; 		//pow(2,exp-1);
			c = a*4.0; 		//pow(2,exp+2);
			m_shadeSummation->setUniform("d", float(1.0/c));
			glCallList(m_dlQuad[exp]);
			
			// successive downsampling and averaging of 4 pixels each step
			m_shadeSummation->setUniform("first", 0);
			while(exp>1 && exp>end_LOD){
				glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w2-b, h2-b, a, a, 0);
				exp--;
				a = pow(2,exp);
				b = a*0.5; //pow(2,exp-1);
				c = a*4.0; //pow(2,exp+2);
				m_shadeSummation->setUniform("d", float(1.0/c));
				glCallList(m_dlQuad[exp]);
			}
			
			// last LOD (Level Of Detail) needs special treatment in the shader (tex-coords)
			if(end_LOD == 0){
				glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w2-b, h2-b, a, a, 0);
				m_shadeSummation->setUniform("first", 2);
				exp--;
				a = pow(2,exp);
				b = 1.0; //pow(2,exp-1);
				c = 4.0; //pow(2,exp+2);
				m_shadeSummation->setUniform("d", float(1.0/c));
				glCallList(m_dlQuad[exp]);
			}
			
		m_shadeSummation->unbind();
		
		// copy result to memory (pixel of lowest level of detail image)
		//unsigned char bSum[4];
		//glReadPixels(w2-b, h2-b, a, a, GL_RGBA, GL_UNSIGNED_BYTE, bSum);
		//printf("%i %i %i %i\n", int(bSum[0]),int(bSum[1]),int(bSum[2]),int(bSum[3]));
		
		glReadPixels(w2-b, h2-b, a, a, GL_RGBA, GL_UNSIGNED_BYTE, &(*vbSum.begin()));
		
		//calculate summation results;
		for(int i=0; i<(elod*elod); i++){
			vfResult[i] = vbSum[4*i] + vbSum[4*i+1]*0.01 + vbSum[4*i+2]*0.0001 + vbSum[4*i+3]*0.000001;
			//printf("%i %i %i %i\n", vbSum[4*i], vbSum[4*i+1], vbSum[4*i+2], vbSum[4*i+3]);
		}
 		
	glDisable(GL_TEXTURE_2D);
	
	
	return vfResult;
}

// Main initialisation function
bool ImageProcessor::init(int w, int h, Camera* cam){
    
    m_width = w;
    m_height = h;
    m_cam_ortho = cam;
    
    m_glWidth = g_Resources->GetScreen()->w;
    m_glHeight = g_Resources->GetScreen()->h;
   	
   	// Initialize shaders
    if(!initShader()){
        return false;
    }
    
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
	
	for(int i=0; i<=MAX_EXPONENT; i++){
		m_dlQuad[i] = glGenLists(1);
		glNewList(m_dlQuad[i], GL_COMPILE);
			dlQuad(i);
		glEndList();
	}
	
	m_cam_orthoGL->Set(	0.0, 0.0, 1.0,
						0.0, 0.0, 0.0,
						0.0, 1.0, 0.0,
						45, m_glWidth, m_glHeight,
						0.1, 10.0,
						GL_ORTHO);
	
	// TODO: determine Buffer with highest precision
	// and forward result to copy shader for summation
	
    return true;
}

