
#ifndef __IMAGE_PROCESSOR_HPP__
#define __IMAGE_PROCESSOR_HPP__

class ImageProcessor;

#include <stdio.h>
#include <GL/gl.h>

#include "Shader.h"
#include "Texture.h"
#include "Camera.h"
#include "Resources.h"

#define NONE 0          // No rectification
#define BARREL 1        // Rectification using Barrel equation

#define SOBEL_THRESHOLD 0.05       // high -> less noise & less edges
#define THINNING_THRESHOLD 0.0     // low -> thin lines
#define SPREADING_THRESHOLD 0.01   // low -> wider lines
#define DISTANCE_SCALING 0.75      // distance to edge slope; low -> steep slope
#define SPREADING_LOOPS 2          // Number of pixels to spread lines

class ImageProcessor{
private:
    int m_width;        // Image width in pixels
    int m_height;       // Image height in pixels
    int m_dlRect;       // Display list for rectification (distorted TexCoords)
    int m_dlImage;      // Display List for normal images (less geometry)
    int m_dlUpsideDown; // Display List for flipping image upside down
    int m_lensMode;     // Enumeration for lens rectification algorithm (NONE, BARREL)
    
    Shader* m_shadeGauss;		// Fragment shader for blurring image using gaussian filter
    Shader* m_shadeSobel;       // Fragment shader for edge detection with sobel algorithm
    Shader* m_shadeThinning;    // Fragment shader for thinning edges
    Shader* m_shadeSpreading;   // Fragment shader for spreading edges
    
    Camera* m_cam_ortho;
    
    bool initShader();
    bool dlImage();
    bool dlFlipUpsideDown();
    bool dlRectification();
    bool transform(int i,int j,double *ix,double *iy);

public:
    
    ImageProcessor();
    ~ImageProcessor();
    
    // Image Processing functions
    void flipUpsideDown(Texture* source, Texture* result);
    void copy(Texture* source, Texture* result);
    void rectification(Texture* source, Texture* result);
    void gauss(Texture* source, Texture* result);
    void sobel(Texture* source, Texture* result);
    void thinning(Texture* source, Texture* result);
    void spreading(Texture* source, Texture* result);
    void render(Texture* tex);
    
    // Main functions
    bool init(int w, int h, Camera* cam);

};

#endif
