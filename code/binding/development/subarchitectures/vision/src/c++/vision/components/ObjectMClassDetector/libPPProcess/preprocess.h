#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include <libAnnotation/annotationlist.h>
#include <libPPProcess/ppParams.h>
#include <opencv/cv.h>
#include <QtGui/QImage>

AnnotationList preProcess(AnnotationList& annotations, preAndPostProcessParams& param, const char* outDir = 0);
void prepareBoundingBoxes(Annotation& annotations, const preAndPostProcessParams& param, int imageWidth, int imageHeight, int windowWidth, int windowHeight, bool addJitter = false, bool checkBorder = true);
void reShapeBoundingBoxes(Annotation& annotation, const preAndPostProcessParams& param, int windowWidth, int windowHeight);
bool preProcessAnnotation(const AnnoRect& rect, const preAndPostProcessParams& param, const QImage& img, QImage& scaledObject);
void blurEdges(QImage &blurMe, const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad);
void drawRandomSamples(Annotation &negFile, int imageWidth, int imageHeight, int samplesPerImage, float scaleSteps, int noScales, int windowWidth, int windowHeight);
void blurEdges(unsigned char* data, unsigned int width, unsigned int height, unsigned int rowstep,
		        const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad);
bool adjustBoundingBox(const AnnoRect& rect, AnnoRect& adjustedRect, const preAndPostProcessParams& param);
void sharpen(uchar *img, int img_width, int img_height, int rowStep, int img_bpp, int sharpen_percent);

template <typename T>
inline T* cvPointerToDataAt(char* data, int x, int y, int rowstep, int channels, int channel = 0)
{
    return reinterpret_cast<T*>(data + ( channels * x + channel ) * sizeof(T)  + y*rowstep);
}

template <typename T>
inline T& cvDataAt(char* data, int x, int y, int rowstep, int channels, int channel = 0)
{
    return *cvPointerToDataAt<T>(data, x, y, rowstep, channels, channel);
}

template <typename T>
inline T* cvPointerToDataAt(IplImage* img, int x, int y, int channel = 0)
{
    return reinterpret_cast<T*>(img->imageData + ( img->nChannels * x + channel ) * sizeof(T)  + y*img->widthStep);
}
template <typename T>
inline T& cvDataAt(IplImage* img, int x, int y, int channel = 0)
{
    return *cvPointerToDataAt<T>(img, x, y, channel);
}

template <typename T>
void blurImageEdges(IplImage* img,
                   const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad)
{
    const unsigned int width  = img->width;
    const unsigned int height = img->height;
    const unsigned int channels = img->nChannels;
    //Blur on the top
    assert(channels < 4);
    T *destpixel,
      *pixel;
    for(int k = topmost - 1; k >= 0; k--)
        for(int l = leftmost; l <= rightmost; l++) {
            int left  = l - kernelRad;
            int right = l + kernelRad;
            if (left < leftmost)
                left = leftmost;
            if (right > rightmost)
                right = rightmost;

            float normFac = 1.0f / (right-left+1);
            destpixel = cvPointerToDataAt<T>(img, l, k);
            switch (channels) {
            case 3: destpixel[2] = 0;
            case 2: destpixel[1] = 0;
            case 1: destpixel[0] = 0;
            }
            for (unsigned int i = left; i <= right; i++) {
                pixel = cvPointerToDataAt<T>(img, i, k+1);
                switch (channels) {
                case 3: destpixel[2] += pixel[2];
                case 2: destpixel[1] += pixel[1];
                case 1: destpixel[0] += pixel[0];
                }
            }
            switch (channels) {
            case 3: destpixel[2] *= normFac;
            case 2: destpixel[1] *= normFac;
            case 1: destpixel[0] *= normFac;
            }
        }

    //Blur on the bottom
    for( unsigned int k = bottommost + 1; k <= height - 1; k++ )
        for(int l = leftmost; l <= rightmost; l++)  {
            int left  = l - kernelRad;
            int right = l + kernelRad;
            if (left < leftmost)
                left = leftmost;
            if (right > rightmost)
                right = rightmost;			
            float normFac = 1.0f / (right-left+1);
            destpixel = cvPointerToDataAt<T>(img, l, k);
            switch (channels) {
            case 3: destpixel[2] = 0;
            case 2: destpixel[1] = 0;
            case 1: destpixel[0] = 0;
            }
            for (unsigned int i = left; i <= right; i++) {
                pixel = cvPointerToDataAt<T>(img, i, k-1);
                switch (channels) {
                case 3: destpixel[2] += pixel[2];
                case 2: destpixel[1] += pixel[1];
                case 1: destpixel[0] += pixel[0];
                }
            }
            switch (channels) {
            case 3: destpixel[2] *= normFac;
            case 2: destpixel[1] *= normFac;
            case 1: destpixel[0] *= normFac;
            }
        }

    //Blur on the right
    for(unsigned int l = rightmost + 1; l < width; l++)  
        for(unsigned int k = topmost; k <= bottommost; k++ ) {		
            int top = k - kernelRad;
            int bottom = k + kernelRad;
            if (top < topmost)
                top = topmost;
            if (bottom > bottommost)
                bottom = bottommost;
            float normFac = 1.0f / (bottom - top + 1);
            destpixel = cvPointerToDataAt<T>(img, l, k);
            switch (channels) {
            case 3: destpixel[2] = 0;
            case 2: destpixel[1] = 0;
            case 1: destpixel[0] = 0;
            }
            for (unsigned int i = top; i <= bottom; i++) {
                pixel = cvPointerToDataAt<T>(img, l-1, i);
                switch (channels) {
                case 3: destpixel[2] += pixel[2];
                case 2: destpixel[1] += pixel[1];
                case 1: destpixel[0] += pixel[0];
                }
            }
            switch (channels) {
            case 3: destpixel[2] *= normFac;
            case 2: destpixel[1] *= normFac;
            case 1: destpixel[0] *= normFac;
            }
        }

    //Blur on the left
    for(int l = leftmost - 1; l >= 0; l--)  
        for(unsigned int k = topmost; k <= bottommost; k++ ) {		
            int top = k - kernelRad;
            int bottom = k + kernelRad;
            if (top < topmost)
                top = topmost;
            if (bottom > bottommost)
                bottom = bottommost;
            float normFac = 1.0f / (bottom - top + 1);
            destpixel = cvPointerToDataAt<T>(img, l, k);
            switch (channels) {
            case 3: destpixel[2] = 0;
            case 2: destpixel[1] = 0;
            case 1: destpixel[0] = 0;
            }
            for (unsigned int i = top; i <= bottom; i++) {
                pixel = cvPointerToDataAt<T>(img, l+1, i);
                switch (channels) {
                case 3: destpixel[2] += pixel[2];
                case 2: destpixel[1] += pixel[1];
                case 1: destpixel[0] += pixel[0];
                }
            }
            switch (channels) {
            case 3: destpixel[2] *= normFac;
            case 2: destpixel[1] *= normFac;
            case 1: destpixel[0] *= normFac;
            }
        }
} 
#endif /*PREPROCESS_H_*/
