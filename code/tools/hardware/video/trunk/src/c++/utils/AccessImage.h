/**
 * @author Michael Zillich
 * @date June 2009
 */

#ifndef ACCESS_IMAGE_H
#define ACCESS_IMAGE_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Video.hpp"

namespace Video
{

/**
 * access an image as img[y][x]
 *
 * (by Gady Agam, http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/index.html)

  # For a single-channel byte image:

  IplImage* img=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
  BwImage imgA(img);
  imgA[i][j] = 111;

  # For a multi-channel byte image:

  IplImage* img=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
  RgbImage  imgA(img);
  imgA[i][j].b = 111;
  imgA[i][j].g = 111;
  imgA[i][j].r = 111;

  # For a multi-channel float image:

  IplImage* img=cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,3);
  RgbImageFloat imgA(img);
  imgA[i][j].b = 111;
  imgA[i][j].g = 111;
  imgA[i][j].r = 111;

 */
template<class T> class AccessImage
{
private:
  const IplImage* imgp;
public:
  AccessImage(const IplImage* img=0) {imgp=img;}
  ~AccessImage(){imgp=0;}
  void operator=(IplImage* img) {imgp=img;}
  //const T* operator[](const int rowIndx) const {
  //  return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));}
  T* operator[](const int rowIndx) const {
    return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));}
};

typedef struct{
  unsigned char b,g,r;
} AccessRgbPixel;

typedef struct{
  float b,g,r;
} AccessRgbPixelFloat;

typedef AccessImage<AccessRgbPixel>       AccessRgbImage;
typedef AccessImage<AccessRgbPixelFloat>  AccessRgbImageFloat;
typedef AccessImage<unsigned char>  AccessBwImage;
typedef AccessImage<float>          AccessBwImageFloat;

/**
 * Access raw image data at position (x,y), where x and y are not checked for
 * valid range.
 * const version.
 */
inline const char* cvAccessImageData(const IplImage *img, int x, int y)
{
  return img->imageData + y*img->widthStep + x*img->nChannels*img->depth/8;
}

/**
 * Access raw image data at position (x,y), where x and y are not checked for
 * valid range.
 * non-const version.
 */
inline char* cvAccessImageData(IplImage *img, int x, int y)
{
  return img->imageData + y*img->widthStep + x*img->nChannels*img->depth/8;
}

}

#endif

