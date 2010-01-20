/**
 * @author Michael Zillich
 * @date June 2009
 */

#ifndef ACCESS_IMAGE_H
#define ACCESS_IMAGE_H

#include <cv.h>
#include "Video.hpp"

namespace Video
{

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

