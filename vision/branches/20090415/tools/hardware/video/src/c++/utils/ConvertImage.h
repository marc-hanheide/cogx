/**
 * @author Michael Zillich
 * @date Februrary 2009
 */

#ifndef CONVERT_IMAGE_H
#define CONVERT_IMAGE_H

#include <stdexcept>
#include <opencv/cv.h>
#include "Video.hpp"

namespace Video
{

using namespace std;

/**
 * Convert from framework imge type to an OpenCV image.
 * note: returned image must be cvRelease'd
 */
IplImage* convertImageToIpl(const Video::Image & image);

/**
 * Convert from framework imge type to an OpenCV grayscale image.
 * note: returned image must be cvRelease'd
 */
IplImage* convertImageToIplGray(const Video::Image & image);

/**
 * Converts an IplImage to a system Image format.
 * note: If img is of appropriate size already, no extra memory allocation takes
 * place.
 */
void convertImageFromIpl(const IplImage *iplImg, Video::Image &img)
  throw(runtime_error);

}

#endif

