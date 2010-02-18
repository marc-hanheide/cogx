/**
 * @author Michael Zillich
 * @date Februrary 2009
 * 
 * TODO convert image size!
 */

#ifndef CONVERT_IMAGE_H
#define CONVERT_IMAGE_H

#include <stdexcept>
#include <cv.h>
#include "Video.hpp"

namespace Video
{

using namespace std;

/**
 * Convert from framework image type to an OpenCV image type (IplImage).
 * note: If iplImg is non-null and of appropriate size already, it will be used
 * as is. If it is non-null but has wrong size it will be released and
 * reallocated. If it is null it will be allocated.
 * Of course you are responsible for releasing the iplImg later on.
 */
void convertImageToIpl(const Video::Image & img, IplImage ** iplImg);

/**
 * Convert from framework image type to an OpenCV image.
 * note: returned image must be cvRelease'd
 * A convenience form of the above.
 */
IplImage* convertImageToIpl(const Video::Image & image);

/**
 * Convert from framework image type to an OpenCV grayscale image type
 * (IplImage).
 * note: returned image must be cvRelease'd
 */
IplImage* convertImageToIplGray(const Video::Image & image);

/**
 * Converts an OpenCV image type (IplImage) to a system Image format.
 * note: If img is of appropriate size already, no extra memory allocation takes
 * place.
 */
void convertImageFromIpl(const IplImage *iplImg, Video::Image &img)
  throw(runtime_error);

/**
 * Swap red and blue channel.
 */
void SwapRedBlueChannel(Video::Image & img);

}

#endif

