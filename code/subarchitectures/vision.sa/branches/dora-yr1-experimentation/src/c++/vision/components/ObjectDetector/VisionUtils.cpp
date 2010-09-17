/**
 * @file VisionUtils.cpp
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Utils for the vision system
 **/


#include <VisionUtils.h>
#include <Color.hh>

namespace cast
{

/**
 * Convert from framework imge type to vs3 image class.
 * note: returned image must be cvRelease'd
 */
Z::Image* convertImageToZImage(const Video::Image & image)
{

	// Copy image data
	char *d = new char[3 * image.width * image.height];

	for(unsigned i=0; i<3 * image.width * image.height; i++)
		d[i] = image.data[i];

  Z::Image *zImage = new Z::Image(d, (unsigned) image.width, (unsigned) image.height, (unsigned) 3, Z::String2ColorFormat("RGB24"), true, 1920);

  assert(zImage != 0);
  return zImage;
}

}

