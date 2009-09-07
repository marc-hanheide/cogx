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
// using namespace Z;

/**
 * Convert from framework imge type to vs3 image class.
 * note: returned image must be cvRelease'd
 */
Z::Image* convertImageToZImage(const Video::Image & image)
{

	// Copy image data
// printf("Image-Size: bytes per line: %u\n", 3 * image.width * image.height);
	char *d = new char[3 * image.width * image.height];
// 	for(y = 0; y < image.height; y++)
// 		for(x = 0; x < image.width; x++)
// 			d[]
	for(unsigned i=0; i<3 * image.width * image.height; i++)
		d[i] = image.data[i];

  Z::Image *zImage = new Z::Image(d, (unsigned) image.width, (unsigned) image.height, (unsigned) 3, Z::String2ColorFormat("RGB24"), true, 1920);
//     cvCreateImage(cvSize(image.width,image.height), IPL_DEPTH_8U, 3);


  assert(zImage != 0);
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
//    for(y = 0; y < image.height; y++)
//      for(x = 0; x < image.width; x++)
//       for(c = 0; c < 3; c++)				/// bytes_per_line  === widthStep ???
//          zImage->data[y*image.width + 3*x + c] = image.data[3*(y*image.width + x) + c];
  return zImage;
}

}

