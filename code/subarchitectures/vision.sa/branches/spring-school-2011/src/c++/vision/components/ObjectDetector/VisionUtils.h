/**
 * @file VisionUtils.h
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Utils for the vision system
 **/

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

// #include <Image.h>
#include <Video.hpp>
#include <Image.hh>


namespace cast
{

// using namespace Z;
// using namespace std;
// using namespace Video;

/**
 * Convert from framework imge type to vs3 image class.
 * note: returned image must be cvRelease'd
 */
Z::Image* convertImageToZImage(const Video::Image & image);
}

#endif