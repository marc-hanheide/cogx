/**
 * @author Michael Zillich
 * @date May 2009
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <tools/hardware/video/src/c++/utils/VideoUtils.h>
#include "VisionData.hpp"

/**
 * Project a SOI in world co-ordinates to an image ROI.
 */
inline ROI projectSOI(const CameraParameters &cam, const SOI &soi)
{
  ROI roi;
  roi.rect.pos = projectPoint(cam, SOI.boundingSphere.pos);
  roi.rect.width = projectSize(cam, SOI.boundingSphere.rad);
  roi.rect.height = projectSize(cam, SOI.boundingSphere.rad);
  roi.time = soi.time;
  return roi;
}

#endif

