/**
 * @author Michael Zillich
 * @date May 2009
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <VideoUtils.h>
#include <VisionData.hpp>

/**
 * Project a SOI in world co-ordinates to an image ROI.
 */
inline VisionData::ROIPtr projectSOI(const Video::CameraParameters &cam, const VisionData::SOI &soi)
{
   VisionData::ROIPtr roi = new VisionData::ROI;
   roi->rect.pos = projectPoint(cam, soi.boundingSphere.pos);
   roi->rect.width = projectSize(cam, soi.boundingSphere.pos, soi.boundingSphere.rad);
   roi->rect.height = projectSize(cam, soi.boundingSphere.pos, soi.boundingSphere.rad);
   roi->time = soi.time;
   return roi;
}

#endif

