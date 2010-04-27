/**
 * @author Michael Zillich
 * @date May 2009
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <VideoUtils.h>
#include <Sphere3.h>
#include <Box3.h>
#include <VisionData.hpp>

/**
 * Project a SOI in world co-ordinates to an image ROI.
 */
inline VisionData::ROIPtr projectSOI(const Video::CameraParameters &cam, const VisionData::SOI &soi)
{
   VisionData::ROIPtr roi = new VisionData::ROI;
   roi->rect.pos = projectPoint(cam, soi.boundingSphere.pos);
   // assuming that the image is a raw, distorted camera image, we now have to
   // to distort the position to match the image
   distortPoint(cam, roi->rect.pos, roi->rect.pos);
   roi->rect.width = projectSize(cam, soi.boundingSphere.pos, 2.*soi.boundingSphere.rad);//std::cout<<"roi->rect.width "<<roi->rect.width<<std::endl;
   roi->rect.height = projectSize(cam, soi.boundingSphere.pos, 2.*soi.boundingSphere.rad);
   roi->time = soi.time;
   return roi;
}


inline bool pointInsideSOI(const VisionData::SOI &soi, const cogx::Math::Vector3 &p)
{
  return pointInsideSphere(soi.boundingSphere, p) &&
         pointInsideBox(soi.boundingBox, p);
}

#endif

