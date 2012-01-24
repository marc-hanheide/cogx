/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _VISION_OBJECTS_H_0DAF_
#define _VISION_OBJECTS_H_0DAF_

#include <VisionData.hpp>
#include <Pose3.h>
#include <cast/architecture/ManagedComponent.hpp>

namespace cogx {

inline
cast::cdl::WorkingMemoryPointerPtr nullWmPointer()
{
  return new cast::cdl::WorkingMemoryPointer(cast::cdl::WorkingMemoryAddress(), "");
}

template<class T>
cast::cdl::WorkingMemoryPointerPtr createWmPointer(const cast::cdl::WorkingMemoryAddress& addr)
{
  if (addr.id.empty())
    return nullWmPointer();
  return new cast::cdl::WorkingMemoryPointer(addr, cast::typeName<T>());
}

inline
void initViewCone(VisionData::ViewCone& vc)
{
  memset(&vc.anchor, 0, sizeof(vc.anchor));
  vc.x = 0;
  vc.y = 0;
  vc.viewDirection = 0;
  vc.tilt = 0;
}

inline
VisionData::ViewConePtr createViewCone()
{
  VisionData::ViewConePtr pVc = new VisionData::ViewCone();
  initViewCone(*pVc);
  return pVc;
}

inline
void initProtoObject(VisionData::ProtoObject& po)
{
  memset(&po.position, 0, sizeof(po.position));
  po.image.width = 0;
  po.image.height = 0;
  po.mask.width = 0;
  po.mask.height = 0;
  memset(&po.sourceImageSize, 0, sizeof(po.sourceImageSize));
  memset(&po.imageOrigin, 0, sizeof(po.imageOrigin));
}

inline
VisionData::ProtoObjectPtr createProtoObject()
{
  VisionData::ProtoObjectPtr pobj = new VisionData::ProtoObject();
  initProtoObject(*pobj);
  return pobj;
}

inline
void initVisualObject(VisionData::VisualObject& vo)
{
  cogx::Math::setIdentity(vo.pose);
  vo.detectionConfidence = 0;
  memset(&vo.boundingSphere, 0, sizeof(vo.boundingSphere));
  vo.presence = VisionData::VopUNKNOWN;
  // Geometry Model - has only sequences
  vo.salience = 0;
  vo.identGain = 0;
  vo.identAmbiguity = 0;
  vo.colorGain = 0;
  vo.colorAmbiguity = 0;
  vo.shapeGain = 0;
  vo.shapeAmbiguity = 0;
}

inline
VisionData::VisualObjectPtr createVisualObject()
{
  VisionData::VisualObjectPtr pobj = new VisionData::VisualObject();
  initVisualObject(*pobj);
  return pobj;
}

};

#endif
// vim: set sw=2 ts=8 sts=4 et :vim
