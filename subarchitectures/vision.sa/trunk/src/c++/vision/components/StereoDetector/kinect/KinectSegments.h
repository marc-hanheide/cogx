/**
 * @file KinectSegments.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D segments from Kinect data.
 */

#ifndef Z_KINECT_SEGMENTS_H
#define Z_KINECT_SEGMENTS_H

#include "KinectBase.h"
#include "VisionCore.hh"

#include "Segment.hh"

namespace Z
{
  
/**
 * @brief Class KinectSegments: Calculate 3D segments from kinect data.
 */
class KinectSegments : public Z::KinectBase
{
private:
  int numSegments;              ///< Number of extracted 3D segments
  
public:
  KinectSegments(KinectCore *kc, VisionCore *vc);
  ~KinectSegments() {}

  int Num3DGestalts() {return numSegments;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
};

}

#endif
