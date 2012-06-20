/**
 * @file KinectClosures.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Calculate 3D closures from Kinect data.
 */

#ifndef Z_KINECT_CLOSURES_H
#define Z_KINECT_CLOSURES_H

#include "KinectBase.h"
#include "VisionCore.hh"

namespace Z
{
  
/**
 * @brief Class KinectPatches: Calculate 3D patches from kinect data.
 */
class KinectClosures : public Z::KinectBase
{
private:
  int numClosures;              ///< Number of extracted 3D plane patches
  
public:
  KinectClosures(KinectCore *kc, VisionCore *vc);
  ~KinectClosures() {}

  int Num3DGestalts() {return numClosures;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
};

}

#endif
