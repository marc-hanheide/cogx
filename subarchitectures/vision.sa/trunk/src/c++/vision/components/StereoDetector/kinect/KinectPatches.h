/**
 * @file KinectPatches.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Kalculate 3D patches from Kinect data.
 */

#ifndef Z_KINECT_PATCHES_H
#define Z_KINECT_PATCHES_H

#include "KinectBase.h"
#include "VisionCore.hh"

namespace Z
{
  
/**
 * @brief Class KinectPatches: Calculate 3D patches from kinect data.
 */
class KinectPatches : public Z::KinectBase
{
private:
  int numPatches;              ///< Number of extracted 3D plane patches
  
public:
  KinectPatches(KinectCore *kc, VisionCore *vc);
  ~KinectPatches() {}

  int Num3DGestalts() {return numPatches;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
};

}

#endif
