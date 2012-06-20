/**
 * @file KinectRectangles.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Calculate 3D rectangles from Kinect data.
 */

#ifndef Z_KINECT_RECTANGLES_H
#define Z_KINECT_RECTANGLES_H

#include "KinectBase.h"
#include "VisionCore.hh"

namespace Z
{
  
/**
 * @brief Class KinectRectangles: Calculate 3D rectangles from kinect data.
 */
class KinectRectangles : public Z::KinectBase
{
private:
  int numRectangles;              ///< Number of extracted 3D rectangles
  
public:
  KinectRectangles(KinectCore *kc, VisionCore *vc);
  ~KinectRectangles() {}

  int Num3DGestalts() {return numRectangles;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
};

}

#endif
