/**
 * @file KinectPclCylinders.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D cylinders from Kinect data with pcl-library.
 */

#ifndef Z_KINECT_PCL_CYLINDERS_H
#define Z_KINECT_PCL_CYLINDERS_H

#include "KinectBase.h"
#include "VisionCore.hh"


namespace Z
{

/**
 * @brief Class KinectPclModels: Calculate 3D cylinders from kinect data with pcl.
 */
class KinectPclCylinders : public KinectBase
{
private:
  int numModels;
  int numCylinders;              ///< Number of extracted 3D cylinders
  

public:
  KinectPclCylinders(KinectCore *kc, VisionCore *vc);
  ~KinectPclCylinders() {}

  int Num3DGestalts() {return numCylinders;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
  
};

}

#endif
