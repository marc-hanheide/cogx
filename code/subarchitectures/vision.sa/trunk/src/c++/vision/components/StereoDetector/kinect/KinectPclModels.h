/**
 * @file KinectPclModels.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D models from Kinect data with pcl-library.
 */

#ifndef Z_KINECT_PCL_CYLINDERS_H
#define Z_KINECT_PCL_CYLINDERS_H

#include "KinectBase.h"
#include "VisionCore.hh"

#include "v4r/PCLAddOns/ModelFitter.h"


namespace Z
{

/**
 * @brief Class KinectPclModels: Calculate 3D models from kinect data with pcl.
 */
class KinectPclModels : public KinectBase
{
private:
  int numModels;                ///< Number of extracted 3D models
  int numPatches;               ///< Number of extracted 3D patches
  int numSpheres;               ///< Number of extracted 3D spheres
  int numCylinders;             ///< Number of extracted 3D cylinders
  
  pclA::ModelFitter *model_fitter;

public:
  KinectPclModels(KinectCore *kc, VisionCore *vc);
  ~KinectPclModels() {}

  int Num3DGestalts() {return numModels;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
  
};

}

#endif
