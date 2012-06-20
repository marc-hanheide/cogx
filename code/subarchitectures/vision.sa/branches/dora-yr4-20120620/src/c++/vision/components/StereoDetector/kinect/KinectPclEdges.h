/**
 * @file KinectPclEdges.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D edge images from Kinect data with Tom's RGBD-Segmenter.
 */

#ifndef Z_KINECT_PCL_EDGES_H
#define Z_KINECT_PCL_EDGES_H

#include "KinectBase.h"
#include "VisionCore.hh"
#include "IdImage.hh"
#include "FormSegments.hh"

#include "v4r/TomGine/tgEngine.h"
#include "v4r/RGBDSegment/RGBDSegment.h"


namespace Z
{

/**
 * @brief Class KinectPclEdges: Calculate 3D edges from kinect data.
 */
class KinectPclEdges : public KinectBase
{
private:
  int numEdgels;                       ///< Number of extracted 3D edgels
  
  TomGine::tgEngine *tgEngine;
  RGBDSegment *rgbdSegment;

public:
  KinectPclEdges(KinectCore *kc, VisionCore *vc);
  ~KinectPclEdges() {}

  int Num3DGestalts() {return numEdgels;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
  
};

}

#endif
