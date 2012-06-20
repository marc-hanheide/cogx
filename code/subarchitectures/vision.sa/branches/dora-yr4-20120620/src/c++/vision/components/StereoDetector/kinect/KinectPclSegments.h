/**
 * @file KinectPclSegments.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D segments from Kinect data.
 */

#ifndef Z_KINECT_PCL_SEGMENTS_H
#define Z_KINECT_PCL_SEGMENTS_H

#include "KinectBase.h"
#include "VisionCore.hh"

#include "Segment.hh"

#include "v4r/TomGine/tgEngine.h"
#include "v4r/RGBDSegment/RGBDSegment.h"

namespace Z
{
  
/**
 * @brief Class KinectPclSegments: Calculate 3D segments from kinect data.
 */
class KinectPclSegments : public Z::KinectBase
{
private:
  int numSegments;              ///< Number of extracted 3D segments
  
  TomGine::tgEngine *tgEngine;
  RGBDSegment *rgbdSegment;
  
  void RightOf(int dirX, int dirY, int &dX, int &dY);
  void LeftOf(int dirX, int dirY, int &dX, int &dY);
  
  cv::Mat_<float> color_edges;
  cv::Mat_<float> depth_edges;
  cv::Mat_<float> curvature_edges;
  cv::Mat_<float> mask_edges;
  cv::Mat_<float> edges;
  
  cv::Vec4f GetSupportVector(int x, int y);

public:
  KinectPclSegments(KinectCore *kc, VisionCore *vc);
  ~KinectPclSegments() {}

  int Num3DGestalts() {return numSegments;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
};

}

#endif
