/**
 * $Id$
 */

#ifndef P_VIEW_HH
#define P_VIEW_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "v4r/PMath/PMatrix.hh"
#include "v4r/PGeometry/Pose.hh"


namespace P
{

class PKeypoint;

class View
{
public:
  unsigned id;
  unsigned idx;
  int time;

  Pose pose;                // pose of that view
  cv::Point3d vr;           // view ray
  cv::Point2d center;       // object center in that image

  std::vector< cv::Ptr<PKeypoint> > keys;
  cv::Mat_<float> descriptors;

  std::vector< cv::Vec4f > pointcloud;  // dense segmented kinect point cloud (object coordinates)

  View();
  ~View();
  
  void copyTo(cv::Ptr<View> &view);
  void save(std::ofstream &os);
  void load(std::ifstream &is);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

