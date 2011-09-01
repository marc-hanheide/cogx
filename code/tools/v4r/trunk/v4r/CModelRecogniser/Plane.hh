/**
 * $Id$
 */

#ifndef P_PLANE_HH
#define P_PLANE_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"


namespace P
{

class Plane
{
public:
  unsigned id;
  static unsigned idcnt;

  cv::Mat_<double> H;           //plane is represented by a homography

  vector< cv::Ptr<PKeypoint> > keys;
  vector< cv::Ptr<PKeypoint> > lastKeys;
  
  cv::Mat_<float> descriptors;

  bool haveMotion;

  Plane(unsigned _id=UINT_MAX);
  ~Plane();
  
  void copyTo(cv::Ptr<Plane> &view);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

