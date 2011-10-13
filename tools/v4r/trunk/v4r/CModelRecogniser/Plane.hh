/**
 * $Id$
 */

#ifndef P_PLANE_HH
#define P_PLANE_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"


namespace P
{

class Plane
{
public:
  unsigned id;
  unsigned idx;
  static unsigned idcnt;

  cv::Mat_<double> H;           //plane is represented by a homography

  std::vector< cv::Ptr<PKeypoint> > keys;
  std::vector< cv::Ptr<PKeypoint> > lastKeys;
  
  cv::Mat_<float> descriptors;

  cv::Point2d center;

  // links for tracking and merging of plane
  std::map<cv::Ptr<Plane>, unsigned> fwTrack;   //<plane link forward track, number of matches>
  std::map<cv::Ptr<Plane>, unsigned> bwTrack;   //<plane link backward track, number of matches>
  std::map<cv::Ptr<Plane>, double> motLink;     //<plane links of the current frame, motion similarity>

  bool haveMotion;
  cv::Scalar col;

  Plane(unsigned _id=UINT_MAX);
  ~Plane();
  
  void copyTo(cv::Ptr<Plane> &view);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

