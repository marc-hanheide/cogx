/**
 * $Id$
 */

#ifndef P_CMODEL_HH
#define P_CMODEL_HH

#include <string>
#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "SphereHistogram.hh"
#include "View.hh"
#include <v4r/PGeometry/Pose.hh>


namespace P
{

class CModel
{
public:
  string id;

  vector<cv::Ptr<View> > views;     // learned views
  cv::Ptr<SphereHistogram> viewHist;
  cv::Point3d center;               // object center
  
  Pose pose;                        // current pose

  CModel(int subdivHist=-1);
  ~CModel();

  void clear();
  void copyTo(cv::Ptr<CModel> &dst);
  void save(ofstream &os);
  void load(ifstream &is);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

