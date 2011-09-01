/**
 * $Id$
 */

#ifndef P_CMODEL_HANDLER_HH
#define P_CMODEL_HANDLER_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <v4r/PGeometry/Pose.hh>
#include "PKeypoint.hh"
#include "CModel.hh"
#include "ConfValues.hh"
#include "SphereHistogram.hh"


namespace P
{

class CModelHandler
{
private:
  void ComputeViewRay(Pose &pose, cv::Point3d &objCenter, cv::Point3d &vr);

public:
  CModelHandler();
  ~CModelHandler();
 
  void RenewProbSphere(const cv::Mat &cam, const cv::Mat &distCoeffs, CModel &model);

  void Save(const string &filename, CModel &model);
  bool Load(const string &filename, CModel &model);
  void Save(const string &filename, cv::Ptr<CModel> &model);
  bool Load(const string &filename, cv::Ptr<CModel> &model);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

