/**
 * $Id$
 */

#ifndef P_MERGE_PLANES_HF_HH
#define P_MERGE_PLANES_HF_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Plane.hh"
#include "PKeypoint.hh"
#include "v4r/PGeometry/SLine2D.hh"
#include "v4r/PGeometry/PHomography.hh"



namespace P
{

class MergePlanesHF
{
private:
  class Parameter
  {
  public:
    double sigmaDistF;
    Parameter(double sigma=2.)
    : sigmaDistF(sigma) {}
  };

  double INV_SQR_SIGMA2;

  bool GetPoints(const Plane &plane, vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2);
  void ComputeErrorF(vector<cv::Point2f> &pts, vector<cv::Vec3f> &lines, double &error);
  void ComputeErrorH(const Plane &plane1, const Plane &plane2, double &error);


public:
  cv::Mat dbg;
  Parameter param;

  MergePlanesHF(Parameter p=Parameter());
  ~MergePlanesHF();

  void Operate(const vector< cv::Ptr<Plane> > &planes);
  void DrawLine(cv::Mat &img, cv::Vec3f &l);
  
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

