/**
 * $Id$
 */

#ifndef P_MERGE_PLANES_HF_HH
#define P_MERGE_PLANES_HF_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "Plane.hh"
#include "PKeypoint.hh"
#include "GraphFrame.hh"
#include "v4r/PGeometry/SLine2D.hh"
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/PCore/toString.hpp"



namespace P
{

class MergePlanesHF
{
public:
  class Parameter
  {
  public:
    int width, height;
    double sigmaDistF;
    double sigmaDistH;
    Parameter(int w=640, int h=480, double sigmaf=2., double sigmah=2.)
    : width(w), height(h), sigmaDistF(sigmaf), sigmaDistH(sigmah) {}
  };

private:
  double INV_SQR_SIGMA2_F, INV_SQR_SIGMA2_H, CONST_GAUSS_F;

  bool GetPoints(const Plane &plane, std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2);
  void ComputeErrorF(std::vector<cv::Point2f> &pts, std::vector<cv::Vec3f> &lines, double &error);
  void ComputeErrorH(const Plane &plane1, const Plane &plane2, double &error);
  void ComputeProbF(const Plane &plane1, const Plane &plane2, double &prob);




public:
  cv::Mat dbg;
  Parameter param;

  MergePlanesHF(Parameter p=Parameter());
  ~MergePlanesHF();

  void Operate(std::vector< cv::Ptr<Plane> > &planes);

  void DrawLine(cv::Mat &img, cv::Vec3f &l);
  
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

