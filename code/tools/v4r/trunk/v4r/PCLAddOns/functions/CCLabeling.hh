/**
 * $Id$
 * Johann Prankl, 2010-12-01
 * prankl@acin.tuwien.ac.at
 */

#ifndef PCLA_CC_LABELING_HH
#define PCLA_CC_LABELING_HH 

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <stdexcept>
#include <float.h>
#include <iostream>
#include <set>
#include <map>
#include <time.h>

namespace pclA
{

class Label
{
public:
  ushort id;
  Label *parent;
  ushort rank;
  Label(ushort _id) : id(_id), parent(this), rank(0) {}
};


class CCLabeling
{
public:
  class Parameter
  {
  public:
    float thr;                       // min euc. distance to connect points (0.02)
    unsigned minClusterSize;         // min cluster size
    Parameter(float th=.02, unsigned minsize=100) : thr(th), minClusterSize(minsize) {}
  };

private:

  Label* Find(Label *x);
  ushort Union(Label *x, Label* y);

  inline float SqrDistance(const cv::Vec4f &pt1, const cv::Vec4f &pt2);
  inline float SqrDistanceZ(const cv::Vec4f &pt1, const cv::Vec4f &pt2);
  inline float Sqr(const float x);


public:
  Parameter param;

  CCLabeling(Parameter _param=Parameter());
  ~CCLabeling();

  void Operate(const cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, vector<unsigned> &cluster_size);
  void FilterClusterSize(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, vector<unsigned> &cluster_size);
  void FilterLargestCluster(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, vector<unsigned> &cluster_size);
  void CreateMask(const cv::Mat_<ushort> &labels, const vector<unsigned> &cluster_size, cv::Mat_<uchar> &mask);

};




/*********************** INLINE METHODES **************************/

inline float CCLabeling::Sqr(const float x)
{
  return x*x;
}

inline float CCLabeling::SqrDistance(const cv::Vec4f &pt1, const cv::Vec4f &pt2)
{
  if (pt2[0]==pt2[0] && pt2[1]==pt2[1] && pt2[2]==pt2[2])
    return Sqr(pt1[0]-pt2[0])+Sqr(pt1[1]-pt2[1])+Sqr(pt1[2]-pt2[2]);

  return FLT_MAX;
}

inline float CCLabeling::SqrDistanceZ(const cv::Vec4f &pt1, const cv::Vec4f &pt2)
{
  if (pt2[0]==pt2[0] && pt2[1]==pt2[1] && pt2[2]==pt2[2])
    return Sqr(pt1[2]-pt2[2]);

  return FLT_MAX;
}



}

#endif

