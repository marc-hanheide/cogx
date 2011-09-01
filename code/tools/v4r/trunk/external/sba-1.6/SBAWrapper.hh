/**
 * $Id$
 */

#ifndef P_BUNDLER_HH
#define P_BUNDLER_HH

#include <set>
#include <map>
#include <v4r/PGeometry/Pose.hh>
#include <v4r/PMath/PVector.hh>
#include <v4r/PMath/PMatrix.hh>
#include <v4r/PMath/PMath.hh>
#include "sba.h"

namespace P
{

class SBAData
{
public:
  cv::Point3d pos;
  vector<pair<unsigned,cv::Point2d> > projs;    //<camera_index,projection>

  SBAData() {};
  SBAData(const cv::Point3d &_pos) : pos(_pos) {};
  SBAData(const cv::Point3f &_pos) : pos(cv::Point3d(_pos.x,_pos.y,_pos.z)) {};


  inline void InsertProjection(const pair<unsigned,cv::Point2d> &pt)
  {
    for (unsigned i=0; i<projs.size(); i++)
      if (projs[i].first == pt.first)
        return;
    projs.push_back(pt);
  };
  inline void InsertProjection(const pair<unsigned,cv::Point2f> &pt)
  {
    for (unsigned i=0; i<projs.size(); i++)
      if (projs[i].first == pt.first)
        return;
    projs.push_back(pair<unsigned,cv::Point2d>(pt.first,cv::Point2d(pt.second.x,pt.second.y)));
  };
}; 



class SBAWrapper
{
private:
  unsigned sbaMaxIter;
  cv::Mat cam;

  vector<cv::Ptr<Pose> > cameras;
  vector<cv::Ptr<SBAData> > points;

  void SortData(vector<cv::Ptr<SBAData> > &data);
  bool Convert2SBA(vector<cv::Ptr<Pose> > &cams, vector<cv::Ptr<SBAData> > &data, int &n2Dprojs, int cnp, int pnp, int mnp, double **motstruct, double **initrot, double **imgpts, char **vmask);
  void Convert2Obj(vector<cv::Ptr<Pose> > &cams, vector<cv::Ptr<SBAData> > &data, int n2Dprojs, double **motstruct, double **initrot, double **imgpts, char **vmask);

  void EvaluateData(vector<cv::Ptr<SBAData> > &data);



public:

  SBAWrapper(unsigned _sbaMaxIter=500, cv::Mat intrinsic=cv::Mat());
  ~SBAWrapper();

  void Clear();

  void SetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Ptr<SBAData> > &_points);
  void InsertCamera(const cv::Mat &R, const cv::Mat &t);
  void InsertPoints(const cv::Point3d &pos, const vector<pair<unsigned,cv::Point2d> > &projs);
  void InsertPoints(const cv::Point3f &pos, const vector<pair<unsigned,cv::Point2f> > &projs);

  int BundleMotStruct(int nconstframes=0);
  int BundleMot(int nconstframes=0);
  
  void GetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Ptr<SBAData> > &_points) const;
  void GetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Point3d> &_points) const;

  void SetCameraParameter(const cv::Mat &intrinsic);
};


/*********************** INLINE METHODES **************************/


}

#endif

