/**
 * $Id$
 */

#ifndef P_KEYPOINT_HH
#define P_KEYPOINT_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PMath/PMatrix.hh"
#include "View.hh"


namespace P
{

class View;

class PKeypoint
{
public:
  unsigned id;
  unsigned nb, nb2;
  static unsigned nbcnt;
  static unsigned nb2cnt;
  static double badPoint;

  cv::Point2d pt;
  double size;
  double angle;
  double response;

  cv::Ptr<View> view;
  float *desc;
  unsigned sizeDesc;

  cv::Point3d pos;
  set<PKeypoint*> links;         // links for bundle adjustment or tracking or ...
  PKeypoint *bw, *fw;            // links for tracking...

  PKeypoint();
  PKeypoint(const cv::KeyPoint &k);
  PKeypoint(const PKeypoint &k);
  PKeypoint(cv::Point2d _pt, double _size, double _angle=-1, double _response=0, unsigned _id=-1);
  ~PKeypoint();


  inline unsigned DescriptorSize(){ return sizeDesc; }
  inline float* Descriptor(){ return desc; }
  inline bool HaveView() const { return (!view.empty());}
  inline bool Have3D() const { return (pos.x==pos.x); }
  inline void Release3D(){ pos.x = badPoint; }
  inline void Set3D(const double d[3]);
  inline void Set3D(const float d[3]);
  inline void ReleaseLinks();
  inline void InsertLink(PKeypoint &key);
  inline void InsertBW(PKeypoint &key);
  inline void InsertFW(PKeypoint &key);
  inline void ReleaseFW();
  inline void ReleaseBW();

  static void Vote(const PKeypoint &model, const PKeypoint &key, const cv::Point2d &center, cv::Point2d &vote, double &dAngle, double &dScale);
  static void Draw(cv::Mat &img, const PKeypoint &key, const cv::Scalar& col);
  static void ConvertToCv(const vector<cv::Ptr<PKeypoint> > &keys, vector<cv::KeyPoint> &cvKeys); 
  static void ConvertFromCv(const vector<cv::KeyPoint> &cvKeys, vector<cv::Ptr<PKeypoint> > &keys);
  static void Copy(const vector<cv::Ptr<PKeypoint> > &src, vector<cv::Ptr<PKeypoint> > &dst);
};

ostream& operator<<(ostream &os, const PKeypoint &k);
istream& operator>>(istream &is, PKeypoint &k);




/*************************** INLINE METHODES **************************/

inline void PKeypoint::InsertBW(PKeypoint &key)
{
  ReleaseBW();
  key.ReleaseFW();
  bw = &key;
  key.fw = this;
}

inline void PKeypoint::InsertFW(PKeypoint &key)
{
  ReleaseFW();
  key.ReleaseBW();
  fw = &key;
  key.bw = this;
}

inline void PKeypoint::ReleaseFW()
{
  if (fw!=0 && fw->bw==this)
  { 
    fw->bw=0;
    fw=0;
  }
}

inline void PKeypoint::ReleaseBW()
{
  if (bw!=0 && bw->fw==this)
  {
    bw->fw=0;
    bw=0;
  }
}

inline void PKeypoint::InsertLink(PKeypoint &key)
{
  links.insert(&key);
  key.links.insert(this);
}

inline void PKeypoint::ReleaseLinks()
{
  set<PKeypoint*>::iterator it;

  for (it = links.begin(); it!=links.end(); it++)
    (*it)->links.erase(this);

  links.clear();
}

inline void PKeypoint::Set3D(const double d[3])
{
  pos.x = d[0];
  pos.y = d[1];
  pos.z = d[2];
}

inline void PKeypoint::Set3D(const float d[3])
{
  pos.x = d[0];
  pos.y = d[1];
  pos.z = d[2];
}



} //--END--

#endif

