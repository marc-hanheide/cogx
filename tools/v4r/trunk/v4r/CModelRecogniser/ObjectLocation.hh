/**
 * $Id$
 */

#ifndef P_OBJECT_LOCATION_HH
#define P_OBJECT_LOCATION_HH

#include <opencv2/core/core.hpp>
#include "v4r/PGeometry/Pose.hh"

namespace P
{

class ObjectLocation
{
public:
  string idObject;
  unsigned idView;

  cv::Mat H;             // location affine homography
  Pose pose;             // location pose
  double conf;

  unsigned idCluster;

  ObjectLocation() : idView(UINT_MAX), conf(0.), idCluster(UINT_MAX){};
  ObjectLocation(const string &oid, unsigned vid) : idObject(oid), idView(vid), conf(0), idCluster(UINT_MAX) {};
  ObjectLocation(const ObjectLocation &obj) 
    : idObject(obj.idObject), 
      idView(obj.idView), 
      pose(obj.pose),
      conf(obj.conf),
      idCluster(obj.idCluster) 
    {
      obj.H.copyTo(H);
    };
  ~ObjectLocation(){};

  ObjectLocation& operator=(const ObjectLocation &obj)
  {
    this->idObject = obj.idObject;
    this->idView = obj.idView;
    this->pose = obj.pose;
    this->conf = obj.conf;
    this->idCluster = obj.idCluster;
    obj.H.copyTo(this->H);
    return *this;
  }

  inline bool HavePose() {return !pose.empty();}
  inline bool HaveH() {return !H.empty();}
};


/*********************** INLINE METHODES **************************/


}

#endif

