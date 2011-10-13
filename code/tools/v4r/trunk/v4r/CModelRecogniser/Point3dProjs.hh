/**
 * $Id$
 */

#ifndef P_POINT_3D_PROJS_HH
#define P_POINT_3D_PROJS_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include "v4r/PMath/PMath.hh"
#include "v4r/PMath/PVector.hh"


namespace P
{


class Point3dProjs
{
public:
  unsigned idx;
  cv::Point3d pt;               // 3d point location
  std::vector< std::pair<unsigned, unsigned> > projs;     // <idxView, idxKeypoint>

  unsigned nb;
  static unsigned nbcnt;

  Point3dProjs() : nb(0) {};
  Point3dProjs(unsigned _idx, cv::Point3d _pt) : idx(_idx), pt(_pt), nb(0) {};
  ~Point3dProjs() {};
  inline bool Insert(unsigned vidx, unsigned kidx, cv::Point3f &pt);
  inline bool TestInsert(unsigned vidx, unsigned kidx);
};







/*************************** INLINE METHODES **************************/

inline bool Point3dProjs::TestInsert(unsigned vidx, unsigned kidx)
{
  if (projs.back().first!=vidx || projs.back().second!=kidx)
    return true;
  return false;
}


inline bool Point3dProjs::Insert(unsigned vidx, unsigned kidx, cv::Point3f &_pt)
{
  if (projs.back().first!=vidx || projs.back().second!=kidx)
  {
    /*PVec::Mul3(&pt.x, projs.size(), &pt.x);
    PVec::Add3(&pt.x, &_pt.x, &pt.x);
    PVec::Mul3(&pt.x, 1./(projs.size()+1.), &pt.x );*/

    projs.push_back(std::pair<unsigned,unsigned>(vidx,kidx));

    return true;
  }
  return false;
}




} //--END--

#endif

