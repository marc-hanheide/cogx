/**
 * $Id$
 */

#ifndef P_SCIRCLE2D_HH
#define P_SCIRCLE2D_HH

#include <opencv2/core/core.hpp>
#include <vector>
#include <fitcircle.h>
#include "v4r/PGeometry/SLine2D.hh"
#include "v4r/PMath/PMath.hh"
#include "v4r/PMath/PVector.hh"

namespace P
{

class SCircle2D
{
public:
  SCircle2D(){}
  ~SCircle2D(){}
  
  static void FitCircle(const vector<cv::Point2d> &ps, cv::Point2d &c, double &r);
  static cv::Point2d CircleCenter(const cv::Point2d &pi, const cv::Point2d &pj, const cv::Point2d &pk);
};


/************************ INLINE METHODES ***************************/

/**
 * FitCircle
 */
void SCircle2D::FitCircle(const vector<cv::Point2d> &ps, cv::Point2d &c, double &r)
{
  vector<double> x(ps.size()), y(ps.size());

  for (unsigned i=0; i<ps.size(); i++)
  {
    x[i] = ps[i].x;
    y[i] = ps[i].y;
  }

  fit_circle(&x[0], &y[0], ps.size(), &c.x, &c.y, &r);
}

/**
 * CircleCenter
 */
inline cv::Point2d SCircle2D::CircleCenter(const cv::Point2d &pi, const cv::Point2d &pj, const cv::Point2d &pk)
{
  // throws an exception if intersection cannot be calculated
  return SLine2D::LineIntersection(
      cv::Point2d((pi.x + pj.x)/2., (pi.y + pj.y)/2.),
      cv::Point2d(pj.y - pi.y, pi.x - pj.x),
      cv::Point2d((pj.x + pk.x)/2., (pj.y + pk.y)/2.),
      cv::Point2d(pk.y - pj.y, pj.x - pk.x));
}




}


#endif

