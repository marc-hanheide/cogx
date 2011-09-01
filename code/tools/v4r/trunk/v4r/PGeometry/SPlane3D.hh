/**
 * $Id$
 */

#ifndef P_SPLANE_3D_HH
#define P_SPLANE_3D_HH

#include <vector>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMath.hh"

namespace P
{

using namespace std;

class SPlane3D
{
private:
  std::vector<cv::Vec4d> points4;
  void FitPlaneLS(std::vector<cv::Vec4d> &p3d, double n[3], double p[3]);
  void ProjectPointsToPlane3D(vector<cv::Vec4d> &p3d, double n[3], double *points);
  bool FitPlaneRANSAC(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2, cv::Vec4d &p4);
  void GetRandIdx(unsigned size, unsigned num, P::vector<unsigned> &idx);
  void GetInlierPlane3D(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2, cv::Vec4d &p3, int &inl);
  bool RefinePlane3dLS(vector<cv::Vec4d> &p3d, cv::Vec4d &p1, cv::Vec4d &p2,  cv::Vec4d &p3,
                       vector<cv::Point3d> &inPlane, double n[3], double p[3]);

  inline bool Contains(const vector<unsigned> &idx, unsigned num);


public:
  vector<int> outl;

  SPlane3D();
  SPlane3D(unsigned size);
  ~SPlane3D();
  
  void GetMean(double mean[3]);
  void GetMedian(double median[3]);

  void FitPlaneLS(double n[3], double p[3]);
  bool FitPlaneRobustLS(double n[3], double p[3]);
  void ProjectPointsToPlane3D(double n[3], double *points);
  bool ProjectPointsToPlane3DRobust(double n[3], double *points, unsigned &num);


  inline void Insert(double p[3]);
  inline void clear();
  inline unsigned size();
};


inline void Plane3dExp2Imp(double p1[3], double p2[3], double p3[3], double &a, double &b, double &c, double &d);
inline void Plane3dImpPointDist(double a, double b, double c, double d, double p[3], double &dist);
inline void Plane3dNormalPointDist(double p[3], double n[3], double pd[3], double &dist);
inline bool Plane3dLineIntersection(double p[3], double n[3], double l[3], double isct[3]);
inline void Plane3dExpNormal(double p1[3], double p2[3], double p3[3], double n[3]);




/************************** INLINE METHODES ******************************/
inline bool SPlane3D::Contains(const vector<unsigned> &idx, unsigned num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}

/**
 * Insert a 3d point to the data structure
 */
inline void SPlane3D::Insert(double p[3])
{
  points4.push_back(cv::Vec4d(p[0],p[1],p[2],1.));
}

/**
 * clear the data structure
 */
inline void SPlane3D::clear()
{
  points4.clear();
}

/**
 * Get data size
 */
inline unsigned SPlane3D::size()
{
  return points4.size();
}


/**
 * Convert explicit plane to implicit
 */
inline void Plane3dExp2Imp(double p1[3], double p2[3], double p3[3], double &a, double &b, double &c, double &d)
{
  a = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
     - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  b = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
     - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  c = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
     - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  d = - p2[0] * a - p2[1] * b - p2[2] * c;
}

inline void Plane3dImpPointDist(double a, double b, double c, double d, double p[3], double &dist)
{
  dist = fabs ( a * p[0] + b * p[1] + c * p[2] + d ) /
      sqrt ( a * a + b * b + c * c );
}

inline void Plane3dNormalPointDist(double p[3], double n[3], double pd[3], double &dist)
{
  double t[3];
  
  t[0] = pd[0] - p[0];
  t[1] = pd[1] - p[1];
  t[2] = pd[2] - p[2];

  dist = t[0]*n[0] + t[1]*n[1] + t[2]*n[2];
}

/**
 * compute plane ray intersection
 * the plane normal n and the ray r must be normalised
 */
inline bool Plane3dLineIntersection(double p[3], double n[3], double r[3], double isct[3])
{
  double temp;

  if ( PVec::Dot3(n, r) == 0.0 )
    return false;

  temp = PVec::Dot3(n,p) / PVec::Dot3(n,r);
  PVec::Mul3(r,temp,isct);

  return true;
}

/**
 * Compute the normal to an explicit plane in 3D
 */
inline void Plane3dExpNormal(double p1[3], double p2[3], double p3[3], double n[3])
{
  double norm;

  n[0] = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
       - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  n[1] = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
       - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  n[2] = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
       - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  norm = sqrt ( PMath::Sqr(n[0]) + PMath::Sqr(n[1]) + PMath::Sqr(n[2] ) );

  if ( norm == 0.0 )
  {
    throw runtime_error ("SPlane3D::Plane3dExpNormal : Invalide plane!");
  }
  else
  {
    n[0] /= norm;
    n[1] /= norm;
    n[2] /= norm;
  }

}


}

#endif

