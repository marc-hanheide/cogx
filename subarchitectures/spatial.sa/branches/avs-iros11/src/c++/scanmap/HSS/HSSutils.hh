//
// = FILENAME
//    HSSutils.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSutils_hh
#define HSSutils_hh

#include <Eigen/Core>
#include <vector>
#include <sys/time.h>
#include <cstdio>

template <class T> const T& HSSmin ( const T& a, const T& b ){return (a<b)?a:b;}
template <class T> const T& HSSmax ( const T& a, const T& b ){return (a<b)?b:a;}

namespace HSS {

inline double sqr(double v) { return v*v; }
inline double deg2rad(double a) { return M_PI/180.0*a; }
inline double rad2deg(double a) { return 180.0/M_PI*a; }

inline bool readCureFileOdom(std::istream &is,
                             Eigen::Vector3d &odom,
                             struct timeval &tv)
{
  std::string line;
  getline(is, line);
  std::istringstream str(line);
  double junk;
  return (str >> junk >> junk >> junk
          >> tv.tv_sec >> tv.tv_usec
          >> junk >> junk >> junk
          >> odom[0] >> odom[1] >> junk >> odom[2]);
}

inline std::string toString(const struct timeval &tv)
{
  char buf[128];
  sprintf(buf, "%ld.%06ld", tv.tv_sec, tv.tv_usec);
  return std::string(buf);
}

inline double pi_to_pi(double a)
{
  int i=0,j=0;

  while (a > M_PI && i++<10) a -= 2.0*M_PI;
  while (a <= -M_PI && j++<10) a += 2.0*M_PI;

  if (i >= 10 || j >= 10) {
    std::cerr << "pi_to_pi got very large/small value, had to stop at "
              << a << std::endl;
  }

  return a;
}

inline double zero_to_2pi(double a)
{
  int i=0,j=0;

  while (a >= 2.0*M_PI && i++<10) a -= 2.0*M_PI;
  while (a < 0 && j++<10) a += 2.0*M_PI;

  if (i >= 10 || j >= 10) {
    std::cerr << "pi_to_pi got very large/small value, had to stop at "
              << a << std::endl;
  }

  return a;
}

inline double getCurrentTime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (1.0*tv.tv_sec + 1e-6*tv.tv_usec);
}

/**
 * Computes v1 + v2 where + should be interpreted as the compund
 * operator which is similar to vector addition. v1+v2 will result in
 * a transformation expressed in the same frame as v1 but v2 added to it. 
 *
 * Example: 
 *   Let xrW be the pose of the robot in the world frame
 *   Let xsR be the pose of the sensor in the robot frame
 *   xsW = xrW + xsR will then be the sensor in the world frame
 *
 * You can provide an optional pointer to a matrix which will return
 * the Jacobian (3x6) of this operation with respect to v1 and v2
 * respectively. The left 3x3 part is w.r.t. v1 and the right
 * w.r.t. v2.
 */
inline Eigen::Vector3d compound(const Eigen::VectorXd &v1,
                                const Eigen::Vector3d &v2,
                                Eigen::MatrixXd *J = 0)
{
  Eigen::Vector3d result;

  if (v1.size() < 3) {
    std::cerr << "compound(" << v1.size() << "," << v2.size()
              << ") both args need to be size >=3 for compound\n";
    throw new std::exception();
  }

  result[0] = v1[0] + cos(v1[2])*v2[0] - sin(v1[2])*v2[1];
  result[1] = v1[1] + sin(v1[2])*v2[0] + cos(v1[2])*v2[1];
  result[2] = pi_to_pi(v1[2] + v2[2]);

  if (J) {
    J->resize(3,6);
    (*J)(0,0) = 1;
    (*J)(0,1) = 0;
    (*J)(0,2) = -sin(v1[2])*v2[0]-cos(v1[2])*v2[1];
    (*J)(0,3) = cos(v1[2]);
    (*J)(0,4) = -sin(v1[2]);
    (*J)(0,5) = 0;
    (*J)(1,0) = 0;
    (*J)(1,1) = 1;
    (*J)(1,2) = cos(v1[2])*v2[0]-sin(v1[2])*v2[1];
    (*J)(1,3) = sin(v1[2]);
    (*J)(1,4) = cos(v1[2]);
    (*J)(1,5) = 0;
    (*J)(2,0) = 0;
    (*J)(2,1) = 0;
    (*J)(2,2) = 1;
    (*J)(2,3) = 0;
    (*J)(2,4) = 0;
    (*J)(2,5) = 1;
  }

  return result;
}


/**
 * Computes v1 + v2 where + should be interpreted as the compund
 * operator which is similar to vector addition. v1+v2 will result in
 * a transformation expressed in the same frame as v1 but v2 added to it. 
 *
 * Example: 
 *   Let xrW be the pose of the robot in the world frame
 *   Let xsR be the pose of the sensor in the robot frame
 *   xsW = xrW + xsR will then be the sensor in the world frame
 *
 * You can provide an optional pointer to a matrix which will return
 * the Jacobian (3x6) of this operation with respect to v1 and v2
 * respectively. The left 3x3 part is w.r.t. v1 and the right
 * w.r.t. v2.
 */
/*
inline Eigen::Vector3d compound(const Eigen::Vector3d &v1,
                                const Eigen::Vector3d &v2,
                                Eigen::MatrixXd *J = 0)
{
  Eigen::Vector3d result;
  
  result[0] = v1[0] + cos(v1[2])*v2[0] - sin(v1[2])*v2[1];
  result[1] = v1[1] + sin(v1[2])*v2[0] + cos(v1[2])*v2[1];
  result[2] = pi_to_pi(v1[2] + v2[2]);
  
  if (J) {
    J->resize(3,6);
    (*J)(0,0) = 1;
    (*J)(0,1) = 0;
    (*J)(0,2) = -sin(v1[2])*v2[0]-cos(v1[2])*v2[1];
    (*J)(0,3) = cos(v1[2]);
    (*J)(0,4) = -sin(v1[2]);
    (*J)(0,5) = 0;
    (*J)(1,0) = 0;
    (*J)(1,1) = 1;
    (*J)(1,2) = cos(v1[2])*v2[0]-sin(v1[2])*v2[1];
    (*J)(1,3) = sin(v1[2]);
    (*J)(1,4) = cos(v1[2]);
    (*J)(1,5) = 0;
    (*J)(2,0) = 0;
    (*J)(2,1) = 0;
    (*J)(2,2) = 1;
    (*J)(2,3) = 0;
    (*J)(2,4) = 0;
    (*J)(2,5) = 1;
  }

  return result;
}
*/

inline Eigen::Vector3d icompound(const Eigen::Vector3d &v,
                                   Eigen::Matrix3d *J = 0)
{
  Eigen::Vector3d result;
  
  result[0] =-cos(v[2])*v[0] - sin(v[2])*v[1];
  result[1] = sin(v[2])*v[0] - cos(v[2])*v[1];
  result[2] = -v[2];
  
  if (J) {
    (*J)(0,0) = -cos(v[2]);
    (*J)(0,1) = -sin(v[2]);
    (*J)(0,2) = sin(v[2])*v[0]-cos(v[2])*v[1];
    (*J)(1,0) = sin(v[2]);
    (*J)(1,1) = -cos(v[2]);
    (*J)(1,2) = cos(v[2])*v[0]+sin(v[2])*v[1];
    (*J)(2,0) = 0;
    (*J)(2,1) = 0;
    (*J)(2,2) = -1;
  }
  
  return result;
}
  
/**
 * Returns the distance from (x,y) to the line given by angle phi
 * through point (xL, yL). The distance is returned with sign where
 * positive means that the point is to the right of the line given that
 * you are facing in the direction of the line.
 *
 * Algorithm: scalar product with direction cosine pointing normal to
 * the right of the line, that in direction phi - 90 degs. Note that
 * sin(x-90) = -cos(x) and cos(x-90) = sin(x).
 * 
 * @param x x coord of point [mm]
 * @param y y coord of point [mm]
 * @param xL x coord of point on the line [mm]
 * @param yL y coord of point on the line [mm]
 * @param phi direction of the line [rad] 
 */
 inline double distPt2Line(double x, double y, 
                           double xL, double yL, double phi)
{ 
  return std::sin(phi) * (x - xL) - std::cos(phi) * (y - yL); 
}


 /**
  * Returns the distance from (x,y) to the line given by angle phi
  * through point (xL, yL). The special thing about this function is
  * that instead of sending in the angle of the line you send in the
  * direction cosines. The distance is returned with sign where
  * positive means that the point is to the right of the line given that
  * you are facing in the direction of the line.
  *
  * Algorithm: scalar product with direction cosine pointing normal to
  * the right of the line, that in direction phi - 90 degs. Note that
  * sin(x-90) = -cos(x) and cos(x-90) = sin(x).
  * 
  * @param x x coord of point [mm]
  * @param y y coord of point [mm]
  * @param xL x coord of point on the line [mm]
  * @param yL y coord of point on the line [mm]
  * @param cosL cos(phi) where phi is direction of the line.
  * @param sinL sin(phi) where phi is direction of the line.
  */
inline double distPt2Line(double x, double y, 
                             double xL, double yL, 
                             double cosL, double sinL)
{ 
  return sinL * (x - xL) - cosL * (y - yL); 
}

/**
 * Returns the distance from P(x,y) to the line given by two points:
 * A(x,y) and B(x,y). The distance is signed, where positive means that
 * the point is to the right of the line facing from A to B.
 *
 * @param xA x coord of point A
 * @param yA y coord of point A
 * @param xB x coord of point B
 * @param yB y coord of point B
 * @param xP x coord of point C
 * @param yP y coord of point C
 */
inline double distPt2LinePts(double xA, double yA,
                                double xB, double yB,
                                double xP, double yP)
{ 
  return ((xP-xA)*(yB-yA)-(yP-yA)*(xB-xA))/hypot(yB-yA,xB-xA); 
}

/**
 * @return the squared distance between between point (x1,y1) and
 * (x2,y2)
 */
inline double sqrDistPt2Pt(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return (dx * dx + dy * dy);
}

/**
 * Check for intersection between two lines A and B described by
 * two points which the line passes through. 
 * 
 *   lineA: (lAx1,lAy1) -> (lAx2,lAy2)
 *   lineB: (lBx1,lBy1) -> (lBx2,lBy2)
 * 
 * tA and tB (if non-NULL) will be set if an intersection is
 * found. tA=0 corresponds to the first point of line A and tA=1 to
 * its second points. The same applies to tB.
 *
 * The intersection point is thus
 *
 * x = lAx1 + tA * (lAx2 - lAx1)
 * y = lAy1 + tA * (lAy2 - lAy1)
 * 
 * @return true if the lines intersect, else false
 */
inline bool linesIntersect(double lAx1, double lAy1, 
                              double lAx2, double lAy2,
                              double lBx1, double lBy1, 
                              double lBx2, double lBy2,
                              double *tA = NULL, double *tB = NULL)
{
  // Base on algorithm found on
  // http://geometryalgorithms.com/Archive/algorithm_0104/algorithm_0104B.htm

  double ux = lAx2 - lAx1;
  double uy = lAy2 - lAy1;
  double vx = lBx2 - lBx1;
  double vy = lBy2 - lBy1;

  double D = ux * vy - uy * vx;

  // First we check if the lines are close to parallell
  if (fabs(D) < 1e-8) {
    return false;
  }

  if (tA != NULL || tB != NULL) {
    double wx = lAx1 - lBx1;
    double wy = lAy1 - lBy1;
    
    if (tA != NULL) 
      *tA = (vx * wy - vy * wx) / D;
    if (tB != NULL) 
      *tB = (ux * wy - uy * wx) / D;
  }

  return true;
}


/**
 * Check for intersection between two line segments A and B
 * described by their start and end points
 * 
 *   lineA: (lAxS,lAyS) -> (lAxE,lAyE)
 *   lineB: (lBxS,lByS) -> (lBxE,lByE)
 * 
 * tA and tB will be set (if non-NULL) if an intersection is
 * found. tA=0 corresponds to the start point of line A and tA=1 to
 * its end points. The same applies to tB. An intersection requires
 * that 0<=tA<=1 and 0<=tB<=1.
 *
 * The intersection point is given by
 *
 * x = lAxS + tA * (lAxE - lAxS)
 * y = lAyS + tA * (lAyE - lAyS)
 *
 * @return true if the segments intersect, else false
 */
inline bool segmentsIntersect(double lAxS, double lAyS, 
                                 double lAxE, double lAyE,
                                 double lBxS, double lByS, 
                                 double lBxE, double lByE,
                                 double *tA = NULL, double *tB = NULL)
{
  double tAA, tBB;
  if (linesIntersect(lAxS, lAyS, lAxE, lAyE,
                     lBxS, lByS, lBxE, lByE, &tAA, &tBB)) {
    if (0 <= tAA && tAA <= 1 && 0 <= tBB && tBB <= 1) {
      if (tA != NULL) *tA = tAA;
      if (tB != NULL) *tB = tBB;
      return true;
    }
  }
                     
  return false;
}

inline void rgb2hsv( float r, float g, float b, float *h, float *s, float *v )
{
  // Copied from http://www.cs.rit.edu/~ncs/color/t_convert.html
  
  float minRGB, maxRGB, delta;

  minRGB = HSSmin(r,HSSmin(g,b));
  maxRGB = HSSmax(r,HSSmax(g,b));

  *v = maxRGB;				// v
  
  delta = maxRGB - minRGB;
  
  if( maxRGB != 0 )
    *s = delta / maxRGB;		// s
  else {
    // r = g = b = 0		// s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }
  
  if (delta < 1e-6) {
    *h = 0;
    *s = 0;
  } else if( r == maxRGB )
    *h = ( g - b ) / delta;		// between yellow & magenta
  else if( g == maxRGB )
    *h = 2 + ( b - r ) / delta;	// between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta;	// between magenta & cyan
  
  *h *= 60;				// degrees
  if( *h < 0 )
    *h += 360;
}

inline void hsv2rgb(float h, float s, float v, float *r, float *g, float *b )
{
  // Copied from http://www.cs.rit.edu/~ncs/color/t_convert.html

  int i;
  float f, p, q, t;
  
  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }

  h /= 60;			// sector 0 to 5
  i = floor( h );
  f = h - i;			// factorial part of h
  p = v * ( 1 - s );
  q = v * ( 1 - s * f );
  t = v * ( 1 - s * ( 1 - f ) );
  
  switch( i ) {
  case 0:
    *r = v;
    *g = t;
    *b = p;
    break;
  case 1:
    *r = q;
    *g = v;
    *b = p;
    break;
  case 2:
    *r = p;
    *g = v;
    *b = t;
    break;
  case 3:
    *r = p;
    *g = q;
    *b = v;
    break;
  case 4:
    *r = t;
    *g = p;
    *b = v;
    break;
  default:		// case 5:
    *r = v;
    *g = p;
    *b = q;
    break;
  }
}

}; // namespace HSS

inline bool operator< (const struct timeval &tv1, const struct timeval &tv2)
{
  return ( (tv1.tv_sec < tv2.tv_sec) ||
           ( (tv1.tv_sec == tv2.tv_sec)  &&
             (tv1.tv_usec < tv2.tv_usec) ) );
}

#endif // HSSutils_hh
