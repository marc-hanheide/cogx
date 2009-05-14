/**
 * Utility stuff for Visual subarchitecture.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <math.h>
#include <exception>
#include <iostream>
#include <vector>
#include <balt/core/BALTException.hpp>
#include <vision/idl/Vision.hh>
#include <CoSyCommon/idl/Math.hh>

#include <opencv/cv.h>
#include <opencv/highgui.h>


//---------- General stuff ------------------------------------------

template<class T>
inline T Sqr(T x)
{
  return x*x;
}

namespace Vision
{

using namespace std;
using namespace Math;
using namespace FrameworkBasics;

// HACK: for arm mounted on B21, the groundplane (= table) is 9 cm below
// Katana base
#define GROUND_HEIGHT 0.000


//---------- Images -------------------------------------------------

/**
 * Save image to given file.
 * File ending indicates the file format (".jpg", ".png", ..)
 * Please avoid uncompressed file formats.
 */
void SaveImage(ImageFrame &img, const string &filename);

/**
 * Get sub image defined by bounding box from whole image.
 * The bounding box is clipped to fit the whole image, and may thus be changed
 * on return.
 * Returns true if at least part of the bounding box was inside whole image,
 * false if the bounding box was completely outside.
 */
//bool GetSubImage(const ImageFrame &whole, Image &part, BBox2D &box);

/**
 * Downsamle an image frame by a given factor.
 * Note: image width must be a multiple of factor.
 * Note: no anti-aliasing is performed.
 */
void DownsampleImage(ImageFrame *src, ImageFrame *dst, int f);


//---------- BALTTime -----------------------------------------------

inline bool operator==(const BALTTime &a, const BALTTime &b)
{
  return a.m_s == b.m_s && a.m_us == b.m_us;
}

inline bool operator!=(const BALTTime &a, const BALTTime &b)
{
  return !(a == b);
}

inline bool operator>(const BALTTime &a, const BALTTime &b)
{
  return (a.m_s > b.m_s) || (a.m_s == b.m_s && a.m_us > b.m_us);
}

inline bool operator>=(const BALTTime &a, const BALTTime &b)
{
  return a == b || a > b;
}

inline bool operator<(const BALTTime &a, const BALTTime &b)
{
  return (a.m_s < b.m_s) || (a.m_s == b.m_s && a.m_us < b.m_us);
}

inline bool operator<=(const BALTTime &a, const BALTTime &b)
{
  return a == b || a < b;
}


inline bool IsZero(double x)
{
  return fpclassify(x) == FP_ZERO;
}

inline bool IsZero(float x)
{
  return fpclassify(x) == FP_ZERO;
}


//---------- Vector2D -----------------------------------------------

inline Vector2D operator*(double s, const Vector2D &p)
{
  Vector2D q;
  q.m_x = s*p.m_x;
  q.m_y = s*p.m_y;
  return q;
}

inline Vector2D operator*(const Vector2D &p, double s)
{
  return s*p;
}

inline Vector2D operator/(const Vector2D &p, double s) throw(BALTException)
{
  if(isnormal(s))
    return (1./s)*p;
  else
    throw BALTException(__HERE__, "division by zero");
}

inline Vector2D operator-(const Vector2D &p)
{
  Vector2D q;
  q.m_x = -p.m_x;
  q.m_y = -p.m_y;
  return q;
}

inline Vector2D operator-(const Vector2D &a, const Vector2D &b)
{
  Vector2D c;
  c.m_x = a.m_x - b.m_x;
  c.m_y = a.m_y - b.m_y;
  return c;
}

inline Vector2D operator+(const Vector2D &a, const Vector2D &b)
{
  Vector2D c;
  c.m_x = a.m_x + b.m_x;
  c.m_y = a.m_y + b.m_y;
  return c;
}

inline double Length(const Vector2D &v)
{
  return sqrt(v.m_x*v.m_x + v.m_y*v.m_y);
}

/**
 * Returns the center (mean vector) of the given vectors.
 */
Vector2D Center(const vector<Vector2D> &points);


//---------- Vector3D -----------------------------------------------

istream& operator>>(istream &is, Vector3D &v);
ostream& operator<<(ostream &os, const Vector3D &v);

// Print vector to a string.
// Note: returns pointer to a static string.
inline const char* ToCString(const Vector3D &v)
{
  static char str[256];
  snprintf(str, 256, "(%.3f %.3f %.3f)", v.m_x, v.m_y, v.m_z);
  return str;
}

inline Vector3D operator*(double s, const Vector3D &p)
{
  Vector3D q;
  q.m_x = s*p.m_x;
  q.m_y = s*p.m_y;
  q.m_z = s*p.m_z;
  return q;
}

inline Vector3D operator*(const Vector3D &p, double s)
{
  return s*p;
}

inline Vector3D operator/(const Vector3D &p, double s) throw(BALTException)
{
  if(isnormal(s))
    return (1./s)*p;
  else
    throw BALTException(__HERE__, "division by zero");
}

inline Vector3D operator-(const Vector3D &p)
{
  Vector3D q;
  q.m_x = -p.m_x;
  q.m_y = -p.m_y;
  q.m_z = -p.m_z;
  return q;
}

inline Vector3D operator-(const Vector3D &a, const Vector3D &b)
{
  Vector3D c;
  c.m_x = a.m_x - b.m_x;
  c.m_y = a.m_y - b.m_y;
  c.m_z = a.m_z - b.m_z;
  return c;
}

inline Vector3D operator+(const Vector3D &a, const Vector3D &b)
{
  Vector3D c;
  c.m_x = a.m_x + b.m_x;
  c.m_y = a.m_y + b.m_y;
  c.m_z = a.m_z + b.m_z;
  return c;
}

//
 inline double distance(const Vector3D &v1, const Vector3D &v2)
{
  double dx = v1.m_x - v2.m_x;
  double dy = v1.m_y - v2.m_y;
  double dz = v1.m_z - v2.m_z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}


inline double Length(const Vector3D &v)
{
  return sqrt(v.m_x*v.m_x + v.m_y*v.m_y + v.m_z*v.m_z);
}

inline void SetZero(Vector3D &v) {v.m_x = v.m_y = v.m_z = 0.;}

inline bool IsZero(const Vector3D &v)
{
  return IsZero(v.m_x) && IsZero(v.m_y) && IsZero(v.m_z);
}

Vector3D Normalise(const Vector3D &v);

Vector3D Center(const vector<Vector3D> &points);

inline double Dot(const Vector3D &a, const Vector3D &b)
{
  return a.m_x*b.m_x + a.m_y*b.m_y + a.m_z*b.m_z;
}

inline Vector3D Cross(const Vector3D &a, const Vector3D &b)
{
  Vector3D c = {a.m_y*b.m_z - a.m_z*b.m_y,
    a.m_z*b.m_x - a.m_x*b.m_z,
    a.m_x*b.m_y - a.m_y*b.m_x};
  return c;
}


//---------- Pose3D -------------------------------------------------

istream& operator>>(istream &is, Pose3D &pose);
ostream& operator<<(ostream &os, const Pose3D &pose);

// Print pose to string.
// If matrix is true, prints rotation matrix instead of rotation vector.
// Note: returns pointer to a static string.
const char* ToCString(const Pose3D &p, bool matrix = false);

// Reads pose from file, ignoring comments.
void ReadPose(const char *filename, Pose3D &pose) throw(BALTException);

inline void SetIdentity(Pose3D &pose)
{
  SetZero(pose.m_position);
  SetZero(pose.m_orientation);
}

inline bool IsIdentity(const Pose3D &pose)
{
  return IsZero(pose.m_position) && IsZero(pose.m_orientation);
}

Pose3D InvertPose3D(const Pose3D &pose);


//---------- Various transformations --------------------------------

/**
 * set 3x3 matrix to identity matrix
 */
void Identity3x3(double M[3][3]);

/**
 * set column of a 3x3 matrix
 */
inline void SetColumn3x3(double M[3][3], int c, double m0, double m1, double m2)
  throw(BALTException)
{
  if(c < 0 || c > 2)
    throw BALTException(__HERE__, "invalid column %d", c);
  M[0][c] = m0;
  M[1][c] = m1;
  M[2][c] = m2;
}

inline void SetColumn3x3(double M[3][3], int c, const Vector3D &v)
{
  SetColumn3x3(M, c, v.m_x, v.m_y, v.m_z);
}

/**
 * get column of a 3x3 matrix
 */
inline void GetColumn3x3(double M[3][3], int c,
    double &m0, double &m1, double &m2) throw(BALTException)
{
  if(c < 0 || c > 2)
    throw BALTException(__HERE__, "invalid column %d", c);
  m0 = M[0][c];
  m1 = M[1][c];
  m2 = M[2][c];
}

inline Vector3D GetColumn3x3(double M[3][3], int c)
{
  Vector3D v;
  double x, y, z;
  GetColumn3x3(M, c, x, y, z);
  v.m_x = x;
  v.m_y = y;
  v.m_z = z;
  return v;
}

/**
 * set row of a 3x3 matrix
 */
inline void SetRow3x3(double M[3][3], int r, double m0, double m1, double m2)
  throw(BALTException)
{
  if(r < 0 || r > 2)
    throw BALTException(__HERE__, "invalid row %d", r);
  M[r][0] = m0;
  M[r][1] = m1;
  M[r][2] = m2;
}

inline void SetRow3x3(double M[3][3], int r, const Vector3D &v)
{
  SetRow3x3(M, r, v.m_x, v.m_y, v.m_z);
}

/**
 * get row of a 3x3 matrix
 */
inline void GetRow3x3(double M[3][3], int r, double &m0, double &m1, double &m2)
  throw(BALTException)
{
  if(r < 0 || r > 2)
    throw BALTException(__HERE__, "invalid row %d", r);
  m0 = M[r][0];
  m1 = M[r][1];
  m2 = M[r][2];
}

inline Vector3D GetRow3x3(double M[3][3], int r)
{
  Vector3D v;
  double x, y, z;
  GetRow3x3(M, r, x, y, z);
  v.m_x = x;
  v.m_y = y;
  v.m_z = z;
  return v;
}

/**
 * Rodrigues formula to convert from rotation vector (pointing along the axis
 * of rotation with length equal to angle of rotation) to 3x3 rotation matrix.
 */
void RotationAxisAngleToMatrix(const Vector3D &r, double R[3][3]);

/**
 * Convert from 3x3 rotation matrix to rotation vector (pointing along the axis
 * of rotation with length equal to angle of rotation).
 */
void RotationMatrixToAxisAngle(double R[3][3], Vector3D &r);

Vector3D Rotate(const Vector3D &r, const Vector3D &p);

Vector3D Translate(const Vector3D &t, const Vector3D p);

// Transform a point from local to global frame.
Vector3D TransformPointToGlobal(const Pose3D &pose, const Vector3D &p);

// Transform a point from global to local frame.
Vector3D TransformPointToLocal(const Pose3D &pose, const Vector3D &p);

// Transform a direction from local to global frame.
Vector3D TransformDirToGlobal(const Pose3D &pose, const Vector3D &d);

// Transform a direction from global to local frame.
Vector3D TransformDirToLocal(const Pose3D &pose, const Vector3D &d);

/**
 * Transform a pose from local to global frame.
 * pose is the reference frame (T2) and p is the pose to be transformed (T1).
 * q = R1 p + t1
 * r = R2 q + t2
 *   = R2 R1 p + R2 t1 + t2
 *   = R3 p + t3
 */
Pose3D TransformPoseToGlobal(const Pose3D &pose, const Pose3D &p);

/**
 * Transform a pose from global to local frame.
 * pose is the reference frame and p is the pose to be transformed.
 */
Pose3D TransformPoseToLocal(const Pose3D &pose, const Pose3D &p);

/**
 * Transform camera image point in [pixel] to 3D view ray in camera
 * co-ordinates.
 * Note: Does not do image undistortion!
 * @param cam  camera
 * @param i  image point in [pixel]
 * @param p  origin of the ray, the focal point of the camera
 * @param d  direction of the ray, unit vector
 */
void ImagePointToLocalRay(const Camera &cam, const Vector2D &i,
    Vector3D &p, Vector3D &d);

/**
 * Transform camera image point in [pixel] to 3D view ray in world co-ordinates.
 * Note: Does not do image undistortion!
 * @param cam  camera
 * @param i  image point in [pixel]
 * @param p  origin of the ray, the focal point of the camera
 * @param d  direction of the ray, unit vector
 */
void ImagePointToGlobalRay(const Camera &cam, const Vector2D &i,
    Vector3D &p, Vector3D &d);

/**
 * Find point on groundplane (normal to z) for a given image point.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param p  image point in pixel coordinates
 * Returns 3D point on ground plane, with z = 0.
 * Note: Does not do image undistortion!
 */
Vector3D ProjectImagePointToGroundplane(double ground_z, const Camera &cam,
    const Vector2D &p);

/**
 * First find point on groundplane (normal to z) for a given image point, then
 * projects given distance (or size) to 3D distance at that point.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param p  image point in pixel coordinates
 * @param s  distance in pixel coordinates
 * Note: Does not do image undistortion!
 */
double ProjectDistanceToGroundplane(double ground_z, const Camera &cam,
    const Vector2D &p, double d);

/**
 * Takes a 2D image ROI and calculates the according SceneObject parameters
 * for a scene object on the ground plane.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param roi  2D image ROI in pixel coordinates
 * @param obj  scene object, pose and bounding box fill be filled
 * Note: Does not do image undistortion!
 */
void ProjectRoiToGroundplane(double ground_z, const Camera &cam, ROI &roi,
    SceneObject &obj);


//---------- Miscellaneous stuff ------------------------------------

void SetDefaultCamera(Camera &cam);


/**
 * Utility function that converts from an indexed entry of an
 * enumeraation, to the corresponding string label -- used currently
 * to translate sceneobject's color, size and shape properties to the
 * corresponding string labels...
 */
string enum2String( int enumVal );


// Constants that provide offsets of the various properties of
// objects...
 static const int COLOR_PROPERTIES_OFFSET = RED; 
 static const int SHAPE_PROPERTIES_OFFSET = SQUARED; 
 static const int SIZE_PROPERTIES_OFFSET = SMALL;
 static const int PG_PROPERTIES_OFFSET = SQUARED;

/**
 * Common function used by several vision components to convert the
 * IDL struct to the OpenCV image format...
 */
IplImage* buffer2image( ImageFrame* _pImage );


/**
 * Function is sort of the inverse of the function above -- OpenCV
 * structure to the format used in the Vision IDL (struct Image)...
 */
void image2buffer( IplImage* _pImage, Image& _buffer );


/**
 * Function draws a polygon on input image, connecting the input
 * pointset in a pair-wise manner, in the color specified as input...
 */
void drawPolygon( IplImage* image, const vector<Vector2D>& points, CvScalar col );


/**
 * String tokenizer by Chris King and tweaked by Petr Prikryl.
 * (see http://gcc.gnu.org/onlinedocs/libstdc++/21_strings/howto.html)
 */
template <typename Container>  
void stringtok( Container &container, std::string const &in,
  const char * const delimiters = " \t\n" )
{
  const std::string::size_type len = in.length();
  std::string::size_type i = 0;

  while( i < len ) {
    // eat leading whitespace
    i = in.find_first_not_of( delimiters, i );
    if( i == std::string::npos )
      return;   // nothing left but white space

    // find the end of the token
    std::string::size_type j = in.find_first_of(delimiters, i);

    // push token
    if( j == std::string::npos ) {
      container.push_back( in.substr(i) );
      return;
    }
    else {
      container.push_back( in.substr(i, j-i) );
    }

    // set up for next loop
    i = j + 1;
  }
}


}

#endif
