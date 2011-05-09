/**
 * Yet another set of 3D matrix utilities.
 *
 * Based on code by Marek Kopicky.
 *
 * A lot of set/get functions are provided to allow You to interface the
 * Pose3 class to representations or linear algebra packages You are already
 * using.
 * 
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef POSE3_H
#define POSE3_H

#include <cassert>
#include <iostream>
#include <opencv/cv.h>
#include <cogxmath_base.h>
#include <Vector3.h>
#include <Matrix33.h>

namespace cogx
{

namespace Math
{

using namespace std;
using namespace cast;

/**
 * Creates matrix from rotation matrix and translation vector
 */
inline Pose3 pose3(const Vector3& pos, const Matrix33& rot)
{
  Pose3 p;
  p.pos = pos;
  p.rot = rot;
  return p;
}

/**
 * Set from a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Pose3 &p, const float m[])
{
  setRow44(p.rot, m);
  set(p.pos, (double)m[3], (double)m[7], (double)m[11]);
}

/**
 * Set from a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Pose3 &p, const double m[])
{
  setRow44(p.rot, m);
  set(p.pos, m[3], m[7], m[11]);
}

/**
 * Set from a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Pose3 &p, const float m[])
{
  setColumn44(p.rot, m);
  set(p.pos, (double)m[12], (double)m[13], (double)m[14]);
}

/**
 * Set from a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Pose3 &p, const double m[])
{
  setColumn44(p.rot, m);
  set(p.pos, m[12], m[13], m[14]);
}

/**
 * Get to a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getRow44(const Pose3 &p, float m[])
{
  getRow44(p.rot, m);
  m[3] = (float)p.pos.x;
  m[7] = (float)p.pos.y;
  m[11] = (float)p.pos.z;
  m[12] = m[13] = m[14] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getRow44(const Pose3 &p, double m[])
{
  getRow44(p.rot, m);
  m[3] = p.pos.x;
  m[7] = p.pos.y;
  m[11] = p.pos.z;
  m[12] = m[13] = m[14] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getColumn44(const Pose3 &p, float m[])
{
  getColumn44(p.rot, m);
  m[12] = (float)p.pos.x;
  m[13] = (float)p.pos.y;
  m[14] = (float)p.pos.z;
  m[3] = m[7] = m[11] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getColumn44(const Pose3 &p, double m[])
{
  getColumn44(p.rot, m);
  m[12] = p.pos.x;
  m[13] = p.pos.y;
  m[14] = p.pos.z;
  m[3] = m[7] = m[11] = 0.;
  m[15] = 1.;
}

inline void setIdentity(Pose3 &p)
{
  setIdentity(p.rot);
  setZero(p.pos);
}

inline bool isIdentity(const Pose3 &p)
{
  return isIdentity(p.rot) && isZero(p.pos);
}

inline bool isFinite(const Pose3 &p)
{
  return isFinite(p.rot) && isFinite(p.pos);
}

/**
 * Transform 3D point from local to world coordinates.
 * b = R * a + p;
 */
inline Vector3 transform(const Pose3 &p, const Vector3& a)
{
  Vector3 b;
  mult(p.rot, a, b);
  b += p.pos;
  return b;
}

/**
 * Transform 3D point from world to local coordinates, assumng R is rotation
 * matrix
 * b = RT * (a - p)
 */
inline Vector3 transformInverse(const Pose3 &p, const Vector3& a)
{
  Vector3 b;
  sub(a, p.pos, b);
  multByTranspose(p.rot, b, b);
  return b;
}

/**
 * Transform 3D direction vector from local to world coordinates.
 * b = R * a;
 */
inline Vector3 transformDirection(const Pose3 &p, const Vector3& a)
{
  Vector3 b;
  mult(p.rot, a, b);
  return b;
}

/**
 * Transform 3D direction vector from world to local coordinates.
 * b = RT * a;
 */
inline Vector3 transformDirectionInverse(const Pose3 &p, const Vector3& a)
{
  Vector3 b;
  multByTranspose(p.rot, a, b);
  return b;
}

/**
 * Transform pose from local to world coordinates.
 * B = [R * AR, R * Ap + p]
 */
inline void transform(const Pose3 &P, const Pose3& A, Pose3& B)
{
  mult(P.rot, A.rot, B.rot);
  B.pos = transform(P, A.pos);
}

/**
 * Transform pose from global to local coordinates.
 * B = [RT * AR, RT * (Ap - p)]
 */
inline void transformInverse(const Pose3 &P, const Pose3& A, Pose3& B)
{
  multByTranspose(P.rot, A.rot, B.rot);
  B.pos = transformInverse(P, A.pos);
}

/**
 * Print a Pose3 to a stream, where rotation is written as rotation
 * vector: '[x y z] [rx ry rz]`
 */
inline void writeText(ostream &os, const Pose3 &p)
{
  writeText(os, p.pos);
  writeTextRotVec(os, p.rot);
}

/**
 * Read a Pose3 from a stream, where rotation is stored as rotation vector.
 * The expected format is: '[x y z] [rx ry rz]', white spaces are ignored.
 */
inline void readText(istream &is, Pose3 &p)
{
  readText(is, p.pos);
  readTextRotVec(is, p.rot);
}

/**
 * Read pose from XML using OpenCV file storage.
 * The pose contains tvec, rvec and/or rmat.
 * If rvec and rmat are given, rvec it will take precedence over rmat.
 */
static void readXML(const string &filename, Pose3 &T)
{
  cv::FileStorage poseFile(filename, cv::FileStorage::READ);
  CvMat *t = (CvMat*)poseFile["tvec"].readObj();
  CvMat *r = (CvMat*)poseFile["rvec"].readObj();
  CvMat *R = (CvMat*)poseFile["rmat"].readObj();
  if(r == 0 && R == 0)
    throw runtime_error("readXML(Pose3): either rvec or rmat must be given");
  if(r != 0)
  {
    fromRotVector(T.rot, vector3(cvmGet(r, 0, 0), cvmGet(r, 1, 0), cvmGet(r, 2, 0)));
  }
  else
  {
    T.rot.m00 = cvmGet(R, 0, 0);  T.rot.m01 = cvmGet(R, 0, 1);  T.rot.m02 = cvmGet(R, 0, 2);
    T.rot.m10 = cvmGet(R, 1, 0);  T.rot.m11 = cvmGet(R, 1, 1);  T.rot.m12 = cvmGet(R, 1, 2);
    T.rot.m20 = cvmGet(R, 2, 0);  T.rot.m21 = cvmGet(R, 2, 1);  T.rot.m22 = cvmGet(R, 2, 2);
  }
  T.pos = vector3(cvmGet(t, 0, 0), cvmGet(t, 1, 0), cvmGet(t, 2, 0));
}

/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline ostream& operator << (ostream &os, const Pose3 &p)
{
  writeText(os, p);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator >> (istream &is, Pose3 &p)
{
  readText(is, p);
  return is;
}

/**
 * Assigns inverse to B, assuming R is orthonormal, i.e. inv(R) = RT, i.e. R
 * is a rotation matrix
 * B = [ART, ART * -Ap]
 */
inline void inverse(const Pose3 &A, Pose3& B)
{
  transpose(A.rot, B.rot);
  B.pos = -A.pos;
  mult(B.rot, B.pos, B.pos);
}

}

}
#endif

