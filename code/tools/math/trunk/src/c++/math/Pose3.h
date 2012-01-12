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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
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
  Pose3 T;
  T.pos = pos;
  T.rot = rot;
  return T;
}

/**
 * Set from a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Pose3 &T, const float m[])
{
  setRow44(T.rot, m);
  set(T.pos, (double)m[3], (double)m[7], (double)m[11]);
}

/**
 * Set from a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Pose3 &T, const double m[])
{
  setRow44(T.rot, m);
  set(T.pos, m[3], m[7], m[11]);
}

/**
 * Set from a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Pose3 &T, const float m[])
{
  setColumn44(T.rot, m);
  set(T.pos, (double)m[12], (double)m[13], (double)m[14]);
}

/**
 * Set from a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Pose3 &T, const double m[])
{
  setColumn44(T.rot, m);
  set(T.pos, m[12], m[13], m[14]);
}

/**
 * Get to a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getRow44(const Pose3 &T, float m[])
{
  getRow44(T.rot, m);
  m[3] = (float)T.pos.x;
  m[7] = (float)T.pos.y;
  m[11] = (float)T.pos.z;
  m[12] = m[13] = m[14] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getRow44(const Pose3 &T, double m[])
{
  getRow44(T.rot, m);
  m[3] = T.pos.x;
  m[7] = T.pos.y;
  m[11] = T.pos.z;
  m[12] = m[13] = m[14] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getColumn44(const Pose3 &T, float m[])
{
  getColumn44(T.rot, m);
  m[12] = (float)T.pos.x;
  m[13] = (float)T.pos.y;
  m[14] = (float)T.pos.z;
  m[3] = m[7] = m[11] = 0.;
  m[15] = 1.;
}

/**
 * Get to a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void getColumn44(const Pose3 &T, double m[])
{
  getColumn44(T.rot, m);
  m[12] = T.pos.x;
  m[13] = T.pos.y;
  m[14] = T.pos.z;
  m[3] = m[7] = m[11] = 0.;
  m[15] = 1.;
}

inline void setIdentity(Pose3 &T)
{
  setIdentity(T.rot);
  setZero(T.pos);
}

inline bool isIdentity(const Pose3 &T)
{
  return isIdentity(T.rot) && isZero(T.pos);
}

inline bool isFinite(const Pose3 &T)
{
  return isFinite(T.rot) && isFinite(T.pos);
}

/**
 * Returns true if A and B's elems are within epsilon of each other.
 */
inline bool equals(const Pose3& A, const Pose3& B, double eps)
{
  return equals(A.pos, B.pos, eps) && equals(A.rot, B.rot, eps);
}

/**
 * Assigns inverse to B, assuming R is orthonormal, i.e. inv(R) = R^T, i.e. R
 * is a rotation matrix. poses T1 = [R1, p1], T2 = [R2, p2]
 * T2 = [R1^T, - R1^T * p1]
 */
inline void inverse(const Pose3 &T1, Pose3& T2)
{
  transpose(T1.rot, T2.rot);
  T2.pos = -T1.pos;
  mult(T2.rot, T2.pos, T2.pos);
}

/**
 * Transform 3D point from local to world coordinates, pose T = [R, p].
 * b = R * a + p;
 */
inline Vector3 transform(const Pose3 &T, const Vector3& a)
{
  Vector3 b;
  mult(T.rot, a, b);
  b += T.pos;
  return b;
}

/**
 * Transform 3D point from world to local coordinates, pose T = [R, p].
 * b = R^T * (a - p)
 */
inline Vector3 transformInverse(const Pose3 &T, const Vector3& a)
{
  Vector3 b;
  sub(a, T.pos, b);
  multByTranspose(T.rot, b, b);
  return b;
}

/**
 * Transform 3D direction vector from local to world coordinates, pose T = [R, p].
 * b = R * a;
 */
inline Vector3 transformDirection(const Pose3 &T, const Vector3& a)
{
  Vector3 b;
  mult(T.rot, a, b);
  return b;
}

/**
 * Transform 3D direction vector from world to local coordinates, pose T = [R, p].
 * b = R^T * a;
 */
inline Vector3 transformDirectionInverse(const Pose3 &T, const Vector3& a)
{
  Vector3 b;
  multByTranspose(T.rot, a, b);
  return b;
}

/**
 * Transform pose from local to world coordinates, poses T = [R, p], T1 = [R1, p1], T2 = [R2, p2].
 * T2 = [R * R1, R * p1 + p]
 */
inline void transform(const Pose3 &T, const Pose3& T1, Pose3& T2)
{
  mult(T.rot, T1.rot, T2.rot);
  T2.pos = transform(T, T1.pos);
}

/**
 * Transform pose from global to local coordinates, poses T = [R, p], T1 = [R1, p1], T2 = [R2, p2].
 * T2 = [R^T * R1, R^T * (p1 - p)]
 */
inline void transformInverse(const Pose3 &T, const Pose3& T1, Pose3& T2)
{
  multByTranspose(T.rot, T1.rot, T2.rot);
  T2.pos = transformInverse(T, T1.pos);
}

/**
 * Print a Pose3 to a stream, where rotation is written as rotation
 * vector: '[x y z] [rx ry rz]`
 */
inline void writeTextRotVec(ostream &os, const Pose3 &T)
{
  writeText(os, T.pos);
  writeTextRotVec(os, T.rot);
}

/**
 * Print a Pose3 to a stream, where rotation is written as rotation
 * vector: '[x y z] [rx ry rz]`
 */
inline void writeTextRotMat(ostream &os, const Pose3 &T)
{
  writeText(os, T.pos);
  os << endl;
  writeTextMatrix(os, T.rot);
}

/**
 * Default: Print a Pose3 to a stream, where rotation is written as rotation
 * vector: '[x y z] [rx ry rz]`
 */
inline void writeText(ostream &os, const Pose3 &T)
{
  writeTextRotVec(os, T);
}

/**
 * Read a Pose3 from a stream, where rotation is stored as rotation vector.
 * The expected format is: '[x y z] [rx ry rz]', white spaces are ignored.
 */
inline void readText(istream &is, Pose3 &T)
{
  readText(is, T.pos);
  readTextRotVec(is, T.rot);
}

/**
 * Read pose from XML using OpenCV file storage.
 * The pose contains tvec, rvec and/or rmat.
 * If rvec and rmat are given, rvec it will take precedence over rmat.
 */
inline void readXML(const string &filename, Pose3 &T)
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
inline ostream& operator << (ostream &os, const Pose3 &T)
{
  writeText(os, T);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator >> (istream &is, Pose3 &T)
{
  readText(is, T);
  return is;
}

/**
 * Returns a convenient string representation of a pose.
 */
inline string toString(const Pose3& T)
{
  ostringstream s;
  writeTextRotMat(s, T);
  return s.str();
}

}

}
#endif

