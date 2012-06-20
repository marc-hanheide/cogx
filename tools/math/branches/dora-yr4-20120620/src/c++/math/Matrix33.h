/**
 * Yet another set of 3D matrix utilities.
 *
 * Based on code by Marek Kopicky.
 *
 * A lot of set/get functions are provided to allow You to interface the
 * Matrix33 class to representations or linear algebra packages You are already
 * using.
 * 
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef MATRIX33_H
#define MATRIX33_H

#include <cassert>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cast/core/CASTUtils.hpp>
#include <cogxmath_base.h>
#include <Math.hpp>
#include <Vector3.h>

namespace cogx
{

namespace Math
{

using namespace std;
using namespace cast;

/**
 * Set from a 3x3 float array in row major order.
 */
inline void setRow33(Matrix33 &a, const float m[])
{
  a.m00 = (double)m[0]; a.m01 = (double)m[1]; a.m02 = (double)m[2];
  a.m10 = (double)m[3]; a.m11 = (double)m[4]; a.m12 = (double)m[5];
  a.m20 = (double)m[6]; a.m21 = (double)m[7]; a.m22 = (double)m[8];
}

/**
 * Set from a 3x3 float array in row major order.
 */
inline void setRow33(Matrix33 &a, const float m[3][3])
{
  a.m00 = (double)m[0][0]; a.m01 = (double)m[0][1]; a.m02 = (double)m[0][2];
  a.m10 = (double)m[1][0]; a.m11 = (double)m[1][1]; a.m12 = (double)m[1][2];
  a.m20 = (double)m[2][0]; a.m21 = (double)m[2][1]; a.m22 = (double)m[2][2];
}

/**
 * Set from a 3x3 double array in row major order.
 */
inline void setRow33(Matrix33 &a, const double m[])
{
  a.m00 = m[0]; a.m01 = m[1]; a.m02 = m[2];
  a.m10 = m[3]; a.m11 = m[4]; a.m12 = m[5];
  a.m20 = m[6]; a.m21 = m[7]; a.m22 = m[8];
}

/**
 * Set from a 3x3 double array in row major order.
 */
inline void setRow33(Matrix33 &a, const double m[3][3])
{
  a.m00 = m[0][0]; a.m01 = m[0][1]; a.m02 = m[0][2];
  a.m10 = m[1][0]; a.m11 = m[1][1]; a.m12 = m[1][2];
  a.m20 = m[2][0]; a.m21 = m[2][1]; a.m22 = m[2][2];
}

/**
 * Set from a 3x3 float array in column major order.
 */
inline void setColumn33(Matrix33 &a, const float m[])
{
  a.m00 = (double)m[0]; a.m01 = (double)m[3]; a.m02 = (double)m[6];
  a.m10 = (double)m[1]; a.m11 = (double)m[4]; a.m12 = (double)m[7];
  a.m20 = (double)m[2]; a.m21 = (double)m[5]; a.m22 = (double)m[8];
}

/**
 * Set from a 3x3 double array in column major order.
 */
inline void setColumn33(Matrix33 &a, const double m[])
{
  a.m00 = m[0]; a.m01 = m[3]; a.m02 = m[6];
  a.m10 = m[1]; a.m11 = m[4]; a.m12 = m[7];
  a.m20 = m[2]; a.m21 = m[5]; a.m22 = m[8];
}

/**
 * Get to a 3x3 float array in row major order.
 */
inline void getRow33(const Matrix33 &a, float m[])
{
  m[0] = (float)a.m00;  m[1] = (float)a.m01;  m[2] = (float)a.m02;
  m[3] = (float)a.m10;  m[4] = (float)a.m11;  m[5] = (float)a.m12;
  m[6] = (float)a.m20;  m[7] = (float)a.m21;  m[8] = (float)a.m22;
}

/**
 * Get to a 3x3 double array in row major order.
 */
inline void getRow33(const Matrix33 &a, double m[])
{
  m[0] = a.m00;  m[1] = a.m01;  m[2] = a.m02;
  m[3] = a.m10;  m[4] = a.m11;  m[5] = a.m12;
  m[6] = a.m20;  m[7] = a.m21;  m[8] = a.m22;
}

/**
 * Get to a 3x3 float array in column major order.
 */
inline void getColumn33(const Matrix33 &a, float m[])
{
  m[0] = (float)a.m00;  m[3] = (float)a.m01;  m[6] = (float)a.m02;
  m[1] = (float)a.m10;  m[4] = (float)a.m11;  m[7] = (float)a.m12;
  m[2] = (float)a.m20;  m[5] = (float)a.m21;  m[8] = (float)a.m22;
}

/**
 * Get to a 3x3 double array in column major order.
 */
inline void getColumn33(const Matrix33 &a, double m[])
{
  m[0] = a.m00;  m[3] = a.m01;  m[6] = a.m02;
  m[1] = a.m10;  m[4] = a.m11;  m[7] = a.m12;
  m[2] = a.m20;  m[5] = a.m21;  m[8] = a.m22;
}

/**
 * Set from a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Matrix33 &a, const float m[])
{
  a.m00 = (double)m[0]; a.m01 = (double)m[1]; a.m02 = (double)m[2];
  a.m10 = (double)m[4]; a.m11 = (double)m[5]; a.m12 = (double)m[6];
  a.m20 = (double)m[8]; a.m21 = (double)m[9]; a.m22 = (double)m[10];
}

/**
 * Set from a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setRow44(Matrix33 &a, const double m[])
{
  a.m00 = m[0]; a.m01 = m[1]; a.m02 = m[2];
  a.m10 = m[4]; a.m11 = m[5]; a.m12 = m[6];
  a.m20 = m[8]; a.m21 = m[9]; a.m22 = m[10];
}

/**
 * Set from a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Matrix33 &a, const float m[])
{
  a.m00 = (double)m[0]; a.m01 = (double)m[4]; a.m02 = (double)m[8];
  a.m10 = (double)m[1]; a.m11 = (double)m[5]; a.m12 = (double)m[9];
  a.m20 = (double)m[2]; a.m21 = (double)m[6]; a.m22 = (double)m[10];
}

/**
 * Set from a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix.
 */
inline void setColumn44(Matrix33 &a, const double m[])
{
  a.m00 = m[0]; a.m01 = m[4]; a.m02 = m[8];
  a.m10 = m[1]; a.m11 = m[5]; a.m12 = m[9];
  a.m20 = m[2]; a.m21 = m[6]; a.m22 = m[10];
}

/**
 * Get to a 4x4 float array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix. The rest of the 4x4 array is left
 * unchanged.
 */
inline void getRow44(const Matrix33 &a, float m[])
{
  m[0] = (float)a.m00;  m[1] = (float)a.m01;  m[2] = (float)a.m02;
  m[4] = (float)a.m10;  m[5] = (float)a.m11;  m[6] = (float)a.m12;
  m[8] = (float)a.m20;  m[9] = (float)a.m21;  m[10]= (float)a.m22;
}

/**
 * Get to a 4x4 double array in row major order, where the upper 3x3 part
 * corresponds to the rotation matrix. The rest of the 4x4 array is left
 * unchanged.
 */
inline void getRow44(const Matrix33 &a, double m[])
{
  m[0] = a.m00;  m[1] = a.m01;  m[2] = a.m02;
  m[4] = a.m10;  m[5] = a.m11;  m[6] = a.m12;
  m[8] = a.m20;  m[9] = a.m21;  m[10]= a.m22;
}

/**
 * Get to a 4x4 float array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix. The rest of the 4x4 array is left
 * unchanged.
 */
inline void getColumn44(const Matrix33 &a, float m[])
{
  m[0] = (float)a.m00;  m[4] = (float)a.m01;  m[8] = (float)a.m02;
  m[1] = (float)a.m10;  m[5] = (float)a.m11;  m[9] = (float)a.m12;
  m[2] = (float)a.m20;  m[6] = (float)a.m21;  m[10]= (float)a.m22;
}

/**
 * Get to a 4x4 double array in column major order, where the upper 3x3 part
 * corresponds to the rotation matrix. The rest of the 4x4 array is left
 * unchanged.
 */
inline void getColumn44(const Matrix33 &a, double m[])
{
  m[0] = a.m00;  m[4] = a.m01;  m[8] = a.m02;
  m[1] = a.m10;  m[5] = a.m11;  m[9] = a.m12;
  m[2] = a.m20;  m[6] = a.m21;  m[10]= a.m22;
}

inline void setRow(Matrix33 &a, int row, const Vector3& v)
{
  assert(0 <= row && row <= 2);
  double *r = &a.m00 + 3*row;
  r[0] = v.x;
  r[1] = v.y;
  r[2] = v.z;
}

inline void setColumn(Matrix33 &a, int col, const Vector3& v)
{
  assert(0 <= col && col <= 2);
  double *c = &a.m00 + col;
  c[0] = v.x;
  c[3] = v.y;
  c[6] = v.z;
}

inline Vector3 getRow(const Matrix33 &a, int row)
{
  assert(0 <= row && row <= 2);
  const double *r = &a.m00 + 3*row;
  return vector3(r[0], r[1], r[2]);
}

inline Vector3 getColumn(const Matrix33 &a, int col)
{
  assert(0 <= col && col <= 2);
  const double *c = &a.m00 + col;
  return vector3(c[0], c[3], c[6]);
}

inline void set(Matrix33 &a, int row, int col, double d)
{
  assert(0 <= row && row <= 2);
  assert(0 <= col && col <= 2);
  double *m = &a.m00;
  m[3*row + col] = d;
}

inline double get(const Matrix33 &a, int row, int col)
{
  assert(0 <= row && row <= 2);
  assert(0 <= col && col <= 2);
  const double *m = &a.m00;
  return m[3*row + col];
}

inline bool isIdentity(const Matrix33 &a)
{
  return
    a.m00 == REAL_ONE && iszero(a.m01)     && iszero(a.m02) &&
    iszero(a.m10)     && a.m11 == REAL_ONE && iszero(a.m12) &&
    iszero(a.m20)     && iszero(a.m21)     && a.m22 != REAL_ONE;
}

inline bool isZero(const Matrix33 &a)
{
  return
    iszero(a.m00) && iszero(a.m01) && iszero(a.m02) &&
    iszero(a.m10) && iszero(a.m11) && iszero(a.m12) &&
    iszero(a.m20) && iszero(a.m21) && iszero(a.m22);
}

inline bool isFinite(const Matrix33 &a)
{
  return
    isfinite(a.m00) && isfinite(a.m01) && isfinite(a.m02) &&
    isfinite(a.m10) && isfinite(a.m11) && isfinite(a.m12) &&
    isfinite(a.m20) && isfinite(a.m21) && isfinite(a.m22);
}

/**
 * Returns true if A and B's elems are within epsilon of each other.
 */
inline bool equals(const Matrix33& A, const Matrix33& B, double eps)
{
  return equals(A.m00, B.m00, eps) && equals(A.m01, B.m01, eps) && equals(A.m02, B.m02, eps) &&
         equals(A.m10, B.m10, eps) && equals(A.m11, B.m11, eps) && equals(A.m12, B.m12, eps) &&
         equals(A.m20, B.m20, eps) && equals(A.m21, B.m21, eps) && equals(A.m22, B.m22, eps);
}

inline void setZero(Matrix33 &a)
{
  a.m00 = REAL_ZERO;   a.m01 = REAL_ZERO;   a.m02 = REAL_ZERO;
  a.m10 = REAL_ZERO;   a.m11 = REAL_ZERO;   a.m12 = REAL_ZERO;
  a.m20 = REAL_ZERO;   a.m21 = REAL_ZERO;   a.m22 = REAL_ZERO;
}

inline void setIdentity(Matrix33 &a)
{
  a.m00 = REAL_ONE;   a.m01 = REAL_ZERO;  a.m02 = REAL_ZERO;
  a.m10 = REAL_ZERO;  a.m11 = REAL_ONE;   a.m12 = REAL_ZERO;
  a.m20 = REAL_ZERO;  a.m21 = REAL_ZERO;  a.m22 = REAL_ONE;
}

/**
 * a = -a
 */
inline void setNegative(Matrix33 &a)
{
  a.m00 = -a.m00;   a.m01 = -a.m01;   a.m02 = -a.m02;
  a.m10 = -a.m10;   a.m11 = -a.m11;   a.m12 = -a.m12;
  a.m20 = -a.m20;   a.m21 = -a.m21;   a.m22 = -a.m22;
}

/**
 * a = transpose(a)
 */
inline void setTranspose(Matrix33 &a)
{
  swap(a.m01, a.m10);
  swap(a.m12, a.m21);
  swap(a.m02, a.m20);
}

/**
 * Sets matrix to the diagonal matrix.
 */
inline void setDiagonal(Matrix33 &a, const Vector3 &v)
{
  a.m00 = v.x;        a.m01 = REAL_ZERO;  a.m02 = REAL_ZERO;
  a.m10 = REAL_ZERO;  a.m11 = v.y;        a.m12 = REAL_ZERO;
  a.m20 = REAL_ZERO;  a.m21 = REAL_ZERO;  a.m22 = v.z;
}

inline Matrix33& operator += (Matrix33 &a, const Matrix33 &b)
{
  a.m00 += a.m00;  a.m01 += a.m01;  a.m02 += a.m02;
  a.m10 += a.m10;  a.m11 += a.m11;  a.m12 += a.m12;
  a.m20 += a.m20;  a.m21 += a.m21;  a.m22 += a.m22;
  return a;
}

inline Matrix33& operator -= (Matrix33 &a, const Matrix33 &b)
{
  a.m00 -= b.m00;  a.m01 -= b.m01;  a.m02 -= b.m02;
  a.m10 -= b.m10;  a.m11 -= b.m11;  a.m12 -= b.m12;
  a.m20 -= b.m20;  a.m21 -= b.m21;  a.m22 -= b.m22;
  return a;
}

inline Matrix33& operator *= (Matrix33 &a, double s)
{
  a.m00 *= s;   a.m01 *= s;   a.m02 *= s;
  a.m10 *= s;   a.m11 *= s;   a.m12 *= s;
  a.m20 *= s;   a.m21 *= s;   a.m22 *= s;
  return a;
}

inline Matrix33& operator /= (Matrix33 &a, double s)
{
  assert(isnormal(s));
  a.m00 /= s;   a.m01 /= s;   a.m02 /= s;
  a.m10 /= s;   a.m11 /= s;   a.m12 /= s;
  a.m20 /= s;   a.m21 /= s;   a.m22 /= s;
  return a;
}

/**
 * C = A * [ b.x 0 0; 0 b.y 0; 0 0 b.z];
 */
inline void multDiag(const Matrix33& A, const Vector3& b, Matrix33& C)
{
  C.m00 = A.m00*b.x; C.m01 = A.m01*b.y; C.m02 = A.m02*b.z;
  C.m10 = A.m10*b.x; C.m11 = A.m11*b.y; C.m12 = A.m12*b.z;
  C.m20 = A.m20*b.x; C.m21 = A.m21*b.y; C.m22 = A.m22*b.z;
}

/**
 * C = transpose(A) * [ b.x 0 0; 0 b.y 0; 0 0 b.z];
 */
inline void multDiagTranspose(const Matrix33& A, const Vector3& b, Matrix33& C)
{
  // if A and C refered to the same matrix, things would get mixed up
  if (&A != &C)
  {
    C.m00 = A.m00*b.x; C.m01 = A.m10*b.y; C.m02 = A.m20*b.z;
    C.m10 = A.m01*b.x; C.m11 = A.m11*b.y; C.m12 = A.m21*b.z;
    C.m20 = A.m02*b.x; C.m21 = A.m12*b.y; C.m22 = A.m22*b.z;
  }
  else
  {
    setTranspose(C);
    multDiag(C, b, C);
  }
}

/**
 * c = A * b
 */
inline void mult(const Matrix33& A, const Vector3& b, Vector3& c)
{
  // note: temps needed in case b and c refer to the same vector
  double x = A.m00 * b.x + A.m01 * b.y + A.m02 * b.z;
  double y = A.m10 * b.x + A.m11 * b.y + A.m12 * b.z;
  double z = A.m20 * b.x + A.m21 * b.y + A.m22 * b.z;

  c.x = x;
  c.y = y;
  c.z = z;
}

/**
 * c = transpose(A) * b
 */
inline void multByTranspose(const Matrix33& A, const Vector3& b, Vector3& c)
{
  // note: temps needed in case b and c refer to the same vector
  double x = A.m00 * b.x + A.m10 * b.y + A.m20 * b.z;
  double y = A.m01 * b.x + A.m11 * b.y + A.m21 * b.z;
  double z = A.m02 * b.x + A.m12 * b.y + A.m22 * b.z;

  c.x = x;
  c.y = y;
  c.z = z;
}

/**
 * C = A + B
 */
inline void add(const Matrix33& A, const Matrix33& B, Matrix33& C)
{
  C.m00 = A.m00 + B.m00;  C.m01 = A.m01 + B.m01;  C.m02 = A.m02 + B.m02;
  C.m10 = A.m10 + B.m10;  C.m11 = A.m11 + B.m11;  C.m12 = A.m12 + B.m12;
  C.m20 = A.m20 + B.m20;  C.m21 = A.m21 + B.m21;  C.m22 = A.m22 + B.m22;
}

/**
 * C = A - B
 */
inline void sub(const Matrix33& A, const Matrix33& B, Matrix33& C)
{
  C.m00 = A.m00 - B.m00;  C.m01 = A.m01 - B.m01;  C.m02 = A.m02 - B.m02;
  C.m10 = A.m10 - B.m10;  C.m11 = A.m11 - B.m11;  C.m12 = A.m12 - B.m12;
  C.m20 = A.m20 - B.m20;  C.m21 = A.m21 - B.m21;  C.m22 = A.m22 - B.m22;
}

/**
 * B = s * A;
 */
inline void mult(double s, const Matrix33& A, Matrix33& B)
{
  B.m00 = A.m00 * s;  B.m01 = A.m01 * s;  B.m02 = A.m02 * s;
  B.m10 = A.m10 * s;  B.m11 = A.m11 * s;  B.m12 = A.m12 * s;
  B.m20 = A.m20 * s;  B.m21 = A.m21 * s;  B.m22 = A.m22 * s;
}

/**
 * C = A * B
 */
inline void mult(const Matrix33& A, const Matrix33& B, Matrix33& C)
{
  // note: temps needed in case C and A or B refer to the same matrix
  double a00 = A.m00 * B.m00 + A.m01 * B.m10 + A.m02 * B.m20;
  double a01 = A.m00 * B.m01 + A.m01 * B.m11 + A.m02 * B.m21;
  double a02 = A.m00 * B.m02 + A.m01 * B.m12 + A.m02 * B.m22;

  double a10 = A.m10 * B.m00 + A.m11 * B.m10 + A.m12 * B.m20;
  double a11 = A.m10 * B.m01 + A.m11 * B.m11 + A.m12 * B.m21;
  double a12 = A.m10 * B.m02 + A.m11 * B.m12 + A.m12 * B.m22;

  double a20 = A.m20 * B.m00 + A.m21 * B.m10 + A.m22 * B.m20;
  double a21 = A.m20 * B.m01 + A.m21 * B.m11 + A.m22 * B.m21;
  double a22 = A.m20 * B.m02 + A.m21 * B.m12 + A.m22 * B.m22;

  C.m00 = a00;  C.m01 = a01;  C.m02 = a02;
  C.m10 = a10;  C.m11 = a11;  C.m12 = a12;
  C.m20 = a20;  C.m21 = a21;  C.m22 = a22;
}

/**
 * C = AT * B
 */
inline void multByTranspose(const Matrix33& A, const Matrix33& B, Matrix33& C)
{
  // note: temps needed in case C and A or B refer to the same matrix
  double a00 = A.m00 * B.m00 + A.m10 * B.m10 + A.m20 * B.m20;
  double a01 = A.m00 * B.m01 + A.m10 * B.m11 + A.m20 * B.m21;
  double a02 = A.m00 * B.m02 + A.m10 * B.m12 + A.m20 * B.m22;

  double a10 = A.m01 * B.m00 + A.m11 * B.m10 + A.m21 * B.m20;
  double a11 = A.m01 * B.m01 + A.m11 * B.m11 + A.m21 * B.m21;
  double a12 = A.m01 * B.m02 + A.m11 * B.m12 + A.m21 * B.m22;

  double a20 = A.m02 * B.m00 + A.m12 * B.m10 + A.m22 * B.m20;
  double a21 = A.m02 * B.m01 + A.m12 * B.m11 + A.m22 * B.m21;
  double a22 = A.m02 * B.m02 + A.m12 * B.m12 + A.m22 * B.m22;

  C.m00 = a00;  C.m01 = a01;  C.m02 = a02;
  C.m10 = a10;  C.m11 = a11;  C.m12 = a12;
  C.m20 = a20;  C.m21 = a21;  C.m22 = a22;
}

inline double trace(const Matrix33 &A)
{
  return A.m00 + A.m11 + A.m22;
}

inline double determinant(const Matrix33 &A)
{
  return
    A.m00*A.m11*A.m22 + A.m01*A.m12*A.m20 +
    A.m02*A.m10*A.m21 - A.m02*A.m11*A.m20 -
    A.m01*A.m10*A.m22 - A.m00*A.m12*A.m21;
}

/**
 * @return false if singular (i.e. if no inverse exists)
 */
inline bool inverse(const Matrix33& A, Matrix33 &B)
{
  double temp00 = A.m11*A.m22 - A.m12*A.m21;
  double temp01 = A.m02*A.m21 - A.m01*A.m22;
  double temp02 = A.m01*A.m12 - A.m02*A.m11;

  double temp10 = A.m12*A.m20 - A.m10*A.m22;
  double temp11 = A.m00*A.m22 - A.m02*A.m20;
  double temp12 = A.m02*A.m10 - A.m00*A.m12;

  double temp20 = A.m10*A.m21 - A.m11*A.m20;
  double temp21 = A.m01*A.m20 - A.m00*A.m21;
  double temp22 = A.m00*A.m11 - A.m01*A.m10;

  double det = temp00*A.m00 + temp01*A.m10 + temp02*A.m20;

  if (iszero(det))
    return false;

  B.m00 = temp00/det;  B.m01 = temp01/det;  B.m02 = temp02/det;
  B.m10 = temp10/det;  B.m11 = temp11/det;  B.m12 = temp12/det;
  B.m20 = temp20/det;  B.m21 = temp21/det;  B.m22 = temp22/det;

  return true;
}

inline void transpose(const Matrix33& A, Matrix33 &B)
{
  if (&A != &B)
  {
    B.m00 = A.m00;  B.m01 = A.m10;  B.m02 = A.m20;
    B.m10 = A.m01;  B.m11 = A.m11;  B.m12 = A.m21;
    B.m20 = A.m02;  B.m21 = A.m12;  B.m22 = A.m22;
  }
  else
    setTranspose(B);
}

inline Matrix33& operator *= (Matrix33 &a, const Matrix33& b)
{
  mult(a, b, a);
  return a;
}

inline Vector3 operator * (const Matrix33& M, const Vector3& v)
{
  Vector3 tmp;
  mult(M, v, tmp);
  return tmp;
}

inline Matrix33 operator - (const Matrix33& A, const Matrix33& B)
{
  Matrix33 tmp;
  sub(A, B, tmp);
  return tmp;
}

inline Matrix33 operator + (const Matrix33& A, const Matrix33& B)
{
  Matrix33 tmp;
  add(A, B, tmp);
  return tmp;
}

inline Matrix33 operator * (const Matrix33& A, const Matrix33& B)
{
  Matrix33 tmp;
  mult(A, B, tmp);
  return tmp;
}

inline Matrix33 operator * (const Matrix33& M, double s)
{
  Matrix33 tmp;
  mult(s, M, tmp);
  return tmp;
}

inline Matrix33 operator * (double s, const Matrix33& M)
{
  Matrix33 tmp;
  mult(s, M, tmp);
  return tmp;
}

/**
 * Creates rotation matrix from rotation around axis for normalised axis.
 * Rodrigues' formula for exponential maps:
 * exp(axis^ angle) = Id + axis^ sin(angle) + axis^ axis^ (1 - cos(angle)),
 * where axis^ is so(3) matrix generated from axis.
 */
inline void fromAngleAxis(Matrix33 &m, double angle, const Vector3& axis)
{
  double s = sin(angle), c = cos(angle);
  double v = REAL_ONE - c, x = axis.x*v, y = axis.y*v, z = axis.z*v;

  m.m00 = axis.x*x + c;
  m.m01 = axis.x*y - axis.z*s;
  m.m02 = axis.x*z + axis.y*s;

  m.m10 = axis.y*x + axis.z*s;
  m.m11 = axis.y*y + c;
  m.m12 = axis.y*z - axis.x*s;

  m.m20 = axis.z*x - axis.y*s;
  m.m21 = axis.z*y + axis.x*s;
  m.m22 = axis.z*z + c;
}

/**
 * Creates rotation matrix from rotation vector (= axis plus angle).
 */
inline void fromRotVector(Matrix33 &m, const Vector3& r)
{
  if(isZero(r))
  {
    setIdentity(m);
  }
  else
  {
    Vector3 axis(r);
    double angle = normalise(axis);
    fromAngleAxis(m, angle, axis);
  }
}

/**
 * Returns rotation angle and axis of rotation matrix.
 */
inline void toAngleAxis(const Matrix33 &m, double& angle, Vector3& axis) 
{
  // Mote: Don't make this eps smaller! Case 2 (angle = pi) might
  // otherwise be missed if the trace is very close to but not
  // exactly -1.
  const double EPS_ANGLE = 1e-6;
  angle = acos((trace(m) - 1.) / 2.);
  // if angle is 0
  if(equals(angle, 0., EPS_ANGLE))
  {
    // for zero angle, axis direction does not matter, any unit vector is fine
    set(axis, 1., 0., 0.);
  }
  // if angle is PI
  else if(equals(angle, M_PI, EPS_ANGLE))
  {
    // this requires special attention as m.mij == m.mji, thus according to
    // the general formula the axis would be zero
    // if m00 is maximum
    if(m.m00 > m.m11 && m.m00 > m.m22)
    {
      axis.x = sqrt(m.m00 - m.m11 - m.m22 + 1.)/2.;
      axis.y = m.m01/(2.*axis.x);
      axis.z = m.m02/(2.*axis.x);
    }
    // if m11 is maximum
    else if(m.m11 > m.m22)
    {
      axis.y = sqrt(m.m11 - m.m00 - m.m22 + 1.)/2.;
      axis.x = m.m01/(2.*axis.y);
      axis.z = m.m12/(2.*axis.y);
    }
    // if m22 is maximum
    else
    {
      axis.z = sqrt(m.m22 - m.m00 - m.m11 + 1.)/2.;
      axis.x = m.m02/(2.*axis.z);
      axis.y = m.m12/(2.*axis.z);
    }
  }
  // general case
  else
  {
    double s = 2. * sin(angle);
    set(axis, (m.m21 - m.m12)/s, (m.m02 - m.m20)/s, (m.m10 - m.m01)/s);
  }
}

/**
 * Returns rotation vector (= axis plus angle).
 */
inline void toRotVector(const Matrix33 &m, Vector3& r)
{
  double angle;
  toAngleAxis(m, angle, r);
  r *= angle;
}

/**
 * a = rotation matrix around X axis
 */
inline void fromRotX(Matrix33 &a, double angle)
{
  double s = sin(angle), c = cos(angle);

  a.m00 = REAL_ONE; a.m01 = REAL_ZERO; a.m02 = REAL_ZERO;
  a.m10 = REAL_ZERO; a.m11 = c;   a.m12 = -s;
  a.m20 = REAL_ZERO; a.m21 = s;   a.m22 = c;
}

/**
 * a = rotation matrix around Y axis
 */
inline void fromRotY(Matrix33 &a, double angle)
{
  double s = sin(angle), c = cos(angle);

  a.m00 = c;   a.m01 = REAL_ZERO; a.m02 = s;
  a.m10 = REAL_ZERO; a.m11 = REAL_ONE; a.m12 = REAL_ZERO;
  a.m20 = -s;  a.m21 = REAL_ZERO; a.m22 = c;
}

/**
 * a = rotation matrix around Z axis
 */
inline void fromRotZ(Matrix33 &a, double angle)
{
  double s = sin(angle), c = cos(angle);

  a.m00 = c;   a.m01 = -s;  a.m02 = REAL_ZERO;
  a.m10 = s;   a.m11 = c;   a.m12 = REAL_ZERO;
  a.m20 = REAL_ZERO; a.m21 = REAL_ZERO; a.m22 = REAL_ONE;
}

/** 
 * Rotation matrix from Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
 *   roll, pitch, yaw  in <-PI/2, PI/2>
 */
inline void fromEuler(Matrix33 &a, double roll, double pitch, double yaw)
{
  double sg = sin(roll), cg = cos(roll);
  double sb = sin(pitch), cb = cos(pitch);
  double sa = sin(yaw), ca = cos(yaw);

  a.m00 = ca*cb;  a.m01 = ca*sb*sg - sa*cg;   a.m02 = ca*sb*cg + sa*sg;
  a.m10 = sa*cb;  a.m11 = sa*sb*sg + ca*cg;   a.m12 = sa*sb*cg - ca*sg;
  a.m20 = -sb;    a.m21 = cb*sg;              a.m22 = cb*cg;
}

inline void fromRPY(Matrix33 &a, double roll, double pitch, double yaw)
{
  fromEuler(a, roll, pitch, yaw);
}

/**
 * Rotation matrix to Euler angles - rotation about: X (roll) -> Y(pitch) -> Z(yaw).
 */
inline void toEuler(const Matrix33 &a, double &roll, double &pitch, double &yaw)
{
  roll = atan2(a.m21, a.m22);
  pitch = atan2(-a.m20, sqrt(a.m21*a.m21 + a.m22*a.m22));
  yaw = atan2(a.m10, a.m00);
}

inline void toRPY(const Matrix33 &a, double &roll, double &pitch, double &yaw)
{
  toEuler(a, roll, pitch, yaw);
}

/**
 * Print as rotation vector in text form to a stream,
 */
inline void writeTextRotVec(ostream &os, const Matrix33 &m)
{
  Vector3 r;
  toRotVector(m, r);
  writeText(os, r);
}

/**
 * Read as rotation vector in text from from a streaa.
 */
inline void readTextRotVec(istream &is, Matrix33 &m)
{
  Vector3 r;
  readText(is, r);
  fromRotVector(m, r);
}

/**
 * Print as rotation matrix in text form to a streaa.
 */
inline void writeTextMatrix(ostream &os, const Matrix33 &a)
{
  const streamsize w = 7;
  streamsize prec = os.precision(3);
  ios::fmtflags flags = os.setf(ios::fixed, ios::floatfield);
  os << '[' << setw(w) << a.m00 << ' ' << setw(w) << a.m01 << ' ' << setw(w) << a.m02 << endl;
  os << ' ' << setw(w) << a.m10 << ' ' << setw(w) << a.m11 << ' ' << setw(w) << a.m12 << endl;
  os << ' ' << setw(w) << a.m20 << ' ' << setw(w) << a.m21 << ' ' << setw(w) << a.m22 << ']';
  os.precision(prec);
  os.setf(flags, ios::floatfield);
}

/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline ostream& operator << (ostream &os, const Matrix33 &m)
{
  writeTextRotVec(os, m);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline istream& operator >> (istream &is, Matrix33 &m)
{
  readTextRotVec(is, m);
  return is;
}

/**
 * Returns a convenient string representation of a matrix.
 */
inline string toString(const Matrix33& m)
{
  ostringstream s;
  writeTextMatrix(s, m);
  return s.str();
}

}

}
#endif

