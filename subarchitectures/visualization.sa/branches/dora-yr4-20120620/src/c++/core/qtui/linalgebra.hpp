/*
 * @author: Marko Mahnič
 * @date: September 2011
 */

#ifndef _QTUI_LINALGEBRA_HPP_4E8464AE_
#define _QTUI_LINALGEBRA_HPP_4E8464AE_

#include <cmath>

namespace linalgebra
{

struct Vector3;
struct Matrix3;

struct Vector3
{
   double x, y, z;
   Vector3(double _x=0, double _y=0, double _z=0) {
      x = _x;
      y = _y;
      z = _z;
   }

   void set(double _x, double _y, double _z) {
      x = _x;
      y = _y;
      z = _z;
   }

   void operator= (const Vector3& v) {
      x = v.x;
      y = v.y;
      z = v.z;
   }

   Vector3 operator+ (const Vector3& v) {
      return Vector3( x+v.x, y+v.y, z+v.z );
   }

   Vector3 operator- (const Vector3& v) {
      return Vector3( x-v.x, y-v.y, z-v.z );
   }

   Vector3 operator* (double s) {
      return Vector3( s*x, s*y, s*z );
   }

   Vector3 dot(const Vector3& v) {
      return Vector3( x*v.x, y*v.y, z*v.z );
   }

   Vector3 cross(const Vector3& v) {
      return Vector3( y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x );
   }

   double length() {
      return sqrt(x*x + y*y + z*z);
   }

   void normalize() {
      double s = length();
      if (s == 0) return;
      x = x / s;
      y = y / s;
      z = z / s;
   }

   void rotate(double alpha, Vector3 r);
};

struct Matrix3
{
   double m[9];

   Matrix3() {
      identity();
   }

   Matrix3(const Matrix3& mat) {
      memcpy(&m[0], &mat.m[0], sizeof(m));
   }

   Matrix3(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3){
      m[0] = v1.x; m[3] = v2.x; m[6] = v3.x;
      m[1] = v1.y; m[4] = v2.y; m[7] = v3.y;
      m[2] = v1.z; m[5] = v2.z; m[8] = v3.z;
   }

   Matrix3(double v1x, double v1y, double v1z,
         double v2x, double v2y, double v2z,
         double v3x, double v3y, double v3z)
   {
      m[0] = v1x; m[3] = v2x; m[6] = v3x;
      m[1] = v1y; m[4] = v2y; m[7] = v3y;
      m[2] = v1z; m[5] = v2z; m[8] = v3z;
   }

   void zero() {
      memset(&m[0], 0, sizeof(m));
   }

   void identity() {
      zero();
      m[0] = 1; m[4] = 1; m[8] = 1;
   }

   void operator= (const Matrix3& mat) {
      memcpy(&m[0], &mat.m[0], sizeof(m));
   }

   Matrix3 operator* (const Matrix3 &mat) const {
      Matrix3 ret;
      double* rm = &ret.m[0];
      const double* am = &mat.m[0];
      rm[0] = m[0] * am[0] + m[3] * am[1] + m[6] * am[2];
      rm[1] = m[1] * am[0] + m[4] * am[1] + m[7] * am[2];
      rm[2] = m[2] * am[0] + m[5] * am[1] + m[8] * am[2];
      rm[3] = m[0] * am[3] + m[3] * am[4] + m[6] * am[5];
      rm[4] = m[1] * am[3] + m[4] * am[4] + m[7] * am[5];
      rm[5] = m[2] * am[3] + m[5] * am[4] + m[8] * am[5];
      rm[6] = m[0] * am[6] + m[3] * am[7] + m[6] * am[8];
      rm[7] = m[1] * am[6] + m[4] * am[7] + m[7] * am[8];
      rm[8] = m[2] * am[6] + m[5] * am[7] + m[8] * am[8];
      return ret;
   }

   Vector3 operator* (const Vector3 &v) const;

   Matrix3 transposed() const {
      Matrix3 ret;
      double* rm = &ret.m[0];
      rm[0] = m[0]; rm[3] = m[1]; rm[6] = m[2];
      rm[1] = m[3]; rm[4] = m[4]; rm[7] = m[5];
      rm[2] = m[6]; rm[5] = m[7]; rm[8] = m[8];
      return ret;
   }
};


inline
void Vector3::rotate(double alpha, Vector3 r)
{
   Vector3 s, t;

   if(alpha != 0.0f){

      r.normalize();
      s = r.cross(Vector3(1,0,0));

      if(s.length() < 1e-3)
         s = r.cross(Vector3(0,1,0));

      s.normalize();
      t = r.cross(s);

      Matrix3 Rx(
            1.0f, 0.0f, 0.0f,
            0.0f, cos(alpha), -sin(alpha),
            0.0f, sin(alpha), cos(alpha));
      Matrix3 Mt(r, s, t);
      Matrix3 M(Mt.transposed());

      Matrix3 X = Mt*Rx*M;

      *this = X * *this;
   }
}

inline
Vector3 Matrix3::operator* (const Vector3 &v) const
{
   Vector3 ret;
   ret.x = m[0] * v.x + m[3] * v.y + m[6] * v.z;
   ret.y = m[1] * v.x + m[4] * v.y + m[7] * v.z;
   ret.z = m[2] * v.x + m[5] * v.y + m[8] * v.z;
   return ret;
}

} // namespace
#endif /* _QTUI_LINALGEBRA_HPP_4E8464AE_ */
