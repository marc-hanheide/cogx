/**
 * $Id$
 */

#ifndef VECTOR4_HH
#define VECTOR4_HH

#include <iostream>
#include "PNamespace.hh"
#include "Except.hh"

namespace P 
{

/**
 *  Vector4 
 */
class Vector4 
{

public:
  double x, y, z, w;
  Vector4();
  Vector4(double xx, double yy, double zz, double ww);
  void Set(double xx, double yy, double zz, double ww){ x=xx, y=yy, z=zz, w=ww;}
  Vector4& operator+=(const Vector4 &v);
  Vector4& operator-=(const Vector4 &v);
  Vector4& operator*=(double s);
  Vector4& operator/=(double s) throw(Except);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();
  void Normalise4();

  inline double X() {return x;}
  inline double Y() {return y;}
  inline double Z() {return z;}
  inline double W() {return w;}
};

Vector4 operator-(const Vector4 &v);
bool operator==(const Vector4 &a, const Vector4 &b);
bool operator!=(const Vector4 &a, const Vector4 &b);
Vector4 operator+(const Vector4 &a, const Vector4 &b);
Vector4 operator-(const Vector4 &a, const Vector4 &b);
Vector4 operator*(const double s, const Vector4 &v);
Vector4 operator*(const Vector4 &v, const double s);
Vector4 operator/(const Vector4 &v, const double s) throw(Except);
double Length(const Vector4 &v);
Vector4 Normalise(const Vector4 &v);
double DistanceSquare(const Vector4 &a, const Vector4 &b);
double Distance(const Vector4 &a, const Vector4 &b);

ostream& operator<<(ostream &os, const Vector4 &v);
istream& operator>>(istream &is, Vector4 &v) throw(Except);
string&       operator<<(string &s, const Vector4 &v);
const string& operator>>(const string &s, Vector4 &v);
}

#include "Vector4.ic"

#endif
