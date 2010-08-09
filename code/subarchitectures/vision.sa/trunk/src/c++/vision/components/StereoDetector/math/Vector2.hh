/**
 * @file Vector2.hh
 * @author Zillich, Richtsfeld
 * @date 2006, 2010
 * @version 0.1
 * @brief 2D Vector class.
 */

#ifndef VEC_VECTOR2_HH
#define VEC_VECTOR2_HH

#include <iostream>
#include <string>
#include <stdexcept>

namespace VEC
{

/**
 * @brief 2D vector class.
 */
class Vector2
{
public:
  double x;
  double y;

  Vector2();
  Vector2(double xx, double yy);
  void Set(double xi, double yi) {x = xi; y = yi;}
  Vector2& operator+=(const Vector2 &v);
  Vector2& operator-=(const Vector2 &v);
  Vector2& operator*=(double s);
  Vector2& operator/=(double s) throw (std::runtime_error);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();
  Vector2 Normal();
  Vector2 NormalClockwise();
  Vector2 NormalAntiClockwise();
};

Vector2 operator-(const Vector2 &v);
bool operator==(const Vector2 &a, const Vector2 &b);
bool operator!=(const Vector2 &a, const Vector2 &b);
Vector2 operator+(const Vector2 &a, const Vector2 &b);
Vector2 operator-(const Vector2 &a, const Vector2 &b);
Vector2 operator*(const double s, const Vector2 &v);
Vector2 operator*(const Vector2 &v, const double s);
Vector2 operator/(const Vector2 &v, const double s) throw (std::runtime_error);
double PolarAngle(const Vector2 &v);
double Length(const Vector2 &v);
Vector2 Normalise(const Vector2 &v);
double Dot(const Vector2 &a, const Vector2 &b);
double Cross(const Vector2 &a, const Vector2 &b);
bool LeftOf(const Vector2 &a, const Vector2 &b);
bool CounterClockwiseTo(const Vector2 &a, const Vector2 &b);
bool RightOf(const Vector2 &a, const Vector2 &b);
bool ClockwiseTo(const Vector2 &a, const Vector2 &b);
double DistanceSquare(const Vector2 &a, const Vector2 &b);
double Distance(const Vector2 &a, const Vector2 &b);
double DistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
double AbsDistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
Vector2 Rotate(const Vector2 &a, double phi);
Vector2 CircleCenter(const Vector2 &pi, const Vector2 &pj, const Vector2 &pk);
Vector2 MidPoint(const Vector2 &a, const Vector2 &b);

std::ostream& operator<<(std::ostream &os, const Vector2 &v);
std::istream& operator>>(std::istream &is, Vector2 &v) throw (std::runtime_error);
std::string&       operator<<(std::string &s, const Vector2 &v);
const std::string& operator>>(const std::string &s, Vector2 &v);

Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2) throw (std::runtime_error);
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw (std::runtime_error);
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2);



}

#include "Vector2.ic"
#endif

