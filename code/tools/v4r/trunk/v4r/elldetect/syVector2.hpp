//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYVECTOR2_HPP
#define  _SYVECTOR2_HPP

#include "multiplatform.hpp"

#include "syExcept.hpp"

NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE 2D vector class.
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzVector2
{
public:
   double x;
   double y;

   CzVector2();
   CzVector2(double xx, double yy);
   void Set(double xi, double yi) {x = xi; y = yi;}
   CzVector2& operator+=(const CzVector2 &v);
   CzVector2& operator-=(const CzVector2 &v);
   CzVector2& operator*=(double s);
   CzVector2& operator/=(double s) __THROW_EXCEPT;
   double NormSquare() const;
   double LengthSquare() const;
   double Norm() const;
   double Length() const;
   void Normalise();
   CzVector2 Normal();
   CzVector2 NormalClockwise();
   CzVector2 NormalAntiClockwise();
};
//end class/////////////////////////////////////////////////////////////////////

CzVector2 operator-(const CzVector2 &v);
bool operator==(const CzVector2 &a, const CzVector2 &b);
bool operator!=(const CzVector2 &a, const CzVector2 &b);
CzVector2 operator+(const CzVector2 &a, const CzVector2 &b);
CzVector2 operator-(const CzVector2 &a, const CzVector2 &b);
CzVector2 operator*(const double s, const CzVector2 &v);
CzVector2 operator*(const CzVector2 &v, const double s);
CzVector2 operator/(const CzVector2 &v, const double s) __THROW_EXCEPT;
double PolarAngle(const CzVector2 &v);
double Length(const CzVector2 &v);
CzVector2 Normalise(const CzVector2 &v);
double Dot(const CzVector2 &a, const CzVector2 &b);
double Cross(const CzVector2 &a, const CzVector2 &b);
bool LeftOf(const CzVector2 &a, const CzVector2 &b);
bool CounterClockwiseTo(const CzVector2 &a, const CzVector2 &b);
bool RightOf(const CzVector2 &a, const CzVector2 &b);
bool ClockwiseTo(const CzVector2 &a, const CzVector2 &b);
double DistanceSquare(const CzVector2 &a, const CzVector2 &b);
double Distance(const CzVector2 &a, const CzVector2 &b);
CzVector2 LineIntersection(const CzVector2 &p1, const CzVector2 &d1,
                           const CzVector2 &p2, const CzVector2 &d2) __THROW_EXCEPT;
CzVector2 LineIntersection(const CzVector2 &p1, const CzVector2 &d1,
                           const CzVector2 &p2, const CzVector2 &d2, double *l1, double *l2) __THROW_EXCEPT;
bool LinesIntersecting(const CzVector2 &a1, const CzVector2 &a2,
                       const CzVector2 &b1, const CzVector2 &b2);
bool LinesIntersecting(const CzVector2 &a1, const CzVector2 &a2,
                       const CzVector2 &b1, const CzVector2 &b2, CzVector2 &isct);
double DistPointToLine(const CzVector2 &q, const CzVector2 &p, const CzVector2 &d);
double AbsDistPointToLine(const CzVector2 &q, const CzVector2 &p, const CzVector2 &d);
CzVector2 Rotate(const CzVector2 &a, double phi);
CzVector2 CircleCenter(const CzVector2 &pi, const CzVector2 &pj,
                       const CzVector2 &pk);
CzVector2 MidPoint(const CzVector2 &a, const CzVector2 &b);

ostream& operator<<(ostream &os, const CzVector2 &v);
istream& operator>>(istream &is, CzVector2 &v) __THROW_EXCEPT;
string&       operator<<(string &s, const CzVector2 &v);
const string& operator>>(const string &s, CzVector2 &v);

NAMESPACE_CLASS_END()

#if defined WIN32 || defined WIN64
   #if defined ELLDETECTEXE
      #include "syVector2.icpp"
   #else
      #include "../C/syVector2.icpp"
   #endif
#else
   #include "syVector2.icpp"
#endif

#endif

