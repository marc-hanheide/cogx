/**
 * @file Vector2.cc
 * @author Zillich
 * @date 2004
 * @version 0.1
 * @brief 2D Vector class
 **/

#include "Vector2.hh"

namespace Z
{

/**
 * @brief Returns intersection of lines defined by point p and direction d (needs not be a unit vector).
 * @param p1 Point 1
 * @param d1 Direction from point 1
 * @param p2 Point 2
 * @param d2 Direction from point 2
 */
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1, const Vector2 &p2, const Vector2 &d2) throw(Except)
{
  double d = Cross(d2, d1);
  if(d == 0.)
    throw Except(__HERE__, "lines do not intersect");
  double l = Cross(d2, p2 - p1)/d;
  return Vector2(p1.x + l*d1.x, p1.y + l*d1.y);
}

/**
 * @brief Returns intersection of lines defined by point p and direction d (needs not be a unit vector).\n
 * l1 and l2 contain the lengths along the lines where the intersection lies,
 * measured in lengths of d1 and d2.
 * So if You want l1 and l2 in pixels, d1 and d2 must be unit vectors.
 * @param p1 Point 1
 * @param d1 Direction from point 1
 * @param p2 Point 2
 * @param d2 Direction from point 2
 * @param l1 length along first line, where the intersection lies
 * @param l2 length along second line, where the intersection lies
 */
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
	 const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw(Except)
{
  double d = Cross(d2, d1);
  if(d == 0.)
    throw Except(__HERE__, "lines do not intersect");
  Vector2 p12 = p2 - p1;
  *l1 = Cross(d2, p12)/d;
  *l2 = Cross(d1, p12)/d;
  return Vector2(p1.x + *l1*d1.x, p1.y + *l1*d1.y);
}

/**
 * @brief Checks wheter lines a and b defined by their end points intersect.
 * @param a1 Line a start point
 * @param a2 Line a end point
 * @param b1 Line b start point
 * @param b2 Line b end point
 */
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2)
{
  double l1 = 0., l2 = 0.;
  LineIntersection(a1, a2 - a1, b1, b2 - b1, &l1, &l2);
  return (l1 >= 0. && l1 <= 1.) && (l2 >= 0. && l2 <= 1.);
}

}

