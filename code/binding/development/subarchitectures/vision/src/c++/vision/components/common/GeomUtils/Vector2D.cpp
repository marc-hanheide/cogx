/** @file Vector2D.cpp
 *  @brief A 2D vector.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#include <cmath>
#include "Vector2D.h"


namespace Geom {

using namespace std;

extern const Vector2D VOID_VECTOR2D=Vector2D(-123456789.0,-123456789.0);

Vector2D::Vector2D(const Vector2D& pp)
{
  x = pp.x;
  y = pp.y;
}


Vector2D::Vector2D(float xx, float yy)
{
  x = xx;
  y = yy;
}


Vector2D& Vector2D::operator=(const Vector2D& pp)
{
    if (this == &pp)
	return (*this);
    x = pp.x;
    y = pp.y;
    return( *this );
}

Vector2D::~Vector2D(void) {

}

//-------------------------------------------------------
// Compute the eucilidiean distance between two Vector2D.
//-------------------------------------------------------

float Vector2D::distance(const Vector2D * pr) const
{
  float a, b;

  if (pr == NULL)
    return( 0 );

  a = x - pr->x;
  b = y - pr->y;

  return( sqrt(a*a + b*b) );
}

float Vector2D::distance(const Vector2D& pt) const
{
  float a, b;

  a = x - pt.x;
  b = y - pt.y;

  return( sqrt(a*a + b*b) );
}

//-------------------------------------------------------
// Compute the cosinus of the angle formed by the vector
// (this)-(pr) and the abscisse axel.
//-------------------------------------------------------
float Vector2D::cosinus(const Vector2D * pr) const
{
  float dist = distance(pr);

  if (dist == 0)
    return( 0 );
  else
    return( (pr->g_x() - g_x())/dist );
}

float Vector2D::cosinus(const Vector2D &p) const
{
  float dist = distance(p);

  if (dist == 0)
    return( 0 );
  else
    return( (p.g_x() - this->x)/dist );
}


//-------------------------------------------------------
// Compute the sinus of the angle formed by the vector
// (this)-(pr) and the abscisse axel.
//-------------------------------------------------------
float Vector2D::sinus(const Vector2D * pr) const
{
  float dist = distance(pr);

  if (dist == 0)
    return( 0 );
  else
    return( (pr->g_y() - g_y())/dist );
}

float Vector2D::sinus(const Vector2D &p) const
{
  float dist = distance(p);

  if (dist == 0)
    return( 0 );
  else
    return( (p.g_y() - this->y)/dist );
}

//-------------------------------------------------------
// Compute the angle formed by the vector
// (this)-(pr) and the abscisse axel.
//-------------------------------------------------------
float Vector2D::angle(const Vector2D * pr) const
{
  float sinb = sinus(pr);
  float cosb = cosinus(pr);

  if ( (sinb == 0) && (cosb == 0) )
    return( 0 );
  
  if (sinb < 0)
    return (-acos(cosb));
  else
    return (acos(cosb));
}


float Vector2D::angle(const Vector2D &p) const
{
    float sinb = sinus(p);
    float cosb = cosinus(p);
    
    if ( (sinb == 0) && (cosb == 0) )
	return( 0 );
    
    if (sinb < 0)
	return (-acos(cosb));
    else
	return (acos(cosb));
}


//////////////////////////
bool Vector2D::lies_between(Vector2D * pr1, Vector2D * pr2) const {
	Vector2D * leftPr;
	Vector2D * rightPr;
	float xmin, xmax, my_x;
	float ceta1, ceta2, ceta;
	float dist, mydist;

	if (pr1->g_x() < pr2->g_x()) {
		leftPr = pr1;
		rightPr = pr2;
	}
	else {
		leftPr = pr2;
		rightPr = pr1;
	}
		
	ceta1 = leftPr->angle(rightPr);
	ceta2 = leftPr->angle(this);
	dist = leftPr->distance(*rightPr);
	xmin = leftPr->g_x();
	xmax = leftPr->g_x() + dist;

	ceta = ceta2-ceta1;
	mydist = distance(*leftPr);
	my_x = xmin + (mydist*cos(ceta));

	if ((my_x < xmin) || (my_x > xmax))
		return(false);
	else
		return(true);
}


//-------------------------------------------------------
// Tranform the coordinates.
//-------------------------------------------------------
void Vector2D::transform(affine affTransform) {
	x= (float)(affTransform.S1*x+affTransform.S2*y+affTransform.Tx);
	y= (float)(affTransform.S3*x+affTransform.S4*y+affTransform.Ty);
} 


// Comparision operator
bool Vector2D::operator==(const Vector2D& pp) const
{
    if ((x==pp.g_x()) && (y==pp.g_y()))
	return true;
    else
	return false;
}

bool Vector2D::operator!=(const Vector2D& pp) const
{
    if ((*this) == pp)
	return false;
    else 
	return true;
}

//-----------------
// Display.
//-------------------------------------------------------
ostream& operator<<(ostream& out, const Vector2D& pt)
{
  out.precision(1);
  out.setf(ios::fixed, ios::fixed);
  out<< "(" << pt.x << "," << pt.y << ")";

  return( out );
}

// ostream& operator<<(ostream& out, const Vector2D * ptr) {
//     if (ptr != NULL)
// 	out << *ptr;
//     return out;
// }


}
