/** @file Vector3D.cpp
 *  @brief A 3D vector.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#include "Vector3D.h"

namespace Geom {

using namespace std;

Vector3D::Vector3D() {
    m_x = 0.0;    m_y = 0.0;    m_z = 0.0;
}
    
Vector3D::Vector3D(float x_,float y_, float z_) {
    set(x_,y_,z_);
}

Vector3D::Vector3D(const Vector3D &src) {
    set(src.x(), src.y(), src.z());
}

Vector3D::~Vector3D() {
}


Vector3D& Vector3D::operator=(const Vector3D& src) {
    if (this == &src)
	return(*this);
    
    set(src.x(), src.y(), src.z());
    return (*this);
}


void Vector3D::set(float x_,float y_, float z_) {
    m_x = x_; m_y = y_; m_z = z_;
}

float Vector3D::distance(const Vector3D& otherpt) const{
    return sqrt( (*this-otherpt)*(*this-otherpt) );
}


float Vector3D::norm() const {
    return sqrt( (*this)*(*this) );  
}


const Vector2D Vector3D::orthoz() {
    return Vector2D(m_x, m_y);
}

void Vector3D::print(ostream *os) const {
    *os << m_x << " " << m_y << " " << m_z << " ";
}


const Vector3D operator+(const Vector3D& pt1, const Vector3D& pt2) {
    return Vector3D(pt1.x() + pt2.x(),
		    pt1.y() + pt2.y(),
		    pt1.z() + pt2.z());
}

const Vector3D operator-(const Vector3D&  pt1, const Vector3D& pt2) {
    return Vector3D(pt1.x() - pt2.x(),
		    pt1.y() - pt2.y(),
		    pt1.z() - pt2.z());
}

float operator*(const Vector3D& pt1, const Vector3D& pt2) {
    return (pt1.x()*pt2.x() + pt1.y()*pt2.y() + pt1.z()*pt2.z());
}

const Vector3D operator*(float scale, const Vector3D& pt) {
    return Vector3D(scale*pt.x(),scale*pt.y(),scale*pt.z());
}

const Vector3D operator*(const Vector3D& pt, float scale) {
    return Vector3D(scale*pt.x(),scale*pt.y(),scale*pt.z());
}

bool operator==(const Vector3D& v1, const Vector3D& v2) {
    if ((v1.x() == v2.x()) && 
	(v1.y() == v2.y()) &&
	(v1.z ()== v2.z()))
	return true;
    else
	return false;
}


const Vector3D operator-(const Vector3D &p) {
  Vector3D q;
  q.x(-p.x());
  q.y(-p.y());
  q.z(-p.z());
  return q;
}

istream& operator>>(istream &is, Vector3D &in_vec) {
    float x, y, z;
    is >> x >> y >> z;
    in_vec.set(x, y, z);
    return is;
}


//--------------------------------
    
    float Length(const Vector3D &v) {
	return sqrt(v.x()*v.x() + v.y()*v.y() + v.z()*v.z());
    }
    
    
// set M to identity matrix
    void Identity3x3(float M[3][3])
    {
	int i, j;
	for(i = 0; i < 3; i++)
	    for(j = 0; j < 3; j++)
		M[i][j] = 0.;
	M[0][0] = M[1][1] = M[2][2] = 1.;
    }
    
    
// Rodrigues formula to convert from rotation vector to rotation matrix
    void Rodrigues(const Vector3D &r, float R[3][3])
    {
	float th = Length(r);
	if(th != 0)
	{
	    float x = r.x()/th, y = r.y()/th, z = r.z()/th;
	    float co = cos(th), si = sin(th);
	    R[0][0] = x*x*(1. - co) + co;
	    R[0][1] = x*y*(1. - co) - z*si;
	    R[0][2] = x*z*(1. - co) + y*si;
	    R[1][0] = x*y*(1. - co) + z*si;
	    R[1][1] = y*y*(1. - co) + co;
	    R[1][2] = y*z*(1. - co) - x*si;
	    R[2][0] = x*z*(1. - co) - y*si;
	    R[2][1] = y*z*(1. - co) + x*si;
	    R[2][2] = z*z*(1. - co) + co;
	}
	else
	{
	    Identity3x3(R);
	}
    }
    
    
    
    Vector3D Rotate(const Vector3D &r, const Vector3D &p) {
	float R[3][3];
	Vector3D q;
	Rodrigues(r, R);
	q.set( R[0][0]*p.x() + R[0][1]*p.y() + R[0][2]*p.z() ,
	       R[1][0]*p.x() + R[1][1]*p.y() + R[1][2]*p.z() ,
	       R[2][0]*p.x() + R[2][1]*p.y() + R[2][2]*p.z() );
	return q;
    }

    Vector3D Translate(Vector3D t, Vector3D p) {
	return t + p;
    }



}


