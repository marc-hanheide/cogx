/** @file Vector3D.h
 *  @brief A 3D vector.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#ifndef VECTOR_3D_H
#define VECTOR_3D_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include "Vector2D.h"

namespace Geom {

    using namespace std;

    class Vector3D {
	
    private:
	float m_x;
	float m_y;
	float m_z;
	
	
    public:
	Vector3D();
	Vector3D(float x_,float y_, float z_);
	Vector3D(const Vector3D &src);
	~Vector3D();
	
	Vector3D& operator=(const Vector3D& src);

	void setZero() {set(0.0, 0.0,0.0);};
	void set(float x_,float y_, float z_);
	float x() const {return m_x;};
	float y() const {return m_y;};
	float z() const {return m_z;};
	void x(float x_) {m_x=x_;};
	void y(float y_) {m_y=y_;};
	void z(float z_) {m_z=z_;};

	float distance(const Vector3D& otherpt) const;
	float norm() const;
	
	// orthographic project on z=0 
	const Vector2D orthoz();
	
	void print(std::ostream *os) const;
	
	friend bool operator==(const Vector3D&, const Vector3D&); 
	friend const Vector3D  operator+(const Vector3D&, const Vector3D&);
	friend const Vector3D  operator-(const Vector3D&, const Vector3D&);
	friend float operator*(const Vector3D&, const Vector3D&);
	friend const Vector3D  operator*(float, const Vector3D&);
	friend const Vector3D  operator*(const Vector3D&, float);
	friend const Vector3D operator-(const Vector3D &p);
	
	friend istream& operator>>(istream &is, Vector3D &in_vec);  
	
    };


    float Length(const Vector3D &v);
    void Rodrigues(const Vector3D &r, float R[3][3]);
    Vector3D Rotate(const Vector3D &r, const Vector3D &p);
    Vector3D Translate(const Vector3D &t, const Vector3D p);
    void Identity3x3(float M[3][3]);

    inline std::ostream& operator<<(std::ostream &os, const Vector3D &pt) {
	pt.print(&os);
	return os;
    }

}

#endif
