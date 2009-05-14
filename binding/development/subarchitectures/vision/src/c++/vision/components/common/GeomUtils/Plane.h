/** @file Plane.h
 *  @brief A 3D plane class definition.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef PLANE_H
#define PLANE_H

#include <iostream>
#include <iomanip>
#include "Vector3D.h"

using namespace Geom;

class Plane {
 public:
    Plane();
    Plane(Vector3D X0_,Vector3D X1_,Vector3D X2_);
    
    const Vector3D& get_X0() const {return X0;};
    const Vector3D& get_X1() const {return X1;};
    const Vector3D& get_X2() const {return X2;};
    void set(Vector3D X0_,Vector3D X1_,Vector3D X2_);
    
    Plane& operator=(const Plane& src);
    const Vector3D& t(double t1,double t2);

    void print(std::ostream *os);
    friend std::ostream& operator<<(std::ostream &os, Plane& plane);
    
 private:
    int active; 
    Vector3D X0;
    Vector3D X1;
    Vector3D X2;
};

#endif
