/** @file Plane.cpp
 *  @brief A 3D plane.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Plane.h"

using namespace std;

Plane::Plane() {
  active=0;
}
  
Plane::Plane(Vector3D X0_,Vector3D X1_,Vector3D X2_) { 
  set(X0_,X1_,X2_); 
}

void Plane::set(Vector3D X0_, Vector3D X1_, Vector3D X2_) {
  X0 = X0_; X1 = X1_ - X0_; X2 = X2_ - X0_;
  active = 1;  
}
 

Plane& Plane::operator=(const Plane& src) {
  if( this == &src ) 
      return (*this);
  X0 = src.X0;
  X1 = src.X1;
  X2 = src.X2;
  return (*this);
}

 
const Vector3D& Plane::t(double t1,double t2) {
  static Vector3D returnpt;
  returnpt = X0 + t1*X1 + t2*X2;
  return returnpt;
}

void Plane::print(ostream *os) {
  *os << "X(t1,t2) = " << X0 << " + t1 " << X1 << " + t2 " << X2 << "  ";
}
      


ostream &operator<<(ostream &os, Plane& plane) {
  plane.print(&os);
  return os;
}
