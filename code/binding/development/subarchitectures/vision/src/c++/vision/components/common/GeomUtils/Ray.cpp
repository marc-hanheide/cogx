/** @file Ray.cpp
 *  @brief A 3D ray.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#include "Ray.h"
#include "GeomMath.h"  // for RTOD, DTOR

using namespace std;

Ray::Ray() {
  active=0;
}

Ray::Ray(Vector3D X0_,Vector3D X1_) { 
  set(X0_,X1_); 
}

void Ray::set(Vector3D X0_,Vector3D X1_) {
  X0 = X0_; 
  X1 = X1_ - X0_;
  active = 1;
}
  
Ray& Ray::operator=(const Ray& src) {
  if( this == &src ) 
      return (*this);
  X0 = src.get_X0();
  X1 = src.get_X1();
  return (*this);
}


const Vector3D& Ray::t(double t_) {
  static Vector3D returnpt;

  returnpt = X0 + t_*X1;
  return returnpt;
}



double Ray::computeDegAngle(Ray ray2) const {
  Vector3D dir1, dir2;

  dir1 = X1;
  dir2 = ray2.X1;

  double dot_prod = dir1 * dir2;
  double norm_prod = dir1.norm() * dir2.norm();

  double theta = acos(dot_prod/norm_prod); 

  theta = RTOD(theta);

  return(theta);
}


void Ray::print(ostream *os) {
  *os << "X(t) = " << X0 << " + t " << X1 << "  ";
}


ostream &operator<<(ostream &os, Ray& ray) {
  ray.print(&os);
  return os;
}
