/** @file Ray.h
 *  @brief A 3D ray.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#ifndef RAY_H
#define RAY_H

#include <iostream>
#include <iomanip>

#include "Vector3D.h"

using namespace Geom;

class Ray {
 private:
  int active; 

  Vector3D X0;
  Vector3D X1;

 public:  
  Ray();
  Ray(Vector3D X0_,Vector3D X1_);
  
  const Vector3D& get_X0() const {return X0;} ;
  const Vector3D& get_X1() const {return X1;} ;
 
  void set(Vector3D X0_,Vector3D X1_);
  Ray& operator=(const Ray& src) ;
  const Vector3D& t(double t_);

  double computeDegAngle(Ray ray2) const;
 
  void print(std::ostream *os);    
  friend std::ostream& operator<<(std::ostream& os, Ray& ray);
};
#endif
