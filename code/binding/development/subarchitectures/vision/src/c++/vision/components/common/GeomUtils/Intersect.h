/** @file Intersect.h
 *  @brief Useful functions for computing interaction of geometrical structures.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _INTERSECT_H_
#define _INTERSECT_H_

#include "Vector3D.h"
#include "Ray.h"
#include "Plane.h"

extern double intersect(Vector3D &X,Ray &ray, double &t);
extern double intersect(Ray &, double &,Ray &, double &);
extern double intersect(Ray &, double &,Plane &, double &,double &);



#endif
