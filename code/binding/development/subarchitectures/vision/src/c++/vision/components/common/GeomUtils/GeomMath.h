/** @file GeomMath.h
 *  @brief Useful data structures and functions for geometrical manipulation.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _GEOM_MATH_H_
#define _GEOM_MATH_H_

#include <cmath>

#ifndef _AFFINE_COMPENSATION_H_
typedef struct affine {
  double S1;
  double S2;
  double S3;
  double S4;
  double Tx;
  double Ty;
} affine;
#endif

#define DTOR(x)		((x)*M_PI/180.0)
#define RTOD(x)		((x)*180/M_PI)

template<class T> T sqr(T x) {return x*x;}

#endif
