/**
 * @file SVector.hh
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Vector class as inline implementation.
 */

#ifndef TGTHREAD_SVECTOR_HH
#define TGTHREAD_SVECTOR_HH

#include "PMath.h"

namespace TGThread
{
extern void Rotate2(double v[2], double phi, double r[2]);
extern double Norm3(double v[3]);
extern void Normalise3(double v[3]);
extern void Mul3(double v[3], double s, double r[3]);
extern void Div3(double v[3], double s, double r[3]);
extern void Sub3(double v1[3], double v2[3], double r[3]);
extern void Add3(double v1[3], double v2[3], double r[3]);
extern double Dot3(double v1[3], double v2[3]);
extern double Distance3(double d1[3], double d2[3]);
extern double DistanceSqr3(double d1[3], double d2[3]);
}

#include "SVector.ic"

#endif

