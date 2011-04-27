/**
 * @file SMatrix.hh
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Matrix class as inline implementation.
 */


#ifndef TGTHREAD_SMATRIX_HH
#define TGTHREAD_SMATRIX_HH

#include "SVector.hh"

namespace TGThread
{
extern void Mul22(double *m, double s, double *r);

extern void Mul33(double *m1, double *m2, double *r);
extern void Mul33(double *m, double s, double *r);
extern double Det33(double *m);
extern bool Inv33(double *m, double *r);
extern void Transpose33(double *m, double *r);

extern void Mul3(double R[9], double v[3], double r[3]);
extern void MulAdd3( double R[9], double v[3], double t[3], double r[3]);

}

#include "SMatrix.ic"

#endif

