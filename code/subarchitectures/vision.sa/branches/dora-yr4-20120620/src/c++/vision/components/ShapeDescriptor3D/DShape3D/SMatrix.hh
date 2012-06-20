/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SMATRIX_HH
#define P_SMATRIX_HH

#include "PNamespace.hh"
#include "Except.hh"

namespace P
{

extern void Mul33(double *m1, double *m2, double *r);
extern void Mul33(double *m, double s, double *r);
extern double Det33(double *m);
extern bool Inv33(double *m, double *r);
extern void Transpose33(double *m, double *r);

}

#include "SMatrix.ic"

#endif

