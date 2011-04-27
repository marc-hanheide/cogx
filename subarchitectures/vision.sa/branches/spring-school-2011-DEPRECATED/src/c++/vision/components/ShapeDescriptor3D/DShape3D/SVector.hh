/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SVECTOR_HH
#define P_SVECTOR_HH

#include "PNamespace.hh"
#include "Except.hh"

namespace P
{

extern double Norm3(double v[3]);
extern void Normalise3(double v[3]);
extern void Mul3(double v[3], double s, double r[3]);
extern void Div3(double v[3], double s, double r[3]);
extern void Sub3(double v1[3], double v2[3], double r[3]);
extern void Add3(double v1[3], double v2[3], double r[3]);
extern double Dot3(double v1[3], double v2[3]);

}

#include "SVector.ic"

#endif

