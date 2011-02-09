/**
 * $Id: Math.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 *
 * Michael Zillich, 2004-3-04
 *
 * TODO: use enums for LEFT, RIGHT, START ...
 */

#ifndef P_MATH_HH
#define P_MATH_HH

#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include "PNamespace.hh"
#include "Except.hh"

namespace P
{

const unsigned UNDEF_ID = UINT_MAX;

const unsigned START     = 0;
const unsigned END       = 1;
const unsigned MID       = 2;
const unsigned ONE_THIRD = 3;
const unsigned TWO_THIRD = 4;
const unsigned STOP      = 15;

inline unsigned OtherEnd(unsigned end)
{
  return end ^ 0x1;
}

// same and opposite sense
const unsigned SAME = START;
const unsigned OPP  = END;

const unsigned LEFT  = 0;
const unsigned RIGHT = 1;
const unsigned CAM3D = 2;

// inner and outer ends of junctions
const unsigned INNER = 0;
const unsigned OUTER = 1;

inline unsigned OtherSide(unsigned side)
{
  return side ^ 0x1;
}

inline unsigned Other(unsigned i)
{
  return i ^ 0x1;
}

extern void InitMath();
extern void ExitMath();
 
extern int RandInt();

extern float FRand();
extern float ExpDev(float lambda);
extern int ExpSelect(int max);

/// Returns true if the value is near zero (+/- epsilon)
extern bool IsZero(double d);

/// Returns true if the values are equal (+/- epsilon)
extern bool IsEqual(double a, double b);

/// Return whether value is a 'reasonable' number
extern bool IsReasonable(double d);

extern int SRound(double d);

/// Square of given number
template <class Num>
extern Num Sqr(Num x);

template <class Num>
extern Num Max(Num a, Num b);
template <class Num>
extern Num Min(Num a, Num b);

template <class Num>
extern int Sign(Num x);

template <class T>
inline void Swap(T &a, T &b);

template <class Num>
inline bool Between(Num x, Num l, Num u);
template <class Num>
inline bool BetweenEq(Num x, Num l, Num u);

extern double AngularResolution();
/// Scale angle to [0..2pi[
extern double ScaleAngle_0_2pi(double a);
/// Scale angle to [-pi..pi[
extern double ScaleAngle_mpi_pi(double a);
/// Scale angle to [0..pi[
extern double ScaleAngle_0_pi(double a);
/// Difference of two angles b - a. The result is scaled to [-pi..pi[
extern double DiffAngle_mpi_pi(double b, double a);
/// Difference of two angles b - a. The result is scaled to [0..2pi[
extern double DiffAngle_0_2pi(double b, double a);
/// Angles between (undirected) lines. The result is scaled to [0..pi/2[
extern double AngleBetweenLines(double b, double a);
/// Scale an integer angle to [0..8[
extern int ScaleIntAngle_0_8(int a);

extern unsigned BinaryToGray(unsigned b) throw(Except);
extern unsigned HammingDistance(unsigned a, unsigned b);

extern bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2);
extern bool ClipLine(int xmin, int ymin, int xmax, int ymax, double *x1, double *y1, double *x2, double *y2);

extern int CityblockDistance(int x1, int y1, int x2, int y2);

extern double Fact(int n);
extern double LogFact(int n);
extern double LogBinCoef(int n, int k) throw(Except);
extern double LogBinDist(int l, int k, double p);
extern double LogBinomialPDF(int l, int k, double p);
extern double BinomialPDF(int l, int k, double p);
extern double LogBinomialCDF(int l, int k, double p);
extern double BinomialCDF(int l, int k, double p);
extern double LogBinomialCDF_tail(int l, int k, double p);
extern double BinomialCDF_tail(int l, int k, double p);
extern double Significance(int m, int k, int l, double p);
extern double PoissonPDF(int k, double alpha);
extern double PoissonCDF(int k, double alpha);
extern double BivariateGaussianPDF(double mx, double my, double s,
    double x, double y);

extern int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y);
extern double timespec_diff(struct timespec *x, struct timespec *y);

extern double SphereVolume(double r);
extern double CircleArea(double r);

extern float DistSqr(float *desc1, float *desc2, unsigned size);

}

#include "Math.ic"

#endif

