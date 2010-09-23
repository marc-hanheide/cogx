/**
 * $Id: Math.hh,v 1.23 2007/03/25 21:35:57 mxz Exp mxz $
 *
 * Michael Zillich, 2004-3-04
 *
 * TODO: use enums for LEFT, RIGHT, START ...
 */

#ifndef Z_MATH_HH
#define Z_MATH_HH

#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <stdexcept>
// #include "Namespace.hh"

namespace Z
{

const unsigned UNDEF_ID = UINT_MAX;

const int START     = 0;
const int END       = 1;
const int MID       = 2;
const int ONE_THIRD = 3;
const int TWO_THIRD = 4;
const int STOP      = 15;

inline int OtherEnd(int end)
{
  return end ^ 0x1;
}

// same and opposite sense
const int SAME = START;
const int OPP  = END;

const int LEFT  = 0;
const int RIGHT = 1;

// inner and outer ends of junctions
const int INNER = 0;
const int OUTER = 1;

inline int OtherSide(int side)
{
  return side ^ 0x1;
}

inline int Other(int i)
{
  return i ^ 0x1;
}

extern void InitMath();
extern void ExitMath();
 
extern int RandInt();

/// Returns true if the value is near zero (+/- epsilon)
extern bool IsZero(double d);

/// Returns true if the values are equal (+/- epsilon)
extern bool IsEqual(double a, double b);

/// Return whether value is a 'reasonable' number
extern bool IsReasonable(double d);

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

extern unsigned BinaryToGray(unsigned b) throw (std::runtime_error);
extern unsigned HammingDistance(unsigned a, unsigned b);

extern bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2);

extern int CityblockDistance(int x1, int y1, int x2, int y2);

extern double Fact(int n);
extern double LogFact(int n);
extern double LogBinCoef(int n, int k) throw (std::runtime_error);
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
extern double BivariateGaussianPDF(double mx, double my, double s, double x, double y);

extern int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y);
extern double timespec_diff(struct timespec *x, struct timespec *y);

extern double SphereVolume(double r);
extern double CircleArea(double r);

// Returns a pseudo random number in [0.0, 1.0]
static float FRand();
// Exponential PDF
static float ExpPdf(float lambda);
// Return an int <= max with probability according to expnential PDF
static int ExpSelect(int max);

}

#include "Math.ic"

#endif

