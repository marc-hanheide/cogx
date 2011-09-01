//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SMATH_H
#define  _SMATH_H

#include "multiplatform.hpp"

#include <limits.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

NAMESPACE_CLASS_BEGIN( RTE )

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

extern unsigned BinaryToGray(unsigned b) __THROW_EXCEPT;
extern unsigned HammingDistance(unsigned a, unsigned b);

extern bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2);

extern int CityblockDistance(int x1, int y1, int x2, int y2);

/*extern int timeval_subtract(struct timeval *result, struct timeval *x,
struct timeval *y);*/
extern double timespec_diff(struct timespec *x, struct timespec *y);

extern double SphereVolume(double r);
extern double CircleArea(double r);

NAMESPACE_CLASS_END()

#if defined WIN32 || defined WIN64
   #if defined ELLDETECTEXE
      #include "SMath.ic"
   #else
      #include "../C/SMath.ic"
   #endif
#else
   #include "SMath.ic"
#endif
#endif

