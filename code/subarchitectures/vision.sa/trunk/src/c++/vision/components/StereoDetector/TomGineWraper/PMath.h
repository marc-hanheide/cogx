/**
 * @file PMath.h
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Math functions for the wraper.
 */

#ifndef P_MATH_H
#define P_MATH_H

namespace P
{
  
bool IsZero(double d);
  
template <class Num>
extern Num Sqr(Num x);

}

#include "PMath.ic"

#endif