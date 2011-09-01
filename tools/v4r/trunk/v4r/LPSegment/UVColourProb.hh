/**
 * $Id$
 */


#ifndef P_UV_COLOUR_PROB_HH
#define P_UV_COLOUR_PROB_HH

#include <opencv/cv.h>
#include <iostream>
#include "v4r/CEdge/PNamespace.hh"
#include "v4r/PMath/PMath.hh"
#include "UVHistogram.hh"
#include "LPDefinitions.hh"

namespace P 
{
 
/**
 * A three channel color histogram.
 */
class UVColourProb
{
public:
  float bins[64][64];

  UVColourProb();
  ~UVColourProb(){};
  bool ComputeProb(UVHistogram &bg, UVHistogram &fg);
  inline float Prob(unsigned char u, unsigned char v);
};

/****************************** INLINE METHODES ********************************/
inline float UVColourProb::Prob(unsigned char u, unsigned char v)
{
  u >>= 2;
  v >>= 2;
  return bins[u][v];
}



}

#endif

