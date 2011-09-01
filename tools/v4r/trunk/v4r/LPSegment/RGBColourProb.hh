/**
 * $Id$
 */


#ifndef P_RGB_COLOUR_PROB_HH
#define P_RGB_COLOUR_PROB_HH

#include <opencv2/core/core.hpp>
#include <iostream>
#include "v4r/CEdge/PNamespace.hh"
#include "v4r/PMath/PMath.hh"
#include "RGBHistogram.hh"
#include "LPDefinitions.hh"

namespace P 
{
 
/**
 * A three channel color histogram.
 */
class RGBColourProb
{
public:
  float bins[16][16][16];

  RGBColourProb();
  ~RGBColourProb(){};
  bool ComputeProb(RGBHistogram &bg, RGBHistogram &fg);
  inline float Prob(unsigned char r, unsigned char g, unsigned char b);
};

/****************************** INLINE METHODES ********************************/
inline float RGBColourProb::Prob(unsigned char r, unsigned char g, unsigned char b)
{
  r >>= 4;
  g >>= 4;
  b >>= 4;
  return bins[r][g][b];
}



}

#endif

