/**
 * $Id$
 */

#include "RGBHistogram.hh"
#include "v4r/CEdge/Except.hh"

namespace P 
{

RGBHistogram::RGBHistogram()
 : sum(0)
{
  for(unsigned i = 0; i < 16; i++)
    for(unsigned j = 0; j < 16; j++)
      for(unsigned k = 0; k < 16; k++)
        bins[i][j][k] = 0;
}

/**
 * Normalise the histogramm to v
 */
void RGBHistogram::Normalise(unsigned v)
{
 
  double n=(double)v/(double)sum;
  for(unsigned i = 0; i < 16; i++)
    for(unsigned j = 0; j < 16; j++)
      for(unsigned k = 0; k < 16; k++)
        bins[i][j][k] = (unsigned short)((double)bins[i][j][k]*n + 0.5);
  sum = v;
}

}

