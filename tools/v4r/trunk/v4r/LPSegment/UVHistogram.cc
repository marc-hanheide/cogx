/**
 * $Id$
 */

#include "UVHistogram.hh"
#include "v4r/CEdge/Except.hh"

namespace P 
{

UVHistogram::UVHistogram()
 : sum(0)
{
  for(unsigned i = 0; i < 64; i++)
    for(unsigned j = 0; j < 64; j++)
      bins[i][j] = 0;
}

/**
 * Normalise the histogramm to v
 */
void UVHistogram::Normalise(unsigned v)
{
 
  double n=(double)v/(double)sum;
  for(unsigned i = 0; i < 64; i++)
    for(unsigned j = 0; j < 64; j++)
        bins[i][j] = (unsigned short)((double)bins[i][j]*n + 0.5);
  sum = v;
}

}

