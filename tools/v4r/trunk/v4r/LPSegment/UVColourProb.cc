/**
 * $Id$
 */

#include "UVColourProb.hh"
#include "v4r/CEdge/Except.hh"

namespace P 
{

UVColourProb::UVColourProb()
{
}


/**
 * compute bayes colour probability
 */
bool UVColourProb::ComputeProb(UVHistogram &bg, UVHistogram &fg)
{
  float colProb;
  float prob,tmp;

  float numPixels = fg.Num()*PROB_FG_WEIGHT + bg.Num();

  if (numPixels==0)
    return false;

  float fgProb = (float)fg.Num()*PROB_FG_WEIGHT/(float)numPixels;

  for (unsigned i = 0; i < 64; i++)
  {
    for (unsigned j = 0; j < 64; j++)
    {
      colProb = ((float)(fg.bins[i][j]*PROB_FG_WEIGHT) + (float)bg.bins[i][j])/numPixels;

      if (colProb > 0.)
      {
        tmp = (fg.bins[i][j]*PROB_FG_WEIGHT)/(float)(fg.Num()*PROB_FG_WEIGHT);
        prob = tmp * fgProb / colProb;
        bins[i][j] = tmp*fgProb / colProb;
      }
      else
        bins[i][j] = 0;
    }
  }
  return true;
}


}

