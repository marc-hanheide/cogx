/**
 * $Id$
 * Johann Prankl, 20091118
 */

#include "RGBColourProb.hh"
#include "v4r/CEdge/Except.hh"

namespace P 
{

RGBColourProb::RGBColourProb()
{
}


/**
 * compute bayes colour probability
 */
bool RGBColourProb::ComputeProb(RGBHistogram &bg, RGBHistogram &fg)
{
  float colProb;
  float prob,tmp;

  float numPixels = fg.Num()*PROB_FG_WEIGHT + bg.Num();

  if (numPixels==0)
    return false;

  float fgProb = (float)fg.Num()*PROB_FG_WEIGHT/(float)numPixels;

  for (unsigned i = 0; i < 16; i++)
  {
    for (unsigned j = 0; j < 16; j++)
    {
      for (unsigned k = 0; k < 16; k++)
      {
        colProb = ((float)(fg.bins[i][j][k]*PROB_FG_WEIGHT) + (float)bg.bins[i][j][k])/numPixels;

        if (colProb > 0.)
        {

          tmp = (fg.bins[i][j][k]*PROB_FG_WEIGHT)/(float)(fg.Num()*PROB_FG_WEIGHT);
          prob = tmp * fgProb / colProb;
          bins[i][j][k] = tmp*fgProb / colProb;
        }
        else
          bins[i][j][k] = 0;
      }
    }
  }
  return true;
}


}

