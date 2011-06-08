/**
 * @file ColorHistogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#include <string.h>
#include "Math.hh"
#include "ColorHistogram.h"

namespace Z
{

ColorHistogram::ColorHistogram(int _nr_bins, std::vector<cv::Vec4f> _p)
{
  nr_bins = _nr_bins;
  points = _p;
  nr_points = points.size();

  redHist = new double[nr_bins];
  greenHist = new double[nr_bins];
  blueHist = new double [nr_bins];
  
  for(unsigned i=0; i<nr_bins; i++)
    redHist[i] = greenHist[i] = blueHist[i] = 0.;

  Z::RGBValue color;
  double bin = 0.;
  for(unsigned i=0; i<points.size(); i++)
  {
    color.float_value = points[i][3];
    
    bin = color.r*(double)nr_bins/255.;
    redHist[(int)bin]+=1;
    bin = color.g*(double)nr_bins/255.;
    greenHist[(int)bin]+=1;
    bin = color.b*(double)nr_bins/255.;
    blueHist[(int)bin]+=1;
  }
  
  for(unsigned i=0; i<nr_bins; i++)
  {
    redHist[i] /= nr_points;
    greenHist[i] /= nr_points;
    blueHist[i] /= nr_points;
  }
// printf("\nColorHistogram with Vec:\n");
// printf("vec: v_points: %u\n", v_points.size());
//PrintHistogram();  /// TODO Print histogram
}


double ColorHistogram::Compare(ColorHistogram *ch)
{
  if(nr_bins != ch->nr_bins)
  {
    printf("ColorHistogram::Compare: Warning: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  
  double this_sum[3] = {0., 0., 0.};
  double ch_sum[3] = {0., 0., 0.};
  
//   for(unsigned col=0; col<3; col++)
//   {
    for(unsigned i=0; i<nr_bins; i++)
    {
      this_sum[0] += redHist[i]*i*1/nr_bins;
      ch_sum[0] += ch->redHist[i]*i*1/nr_bins;
      this_sum[1] += greenHist[i]*i*1/nr_bins;
      ch_sum[1] += ch->greenHist[i]*i*1/nr_bins;
      this_sum[2] += blueHist[i]*i*1/nr_bins;
      ch_sum[2] += ch->blueHist[i]*i*1/nr_bins;
    }
//   }

  double overall_sum = 0;
  for(unsigned col=0; col<3; col++)
  {
    overall_sum += fabs(this_sum[col] - ch_sum[col]);
// printf("  this_sum[%u]: %4.4f\n", col, this_sum[col]);
// printf("  ch_sum[%u]: %4.4f\n", col, ch_sum[col]);
  }
  overall_sum /= 3.;
  return overall_sum;
}

void ColorHistogram::PrintHistogram()
{
  printf("Print histogram:\n");
  for(unsigned i=0; i<nr_bins; i++)
    printf(" red[%u]: %4.3f\n", i, redHist[i]);
  for(unsigned i=0; i<nr_bins; i++)
    printf(" gre[%u]: %4.3f\n", i, greenHist[i]);
  for(unsigned i=0; i<nr_bins; i++)
    printf(" blu[%u]: %4.3f\n", i, blueHist[i]);
}
  
}

