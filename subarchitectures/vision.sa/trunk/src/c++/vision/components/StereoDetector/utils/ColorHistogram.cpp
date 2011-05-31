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

ColorHistogram::ColorHistogram(int _nr_bins, cv::Mat_<cv::Vec4f> _p)
{  
  nr_bins = _nr_bins;
  points = _p;
  nr_points = _p.cols*_p.rows;

  redHist = new double[nr_bins];
  greenHist = new double[nr_bins];
  blueHist = new double [nr_bins];
  
  for(unsigned i=0; i<nr_bins; i++)
  {
    redHist[i] = 0.;
    greenHist[i] = 0.;
    blueHist[i] = 0.;
  }
  

  Z::RGBValue color;
  double bin = 0.;
  for(unsigned i=0; i<_p.cols; i++)
  {
    color.float_value = _p(0, i)[3];
    
    bin = color.r*(double)nr_bins/255.;
    redHist[(int)bin]+=1;
    bin = color.g*(double)nr_bins/255.;
    greenHist[(int)bin]+=1;
    bin = color.b*(double)nr_bins/255.;
    blueHist[(int)bin]+=1;
  }
  
  // normalisation
  for(unsigned i=0; i<nr_bins; i++)
  {
    redHist[i] /= nr_points;
    greenHist[i] /= nr_points;
    blueHist[i] /= nr_points;
  }
  
  PrintHistogram();
printf("ColorHistogram::ColorHistogram: Create Histogram end!!!\n");
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

