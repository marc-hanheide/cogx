/**
 * @file ColorHistogram.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#ifndef Z_COLOR_HISTOGRAM_HH
#define Z_COLOR_HISTOGRAM_HH

#include <cv.h>
#include "Color.hh"

namespace Z
{


/**
 * @brief Class ColorHistogram
 */
class ColorHistogram
{
private:
  int nr_points;
  int nr_bins;
  cv::Mat_<cv::Vec4f> points;
  
  double *redHist, *greenHist, *blueHist;
  
public:
  ColorHistogram(int _nr_bins, cv::Mat_<cv::Vec4f> _p);
  
  void PrintHistogram();

};

}

#endif

