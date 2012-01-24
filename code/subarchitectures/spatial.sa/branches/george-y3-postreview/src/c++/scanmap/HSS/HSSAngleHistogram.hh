//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2007 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSAngleHistogram_hh
#define HSSAngleHistogram_hh

#include "HSSutils.hh"
#include "HSSScan2D.hh"

#ifndef DEPEND
#include <vector>
#endif

namespace HSS {

class AngleHistogram {
public:
  /**
   * Constructor
   */
  AngleHistogram();

  /**
   * Destructor
   */
  ~AngleHistogram();

  void setUseRangeWeighted(bool v) { m_RangeWeighted = v; }
  void setMinPeakFrac(double f) { m_MinPeakFrac = f; }
  void setZeroThreshold(double t) { m_ZeroThreshold = t; }
  void setPeakBinWidth(int w) { m_PeakBinWidth = w; }
  
  /**
   * Call this function to check if a scan is corridor like
   *
   * The direction of the corridor is defined in the coordinate system
   * of the sensor in the range [0,pi]
   *
   * @param scan the scan with the data to extract the corridor info from
   * @param dir return value, the direction of the corridor 
   *
   * @return true if the scan is from a corridor-like environment
   */
  bool isCorridorLike(const HSS::Scan2D &scan, double *dir = 0);

protected:
  /**
   * Builds up an angle histogram for the scan but looking at the
   * direction between points close by. This says something about the
   * main direction of wall (if there are any).
   */
  void findCorridorAngle(const HSS::Scan2D &scan, std::vector<int> &pts);

  void smoothHistogram();

protected:

  // Is true if we want to use a range weighted histogram
  bool m_RangeWeighted;

  // Number of angle bins
  int m_NumAngleBins;

  // Opening angle between points to calculate the direction of the
  // line
  double m_DirCalcDA;

  // The angle histgram
  std::vector<double> m_Hist;

  // The direction of the corridor is defined in the coordinate system
  // of the sensor in the range [0,pi]
  double m_PeakDir;

  // We remove noise from the histogram by removing datapoints that
  // are lower than a certain fraction of the peak value
  double m_ZeroThreshold;

  // When calculating the fraction of weight around the peak we include
  // points in bins this far to the left and right
  int m_PeakBinWidth;

  // The fraction of weight around the peak 
  double m_PeakFrac;

  // Minimum fraction of the points that have to be in the peak
  double m_MinPeakFrac;

};

}; // namespace HSS

#endif // HSSAngleHistogram_hh
