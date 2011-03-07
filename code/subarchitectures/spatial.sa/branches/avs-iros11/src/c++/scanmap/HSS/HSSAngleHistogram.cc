//
// = FILENAME
//    HSSAngleHistogram.cc
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

#include "HSSAngleHistogram.hh"
#include "HSSutils.hh"

namespace HSS {

AngleHistogram::AngleHistogram()
  :m_RangeWeighted(false),
   m_NumAngleBins(180),
   m_DirCalcDA(2.0*M_PI/180),
   m_PeakBinWidth(25),
   m_MinPeakFrac(0.75)
{}

AngleHistogram::~AngleHistogram()
{}

bool
AngleHistogram::isCorridorLike(const HSS::Scan2D &scan, double *dir)
{
  std::vector<int> pts;  // vector with the indecis of points to use

  for (unsigned int i = 0; i < scan.range.size(); i++) {
    if (scan.valid[i]) pts.push_back(i);
  }

  findCorridorAngle(scan, pts);

  if (dir) *dir = m_PeakDir;

  bool ret = false;

  if (m_PeakFrac >= m_MinPeakFrac)
    ret = true;
  else 
    ret = false;

  if (ret) {
    std::cerr << "HSS::AngleHistogram::isCorridorLike CORR"
              << " m_PeakFrac=" << m_PeakFrac
              << " m_PeakDir=" << HSS::rad2deg(m_PeakDir) << "deg\n";
  } else {
    std::cerr << "HSS::AngleHistogram::isCorridorLike NON-corr"
              << " m_PeakFrac=" << m_PeakFrac
              << " m_PeakDir=" << HSS::rad2deg(m_PeakDir) << "deg\n";
  }

  return ret;
}

void
AngleHistogram::findCorridorAngle(const HSS::Scan2D &scan, 
                                  std::vector<int> &pts)
{
  m_Hist.clear();
  m_Hist.resize(m_NumAngleBins);

  double dtheta = (scan.max_theta - scan.min_theta) / ( scan.range.size() - 1);
  int width = int(m_DirCalcDA / dtheta + 0.5);

  // Build the angle histogram
  for (unsigned int i = 0; i < pts.size()-width; i++) {
    
    double a = atan2(scan.range[pts[i+width]] * sin(scan.theta[pts[i+width]]) - 
                     scan.range[pts[i]] * sin(scan.theta[pts[i]]),
                     scan.range[pts[i+width]] * cos(scan.theta[pts[i+width]]) - 
                     scan.range[pts[i]] * cos(scan.theta[pts[i]]));

    // Add the vote to the right bin in the histogram
    if (a < 0) a += M_PI;
    int bin = int(a / M_PI * m_NumAngleBins + 0.5);
    if (bin >= m_NumAngleBins) bin = 0;
    if (m_RangeWeighted) {
      m_Hist[bin] += scan.range[i+int(width/2)];
    } else {
      m_Hist[bin] += 1;
    }
  }

  // Now look for the peak value
  double maxV = 0;
  int maxI = -1;
  for (int i = 0; i < m_NumAngleBins; i++) {
    if (m_Hist[i] > maxV) {
      maxI = i;
      maxV = m_Hist[i];
    }
  }

  // Zero all bins that are small compared to the peak to get rid of
  // some of the noise
  double sum = 0;
  for (int i = 0; i < m_NumAngleBins; i++) {
    if (m_Hist[i] < 0.05*maxV) {
      m_Hist[i] = 0;
    } else {
      sum += m_Hist[i];
    }
  }
  
  // Calculate the fraction of weight around the peak value
  double peak = 0;
  for (int i = -m_PeakBinWidth; i <= m_PeakBinWidth; i++) {
    int index = (maxI + i + m_NumAngleBins) % m_NumAngleBins;
    peak += m_Hist[index];
  }
  
  //std::cerr << "peak=" << peak << " sum=" << sum << std::endl;
  m_PeakFrac = peak / sum;

  // Smooth the histogram before we look for the peak
  smoothHistogram();

  // Now look for the maximum direction
  maxV = 0;
  maxI = -1;
  std::cerr << "Hist for " << scan.m_Id << ": h=[";
  for (int i = 0; i < m_NumAngleBins; i++) {
    if (m_Hist[i] > maxV) {
      maxI = i;
      maxV = m_Hist[i];
    }
    std::cerr << m_Hist[i] << " ";
  }
  std::cerr << "];" << std::endl;

  m_PeakDir = M_PI * maxI / m_NumAngleBins;
}

void
AngleHistogram::smoothHistogram()
{
  std::vector<double> tmpHist;
  tmpHist.resize(m_NumAngleBins);
  for (int i = 0; i < m_NumAngleBins; i++) {
    tmpHist[i] = (m_Hist[(i-1+m_NumAngleBins)%m_NumAngleBins] + 
                  m_Hist[i] + 
                  m_Hist[(i+1)%m_NumAngleBins]) / 3;
  }

  for (int i = 0; i < m_NumAngleBins; i++) {
    m_Hist[i] = tmpHist[i];
  }
}

}; // namespace HSS
