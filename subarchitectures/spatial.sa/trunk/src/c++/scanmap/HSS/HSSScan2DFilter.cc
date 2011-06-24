//
// = FILENAME
//    HSSScan2DFilter.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSScan2DFilter.hh"

namespace HSS {

void 
Scan2DFilter::filterScanRange(Scan2D &scan)
{
  for (unsigned int i = 0; i < scan.range.size(); i++) {

    // do not filter reads that are outliers
    if (scan.range[i] > scan.max_range ||
        scan.range[i] < scan.min_range) {

      scan.valid[i] = 0;
    }

  }
}

void 
Scan2DFilter::filterScanMean(Scan2D &scan, int width, double maxRangeDiff)
{
  std::vector<double> tmprange;
  tmprange.resize(scan.range.size());
  for (unsigned int i = 0; i < scan.range.size(); i++) {
    tmprange[i] = scan.range[i];
  }

  for (unsigned int i = 0; i < tmprange.size(); i++) {

    // do not filter reads that are outliers
    if (!scan.valid[i]) continue;

    double sum = tmprange[i];
    int n = 1;
    
    for (int j = -width; j <= width; j++) {
      
      if (j == 0) continue;
      
      if (i+j >= 0 &&
          i+j < tmprange.size() &&
          scan.valid[i+j] &&
          fabs(tmprange[i] - tmprange[i+j]) < maxRangeDiff*fabs(j)) {
        sum += tmprange[i+j];
        n++;
      }
    }

    scan.range[i] = sum / n;
  }
}

void 
Scan2DFilter::filterScanSmall(Scan2D &scan, int minlen, double maxRangeDiff)
{
  int start = 0;
  int end = 0;
  int len = 1;
  for (unsigned int i = 1; i < scan.range.size(); i++) {

    if (!scan.valid[i]) {
      end++;
      continue;
    } 

    if (fabs(scan.range[i] - scan.range[end]) < maxRangeDiff*(i-end)) {
      end++;
      len++;
      continue;
    }

    if (len < minlen) {
      for (int j = start; j<=end; j++) {
        scan.valid[j] = 0;
      }      
    }

    start = end = i;
    len = 1;
  }
}

void 
Scan2DFilter::filterScanMotion(HSS::Scan2D &scan, 
                                 std::vector<ScanMotionDetector::Movement> &m,
                                 int width)
{
  for (unsigned int i = 0; i < m.size(); i++) {
    for (int j = 0; j < m[i].nPts; j++) {
      for (int k = -width; k <= width; k++) {
        int l = m[i].scanIndex[j] + k;
        if (l >= 0 && l < int(scan.range.size())) {
          scan.valid[l] = 0;
        }
      }
    }
  }
};

void 
Scan2DFilter::filterScanMotion(HSS::Scan2D &scan, 
                                 ScanMotionDetector &motionDetector,
                                 const Eigen::VectorXd &X, 
                                 const Eigen::Vector3d &xsR,
                                 int width)
{
  double r[scan.range.size()];
  for (unsigned int i = 0; i < scan.range.size(); i++) {
    r[i] = scan.range[i];
  }
  double startAngle = scan.min_theta;
  double angleStep = (scan.max_theta - scan.min_theta) / (scan.range.size()-1);

  double t = 1.0*scan.tv.tv_sec + 1e-6*scan.tv.tv_usec;

  if (motionDetector.checkForMotion(t, scan.range.size(), r, 
                                    startAngle, angleStep,
                                    xsR[0], xsR[1], xsR[2],
                                    X[0], X[1], X[2])) {

    std::vector<HSS::ScanMotionDetector::Movement> m = 
      motionDetector.getMovments();

    filterScanMotion(scan, m, width);
  }
}

}; // namespace HSS
