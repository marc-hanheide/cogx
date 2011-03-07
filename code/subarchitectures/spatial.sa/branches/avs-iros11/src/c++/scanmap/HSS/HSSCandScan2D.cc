//
// = FILENAME
//    HSSCandScan2D.cc
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

#include "HSSCandScan2D.hh"
#include "HSSAngleHistogram.hh"

namespace HSS {

void makeScanCandidate(HSS::CandScan2D &scan, const Eigen::Vector3d &xsR)
{
  scan.m_Corridor = false;
  scan.m_C.setZero();

  if (scan.range.empty()) return;

  ////////////////////////////////
  // Check if the scan is corridor like
  HSS::AngleHistogram ah;
  ah.setMinPeakFrac(0.7);

  scan.m_Corridor = ah.isCorridorLike(scan, &scan.m_MainDirS);
  
  std::cerr << "Scan->ScanCand: " << scan.m_Id << " " << scan.m_Corridor << std::endl;
    
  // Adjust the corridor direction to be in the robot frame
  // instead of sensor frame
  scan.m_MainDirR = HSS::pi_to_pi(scan.m_MainDirS + xsR[2]);
  
  // We do not want the direction of the corridor to result in any
  // changes of direction of motion so we make sure that it is
  // +-pi/2
  if (scan.m_MainDirS < -M_PI_2) scan.m_MainDirS += M_PI;
  else if (scan.m_MainDirS > M_PI_2) scan.m_MainDirS -= M_PI;


  ///////////////////////////////
  // Calculate the center of mass and a box approximation so that we
  // can check for scan overlap
  double xSum  = 0, ySum = 0;
  std::vector<double> x,y;
  x.resize(scan.range.size());
  y.resize(scan.range.size());
  int n = 0;
  for (unsigned int i = 0; i < scan.range.size(); i++) {
    if (!scan.valid[i]) continue;
    x[n] = scan.range[i] * cos(scan.theta[i] - scan.m_MainDirS);
    y[n] = scan.range[i] * sin(scan.theta[i] - scan.m_MainDirS);
    xSum += x[n];
    ySum += y[n];
    n++;
  }
  if (n > 0) {
    scan.m_C[0] = xSum / n;
    scan.m_C[1] = ySum / n;

    // Build histograms in the x- and y-directions and get a bounding
    // box that contains X% of all the points in the x- and
    // y-directions.
    const double minX = -10, maxX = 10, minY = -10, maxY = 10;
    const double step = 0.1;
    const double nX = int((maxX - minX) / step + 0.5) + 1;
    const double nY = int((maxY - minY) / step + 0.5) + 1;

    std::vector<int> histX, histY;
    histX.resize(nX);
    histY.resize(nY);
    for (int i = 0; i < nX; i++) histX[i] = 0;
    for (int i = 0; i < nY; i++) histY[i] = 0;
    for (int i = 0; i < n; i++) {
      if (x[i] >= minX && x[i] <= maxX) {
        histX[int((x[i] - minX) / step + 0.5)]++;        
      }
      if (y[i] >= minY && y[i] <= maxY) {
        histY[int((y[i] - minY) / step + 0.5)]++;        
      }
    }

    int m = 0;

    double bb[4];

    const double limit = 0.9;

    // Find the lower limit in X
    m = 0;
    for (int i = nX-1; i >= 0; i--) {
      m += histX[i];
      if (1.0 * m / n > limit) {
        bb[0] = minX + step * i;
        break;
      }
    } 
    if (1.0 * m / n < limit) bb[0] = minX;
    // Find the upper limit in X
    m = 0;
    for (int i = 0; i < nX; i++) {
      m += histX[i];
      if (1.0 * m / n >= limit) {
        bb[1] = minX + step * i;
        break;
      }
    } 
    if (1.0 * m / n < limit) bb[1] = maxX;
    // Find the lower limit in Y
    m = 0;
    for (int i = nY-1; i >= 0; i--) {
      m += histY[i];
      if (1.0 * m / n > limit) {
        bb[2] = minY + step * i;
        break;
      }
    } 
    if (1.0 * m / n < limit) bb[2] = minY;
    // Find the upper limit in Y
    m = 0;
    for (int i = 0; i < nY; i++) {
      m += histY[i];
      if (1.0 * m / n > limit) {
        bb[3] = minY + step * i;
        break;
      }
    } 
    if (1.0 * m / n < limit) bb[3] = maxY;

    bb[0] -= 0.1;
    bb[1] += 0.1;
    bb[2] -= 0.1;
    bb[3] += 0.1;

    scan.m_BB.resize(2,4);
    scan.m_BB(0,0) = bb[0];
    scan.m_BB(1,0) = bb[2];
    scan.m_BB(0,1) = bb[1];
    scan.m_BB(1,1) = bb[2];
    scan.m_BB(0,2) = bb[1];
    scan.m_BB(1,2) = bb[3];
    scan.m_BB(0,3) = bb[0];
    scan.m_BB(1,3) = bb[3];
    scan.m_BBdir = scan.m_MainDirS;
    scan.m_BBw = bb[1] - bb[0];
    scan.m_BBh = bb[3] - bb[2];

    // Rotate the BB cordinates back to the scanner reference frame
    Eigen::MatrixXd R(2,2);
    R(0,0) = cos(scan.m_MainDirS);
    R(0,1) =-sin(scan.m_MainDirS);
    R(1,0) =-R(0,1);
    R(1,1) = R(0,0);
    scan.m_BB = R * scan.m_BB;

    //std::cerr<<"BB: xC=" << scan.m_C[0] << " yC=" << scan.m_C[1] << std::endl;
  }
}

}; // namespace HSS
