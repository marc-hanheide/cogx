//
// = FILENAME
//    HSSScanMotionDetector.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2004 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSScanMotionDetector.hh"
#include "HSSutils.hh"

#ifndef DEPEND
#include <cstring>  // memcpy
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#endif

namespace HSS {

ScanMotionDetector::ScanMotionDetector(bool useAlgorithmBackward,
                                             int maxNScans,
                                             int backwardRef)
{
  // Parameters
  if (maxNScans < 0) {
    m_MaxNScans = 10;
  } else {
    m_MaxNScans = maxNScans;
  }
  m_MaxDetectableRange = 5.5;
  m_MinViolationPts = 2;
  m_MaxDepthDiff = 0.5;
  if (backwardRef < 0) {
    m_BackwardRef = m_MaxNScans / 2;
  } else {
    m_BackwardRef = backwardRef;
  }
  m_ViolationPtsPerM = 0.3;
  m_SideSkip = 15;
  m_UseAlgorithmBackward = useAlgorithmBackward;

  if (m_BackwardRef >= m_MaxNScans) {
    std::cerr << "ScanMotionDetector WARNING: "
              << "Cannot have back ref larger than " << maxNScans << "\n";
    m_BackwardRef = m_MaxNScans - 1;
  }
  
  std::cerr << "ScanMotionDetector using a Buffer with " << m_MaxNScans
               << " scans for motion detection\n";

  m_FreeSpace = new FreeSpaceScan[m_MaxNScans];
  m_NScans = 0;
  m_LastScan = -1;

  m_TrigLookupRes = HSS::deg2rad(0.05);
  m_TrigLookupSize = int(2.0 * M_PI / m_TrigLookupRes + 0.5);
  m_CosLookup = new double[m_TrigLookupSize];
  m_SinLookup = new double[m_TrigLookupSize];
  for (int i = 0; i < m_TrigLookupSize; i++) {
    m_CosLookup[i] = cos(m_TrigLookupRes * i);
    m_SinLookup[i] = sin(m_TrigLookupRes * i);
  }

  m_Movements.clear();

  useNonhumanRemoval(true);
  setNonhumanParams(); // No params gives the default values
}

ScanMotionDetector::~ScanMotionDetector()
{
  delete [] m_FreeSpace;
  delete [] m_CosLookup;
  delete [] m_SinLookup;
}

void
ScanMotionDetector::reset()
{
  std::cerr << "Resetting ScanMotionDetector\n";
  m_Movements.clear();
  m_NScans = 0;
  m_LastScan = -1;
}

bool 
ScanMotionDetector::checkForMotion(double t,
                                      int n, double *r, 
                                      double startAngle, double angleStep,
                                      double xlR, double ylR, double alR,
                                      double xrF, double yrF, double arF)
{
  double xlF, ylF, alF;
  xlF = xrF + xlR * cos(arF) - ylR * sin(arF);
  ylF = yrF + xlR * sin(arF) + ylR * cos(arF);
  alF = arF + alR;

  renewFreeSpaceModel(t, n, r, startAngle, angleStep, 
                      xlF, ylF, alF);

  // We do not check for motion until the buffer of reference scans
  // are full
  if (m_NScans == m_MaxNScans) {

    // Forget all previous detected motion
    m_Movements.clear();

    // Compare last scan with oldest reference scan 
    checkForMotion((m_LastScan + 1) % m_NScans, m_LastScan);

    // Use the last scan as a reference scan and check for motion
    // using an old scan
    int nForward = m_Movements.size();
    if (m_UseAlgorithmBackward) {
      checkForMotion(m_LastScan, 
                     (m_LastScan - m_BackwardRef + m_NScans) % m_NScans, 
                     true);
    }

    // Go through the motions that were found when running the
    // algorithm backward and combining it with forward prediction. If
    // any of these movements are close to any of the forward ones we
    // skip it
    std::vector<ScanMotionDetector::Movement>::iterator iter = m_Movements.begin();
    for (int j = 0; j < nForward; j++) {
      iter++;
    }

    for (; iter != m_Movements.end();) {
      // Compare again the forward ones
      bool discard = false;
      int j = 0;
      for (j = 0; !discard && j < nForward; j++) {
        if (hypot(iter->yC - m_Movements[j].yC,
                  iter->xC - m_Movements[j].xC) < 0.5) {
          discard = true;
          break;
        }
      }

      if (discard) {
        iter = m_Movements.erase(iter);
      } else {
        iter++;
      }
    }

  } else {
    std::cerr << "ScanMotionDetector still waiting for ref.scan buffer"
              << " to fill, have " << m_NScans 
              << " but need " << m_MaxNScans << std::endl;
    return false;
  }

  if (m_TryRemoveNonhumanMotion) tryRemoveNonhumanMotion();

  return (!m_Movements.empty());
}

void
ScanMotionDetector::tryRemoveNonhumanMotion()
{
  // We loop through the detected motion segments and try to find
  // those that are unlikely to have been cause by humans
  for (std::vector<Movement>::iterator mi = m_Movements.begin();
       mi != m_Movements.end();) {
    
    // We calculate how many pixels are connected together and how far
    // these are apart to determine if it might be a human or not
    
    int startI = mi->scanIndex[0];

    // Look towards lower indices
    int i = startI - 1;
    while (i > 0) {
      double d =hypot(m_FreeSpace[m_LastScan].yRaw[i] -
                      m_FreeSpace[m_LastScan].yRaw[startI],
                      m_FreeSpace[m_LastScan].xRaw[i] -
                      m_FreeSpace[m_LastScan].xRaw[startI]);
      
      if (d < m_MaxPtDistHumanSegment) {
        startI = i;
      }
      i--;
    }

    int stopI = mi->scanIndex[mi->nPts - 1];

    // Look towards higher indices
    i = stopI + 1;
    while (i < (m_FreeSpace[m_LastScan].n - 1) ) {
      double d =hypot(m_FreeSpace[m_LastScan].yRaw[i] -
                      m_FreeSpace[m_LastScan].yRaw[stopI],
                      m_FreeSpace[m_LastScan].xRaw[i] -
                      m_FreeSpace[m_LastScan].xRaw[stopI]);
      
      if (d < m_MaxPtDistHumanSegment) {
        stopI = i;
      }
      i++;
    }

    double size = hypot(m_FreeSpace[m_LastScan].yRaw[stopI] -
                        m_FreeSpace[m_LastScan].yRaw[startI],
                        m_FreeSpace[m_LastScan].xRaw[stopI] -
                        m_FreeSpace[m_LastScan].xRaw[startI]);

    if (size > m_MaxHumanSize || mi->nPts < m_MinNumPtsOnHuman) {
      mi = m_Movements.erase(mi);
    } else {
      mi++;
    }
  }
}

void 
ScanMotionDetector::renewFreeSpaceModel(double t, int n, double *r, 
                                           double startAngle, double angleStep,
                                           double xlF, double ylF, double alF)
{
  m_LastScan = (m_LastScan + 1) % m_MaxNScans;

  // Store basic information such as the time of the scan, the pose of
  // the laser in the fixed frame and setup of the laser scan
  m_FreeSpace[m_LastScan].t = t;
  m_FreeSpace[m_LastScan].xL = xlF;
  m_FreeSpace[m_LastScan].yL = ylF;
  m_FreeSpace[m_LastScan].aL = alF;
  m_FreeSpace[m_LastScan].aStart = startAngle;
  m_FreeSpace[m_LastScan].aStep = angleStep;
  m_FreeSpace[m_LastScan].n = n;

  m_FreeSpace[m_LastScan].nForwardMotionPts = 0;  

  // When we look for motion we will check if any of the points in the
  // new scans comes into the freespace area defined by an earlier
  // scan. To avoid too much noise we do some smoothing.  We smooth it
  // in a way that for each reading we put it equal to the shortest of
  // its smoothWidth neighbors and we also add an extra margin of
  // lengthMargin, i.e. we decrease it even more. Any new scan point
  // that violates the freespace defined by this area is likely to be
  // the result of motion.
  const int smoothWidth = 5;
  const double lengthMargin = 0.2;
  for (int i = 0; i < smoothWidth; i++) {
    m_FreeSpace[m_LastScan].r[i] = 0;
    m_FreeSpace[m_LastScan].rRaw[i] = r[i];
  }
  for (int i = smoothWidth; i < (n - smoothWidth); i++) {
    m_FreeSpace[m_LastScan].r[i] = r[i];
    m_FreeSpace[m_LastScan].rRaw[i] = r[i];
    for (int j = -smoothWidth; j <= smoothWidth; j++) {
      if (r[i+j] < m_FreeSpace[m_LastScan].r[i])
        m_FreeSpace[m_LastScan].r[i] = r[i+j];
    }
    m_FreeSpace[m_LastScan].r[i] -= lengthMargin;
    if (m_FreeSpace[m_LastScan].r[i] < 0)
      m_FreeSpace[m_LastScan].r[i] = 0;
  } 
  for (int i = (n - smoothWidth); i < n; i++) {
    m_FreeSpace[m_LastScan].r[i] = 0;
    m_FreeSpace[m_LastScan].rRaw[i] = r[i];
  }

  // The angle index into the lookup table of the first scan point of
  // the new scan
  int startIndex = 
    int(HSS::zero_to_2pi(m_FreeSpace[m_LastScan].aL + 
                         m_FreeSpace[m_LastScan].aStart) / 
        m_TrigLookupRes );

  // The number of index steps in the lookup table that each angle
  // step in the new scan corresponds to
  int stepi = int(m_FreeSpace[m_LastScan].aStep / m_TrigLookupRes + 0.5);
  for (int i = 0; i < n; i++) {
    m_FreeSpace[m_LastScan].xRaw[i] = m_FreeSpace[m_LastScan].xL +
      m_FreeSpace[m_LastScan].rRaw[i] * 
      m_CosLookup[(startIndex + stepi * i) % m_TrigLookupSize];
    m_FreeSpace[m_LastScan].yRaw[i] = m_FreeSpace[m_LastScan].yL +
      m_FreeSpace[m_LastScan].rRaw[i] * 
      m_SinLookup[(startIndex + stepi * i) % m_TrigLookupSize];     
  }

  if (m_NScans < m_MaxNScans) m_NScans++;
}

void 
ScanMotionDetector::checkForMotion(int refScan, int scan,
                                      bool predictForward)
{  
  /*
  std::cerr << "checkForMotion(" << refScan << "," << scan 
            << ", " << int(predictForward) << ")\n";
  */

  int method = ALG_FORWARD;
  if (m_FreeSpace[scan].t < m_FreeSpace[refScan].t) {
    method = ALG_BACKWARD;
  }

  // Counter of number of consecutive points that violates the
  // freespace defined by the reference scan
  int nViols = 0;

  // Loop through the scan and calculate the range from the center of
  // the refence scan to the scan points we are using to check for
  // motion.
  for (int i = m_SideSkip; i < (m_FreeSpace[scan].n - m_SideSkip); i++) {

    // When we look for backward motion we first check if this point
    // has already been classified as moving when this was the newest
    // scan and forward checking was done
    bool skipThisPt = false;
    if (method == ALG_BACKWARD) {
      for (int ii = 0; 
           !skipThisPt && ii < m_FreeSpace[scan].nForwardMotionPts; ii++) {
        if (i == m_FreeSpace[scan].forwardMotionPts[ii]) {
          skipThisPt = true;
          break;
        }
      }
    }
    if (skipThisPt) continue;

    // Get the distance from the reference scan center to the scan point
    double d = hypot(m_FreeSpace[scan].yRaw[i] - m_FreeSpace[refScan].yL,
                     m_FreeSpace[scan].xRaw[i] - m_FreeSpace[refScan].xL);

    // Angle from x-axis of the reference scan and the new obstacle point
    double aDiff = HSS::pi_to_pi(atan2(m_FreeSpace[scan].yRaw[i] - 
                                       m_FreeSpace[refScan].yL,
                                       m_FreeSpace[scan].xRaw[i] - 
                                       m_FreeSpace[refScan].xL) -
                                 m_FreeSpace[refScan].aL);

    // Calculate the index in the reference scan that the angle to the
    // x-axis above corresponds to
    int ai = int((aDiff + 0.5 * m_FreeSpace[refScan].aStep) / 
                 m_FreeSpace[refScan].aStep);

    bool violation = false;
    // To check for violation the new scan has to be enough inside the
    // reference scan.
    if (0 < ai && ai < (m_FreeSpace[refScan].n - 1)) {

      // To classify it as a violation we demand that the range is not
      // too big and that it is inside the free space defined by the
      // three closest points in the reference scan.
      if (d < m_MaxDetectableRange &&
          d < m_FreeSpace[refScan].r[ai-1] &&
          d < m_FreeSpace[refScan].r[ai] &&
          d < m_FreeSpace[refScan].r[ai+1]) {

        violation = true;
        nViols++;
      }
    }

    // If the difference in depth between to consecutive points is
    // very large we split the cluster
    bool splitCluster = false;
    if (violation) {
      double jumpB = 0, jumpF = 0;
      if (i > 0 && nViols > 1) {
        jumpB = fabs(m_FreeSpace[scan].rRaw[i] - m_FreeSpace[scan].rRaw[i-1]);
      } 
      if (i < (m_FreeSpace[scan].n - 1) && nViols > 0) {
        jumpF = fabs(m_FreeSpace[scan].rRaw[i] - m_FreeSpace[scan].rRaw[i+1]);
      }
      if (jumpB > m_MaxDepthDiff || jumpF > m_MaxDepthDiff){
        splitCluster = true;
      }
    }


    // We wait to process the points that violate the reference scan
    // until we reach the first point that do not violate
    if (!violation || splitCluster) {

      // Depending on how we entered this part of the code, i.e if a
      // cluster was split or it there is no more violation the first
      // point of violation is different.
      int firstViol;
      if (splitCluster) {
        firstViol = i - nViols + 1;
      } else {
        firstViol = i - nViols;
      }

      // The closer the points are that are violating the freespace
      // the more points we demand to avoid triggering too many false
      // alarms that might come from table legs etc that are only
      // sometimes detected and thus will make th efreespace "flicker"
      // in that area.
      int minReqNPts = int(m_MinViolationPts + 
                           (m_MaxDetectableRange - 
                            m_FreeSpace[scan].rRaw[firstViol]) *
                           m_ViolationPtsPerM);
      if (minReqNPts < m_MinViolationPts) minReqNPts = m_MinViolationPts;

      // We need to have at least aminimum of consecutive violating
      // points to clssify it as motion.
      if (nViols >= minReqNPts) {

        // We can only handle a certain amount of moving clusters
        Movement m;
        m.nPts = nViols;

        // Store the points that have been classified as
        // moving.
        for (int p = 0; p < nViols; p++) {
          m.scanIndex[p] = firstViol + p;
        }
        
        // Initialize the state of this blob as new since we do not
        // keep any history here
        m.state = 0;
        m.timestamp = m_FreeSpace[m_LastScan].t;
        
        // Store information about if it is forward or backward
        // extraction
        m.method = method;
        
        // If the scan we check for motion is the last scan acquired
        // we store the points that were classified as moving
        if (scan == m_LastScan) {
          memcpy(m_FreeSpace[scan].forwardMotionPts + 
                 m_FreeSpace[scan].nForwardMotionPts,  
                 m.scanIndex, sizeof(unsigned short) * m.nPts);
          m_FreeSpace[scan].nForwardMotionPts += m.nPts;
        }
        
        // If we are told to predict forward we do so. This is
        // typically the case when reverse the order of the scans
        // and use the last scan as the reference scan and the
        // oldest to check for violations. We will then get the
        // position of the moving object some time ago and the
        // indices will not refer to the last scan either.
        bool shouldKeepNew = false;
        if (predictForward) {

          if (predictMotionForward(m, scan)) {
            if (calcCenterOfMass(m, m_LastScan, m_PredAcc)) {
              shouldKeepNew = true;
            }
          }
        } else {
          if (calcCenterOfMass(m, scan)) {
            shouldKeepNew = true;
          }
        }

        if (shouldKeepNew) {
          m_Movements.push_back(m);
        }
      }
      nViols = 0;
    }

  }
}

bool
ScanMotionDetector::calcCenterOfMass(Movement &m, int scan,
                                     unsigned short *predAcc)
{  
  if (m.nPts <= 0) return false;

  double xsum = 0;
  double ysum = 0;
  int numPts = 0;
  for (int i = 0; i < m.nPts; i++) {
    if (predAcc) {
      xsum += m_FreeSpace[scan].xRaw[m.scanIndex[i]] * predAcc[i];
      ysum += m_FreeSpace[scan].yRaw[m.scanIndex[i]] * predAcc[i];
      numPts += predAcc[i];
    } else {
      xsum += m_FreeSpace[scan].xRaw[m.scanIndex[i]];
      ysum += m_FreeSpace[scan].yRaw[m.scanIndex[i]];
      numPts++;
    }
  }
  m.xC = xsum / numPts;
  m.yC = ysum / numPts;

  return true;
}

/**
 * FIXME This function needes some serious optimizing. There are way
 * too many calls to cos and sin hidden in the transformations
 */  
bool
ScanMotionDetector::predictMotionForward(Movement &m, int scan) 
{
  if (scan < 0 || scan >= m_NScans) {
    std::cerr << "ScanMotionDetector scan index " << scan 
              << " out of bounds\n";
    return false;
  }

  if (scan == m_LastScan) {
    std::cerr << "ScanMotionDetector "
              << "No need to predict, already at last scan\n";
    return true;
  }

  // Check how many scans we have to predict.
  int nSteps;
  if (m_LastScan < scan)
    nSteps = m_LastScan + m_NScans - scan;
  else 
    nSteps = m_LastScan - scan;

  // To allow for tracking over time we use the avegare motion from
  // the last frame to predict how the motion in the next
  double prevAvgDx = 0;
  double prevAvgDy = 0;

  // Initialize the accumulator that keeps track of how many points
  // are mapped to a certain point when moving between scans. This is
  // used to correctly calculate the average motion for prediction and
  // the later the center of mass.
  unsigned short newPredAcc[m.nPts];
  for (int i = 0; i < m.nPts; i++) {
    m_PredAcc[i] = 1;
  }

  // We keep track of how many old points that we have successfully
  // tracked
  int numOrigPts = 0;

  // We do the prediction by simply looking for the nearest neighbor
  Movement newM;
  for (int i = 0; i < nSteps; i++) {

    numOrigPts = 0;
    newM.nPts = 0; 
    FreeSpaceScan &fss1 = m_FreeSpace[(scan + i) % m_MaxNScans];
    FreeSpaceScan &fss2 = m_FreeSpace[(scan + i + 1) % m_MaxNScans];

    // We put a limit on the difference that we allow between the
    // position in one scan and the next when we look for nearest neighbor
    const double maxObjSpeed = 3.0; // m/s
    double maxObjDist = maxObjSpeed * (fss2.t - fss1.t);
    double dThres = maxObjDist+hypot(fss2.yL - fss1.yL, fss2.xL - fss1.xL);
    if (dThres < 0.2) dThres = 0.2;

    // Calculate the rotation betwen the scans and how many scan
    // indices that corresponds to
    int angDiffI = int(HSS::pi_to_pi(fss2.aL - fss1.aL) / fss2.aStep);

    double dx[m.nPts];
    double dy[m.nPts];
    double dxAvg = 0;
    double dyAvg = 0;
    for (int j = 0; j < m.nPts; j++) {
      // Index to point in first scan
      int ai1 = m.scanIndex[j];
      // The approximate index in the other scan for this point
      // assuming pure rotation
      int ai2 = ai1 - angDiffI;

      // We make the search quite narrow since the backward search is
      // only supposed to detect motion straight away from the
      // sensor. The other type of motion should be detected by the
      // forward method.
      const int width = 5;
      int bestI = -1;
      double minD = dThres;
      for (int k = ai2 - width; k <= ai2 + width; k++) {
        if (0 <= k && k < fss2.n) {
          double d = hypot(fss2.yRaw[k] - (fss1.yRaw[ai1] + prevAvgDy),
                           fss2.xRaw[k] - (fss1.xRaw[ai1] + prevAvgDx));
          if (d < minD) {
            minD = d;
            bestI = k;
          }
        }
      }
      
      if (bestI > 0) {
        dx[newM.nPts] = fss2.xRaw[bestI] - fss1.xRaw[m.scanIndex[j]];
        dy[newM.nPts] = fss2.yRaw[bestI] - fss1.yRaw[m.scanIndex[j]];
        dxAvg += dx[newM.nPts] * m_PredAcc[j];
        dyAvg += dy[newM.nPts] * m_PredAcc[j];
        newPredAcc[newM.nPts] = m_PredAcc[j];
        numOrigPts += m_PredAcc[j];
        newM.scanIndex[newM.nPts] = bestI;
        newM.nPts++;
      }
    }

    if (newM.nPts == 0) {
      m.nPts = 0;
      return false;
    } else {

      dxAvg /= numOrigPts;
      dyAvg /= numOrigPts;

      const double maxDiff2Avg = 0.15;

      // Remove points that are too far from the average and recalculate
      // the average
      double avgDx = 0;
      double avgDy = 0;
      int nAvgPts = 0;
      for (int k = 0; k < newM.nPts; k++) {
        if (hypot(dy[k] - dyAvg, dx[k] - dxAvg) <= maxDiff2Avg) {
          avgDx += dx[k] * newPredAcc[k];
          avgDy += dy[k] * newPredAcc[k];
          nAvgPts += newPredAcc[k];
        }
      }
      if (nAvgPts > 0) {
        avgDx /= nAvgPts;
        avgDy /= nAvgPts;
      } else {
        avgDx = dxAvg;
        avgDy = dyAvg;
      }
      prevAvgDx = avgDx;
      prevAvgDy = avgDy;
      

      // Update the information about the points that are moving. We
      // make sure at the same time that points are not represented
      // twice and we also remove points that moved in a very
      // differently from teh rest of the points. These often
      // correspond to mixed pixel points or points that are
      // associated with another target, ths splitting the cluster.
      m.nPts = 0;
      for (int k = 0; k < newM.nPts; k++) {
        bool skip = false;

        // Make sure that it is not there already
        for (int kk = 0; !skip && kk < m.nPts; kk++) {
          if (m.scanIndex[kk] == newM.scanIndex[k]) {            
            skip = true;

            // If this is a duplicate it means that more than one
            // point has been mapped to the same point in the next
            // scan so we need to increment the accumulator to account
            // for this
            m_PredAcc[kk] += newPredAcc[k];
          }
        }

        // Make sure that this point did not move very differentl from
        // the rest of the points from one scan the other
        if (hypot(dy[k] - avgDy, dx[k] - avgDx) > maxDiff2Avg) {
          skip = true;
        }

        if (!skip) {
          m_PredAcc[m.nPts] = newPredAcc[k];
          m.scanIndex[m.nPts] = newM.scanIndex[k];
          m.nPts++;          
        }
      }
    }
  }

  return true;
}

void 
ScanMotionDetector::displayMotion(peekabot::GroupProxy &refroot)
{
  if (m_Movements.empty()) return;

  peekabot::GroupProxy CMs;
  CMs.add(refroot, "CMs", peekabot::REPLACE_ON_CONFLICT);

  for (unsigned int i = 0; i < m_Movements.size(); i++) {
    peekabot::LineCloudProxy lcp;
    char name[128];
    sprintf(name,"motion%02d", i);
    lcp.add(CMs, name); 
    
    peekabot::VertexSet vs;
    
    double astep = 2.0*M_PI/40;
    for (double a = 0; a < 2.0*M_PI; a+=astep) {
      vs.add_vertex(m_Movements[i].xC + 0.1*cos(a),
                    m_Movements[i].yC + 0.1*sin(a),
                    0);
      vs.add_vertex(m_Movements[i].xC + 0.1*cos(a+astep),
                    m_Movements[i].yC + 0.1*sin(a+astep),
                    0);
    }
    
    lcp.set_vertices(vs);
    lcp.set_color(0,0,0);
  }

  peekabot::PointCloudProxy pcp;
  pcp.add(refroot, "motionpts", peekabot::REPLACE_ON_CONFLICT);
  {
    peekabot::VertexSet vs;
    for (unsigned int i = 0; i < m_Movements.size(); i++) {
      for (int j = 0; j < m_Movements[i].nPts; j++) {
        vs.add_vertex(m_FreeSpace[m_LastScan].xRaw[m_Movements[i].scanIndex[j]],
                      m_FreeSpace[m_LastScan].yRaw[m_Movements[i].scanIndex[j]],
                      0.001);
      }
    }
    pcp.set_vertices(vs);
  }
  pcp.set_color(0,0,0);
}

void 
ScanMotionDetector::displayFreeSpace(peekabot::GroupProxy &refroot,
                                        bool backwardFreeSpace,
                                        bool forwardFreeSpace)
{
  if (m_NScans < m_MaxNScans) return;

  peekabot::GroupProxy freespace;
  freespace.add(refroot, "freespace", peekabot::REPLACE_ON_CONFLICT);
  
  int sindex = 0;
  
  for (int j = 0; j < 2; j++) {
    
    if (((j == 0) && forwardFreeSpace) ||
        ((j == 1) && backwardFreeSpace)) {

      peekabot::LineCloudProxy lcp;

      if (j == 0) {
        // Reference scan for forward test
        sindex = (m_LastScan + 1) % m_NScans;

        lcp.add(freespace, "forwref");
      } else {
        // Reference scan for backward test
        sindex = m_LastScan;

        lcp.add(freespace, "backref");
      }
      
      peekabot::VertexSet vs;      
      int n = m_FreeSpace[sindex].n;
      for (int i = 0; i < (n - 1); i++) {
        
        double  aS = m_FreeSpace[sindex].aL + 
          m_FreeSpace[sindex].aStart + m_FreeSpace[sindex].aStep * i;
        double  aE = m_FreeSpace[sindex].aL + 
          m_FreeSpace[sindex].aStart + m_FreeSpace[sindex].aStep * (i + 1);
        
        vs.add_vertex(m_FreeSpace[sindex].xL + 
                      m_FreeSpace[sindex].r[i] * cos(aS),
                      m_FreeSpace[sindex].yL +
                      m_FreeSpace[sindex].r[i] * sin(aS),
                      0);       
        vs.add_vertex(m_FreeSpace[sindex].xL +
                      m_FreeSpace[sindex].r[i+1] * cos(aE),
                      m_FreeSpace[sindex].yL +
                      m_FreeSpace[sindex].r[i+1] * sin(aE),
                      0);
      }
      lcp.set_vertices(vs);

      if (j==0) {
        // Forward test zone is RED
        lcp.set_color(1,0,0);
      } else {
        // Backward test zone is BLUE
        lcp.set_color(0,0,1);
      }

    }
  }

  if (backwardFreeSpace) {

    // Mark the points that were originally detected in the backward test
    sindex = (m_LastScan - m_BackwardRef + m_NScans) % m_NScans;

    peekabot::PointCloudProxy pcp;
    pcp.add(freespace, "backpts");
    {
      peekabot::VertexSet vs;      
      int n = m_FreeSpace[sindex].n;
      for (int i = 0; i < (n - 1); i++) {
        
        vs.add_vertex(m_FreeSpace[sindex].xRaw[i],
                      m_FreeSpace[sindex].yRaw[i],
                      0);
      }
      pcp.set_vertices(vs);
    }
    pcp.set_color(0,0,1);

    // Mark the position of the sensor at the backward test situation
    peekabot::LineCloudProxy lcp;
    lcp.add(freespace, "backpose");
    {
      peekabot::VertexSet vs;
      vs.add_vertex(m_FreeSpace[sindex].xL,
                    m_FreeSpace[sindex].yL,
                    0);
      vs.add_vertex(m_FreeSpace[sindex].xL + 5.0*cos(m_FreeSpace[sindex].aL+M_PI_2),
                    m_FreeSpace[sindex].yL + 5.0*sin(m_FreeSpace[sindex].aL+M_PI_2),
                    0);
      vs.add_vertex(m_FreeSpace[sindex].xL - 5.0*cos(m_FreeSpace[sindex].aL),
                    m_FreeSpace[sindex].yL - 5.0*sin(m_FreeSpace[sindex].aL),
                    0);
      vs.add_vertex(m_FreeSpace[sindex].xL + 5.0*cos(m_FreeSpace[sindex].aL),
                    m_FreeSpace[sindex].yL + 5.0*sin(m_FreeSpace[sindex].aL),
                    0);
      lcp.set_vertices(vs);
    }
    lcp.set_color(0,0,1);
  }
}

void 
ScanMotionDetector::displayAllReferenceScans(peekabot::GroupProxy &refroot)
{
  if (m_NScans < m_MaxNScans) return;

  /*
  for (int j = 0; j < m_NScans; j++) {

    int scan = (m_LastScan - j + m_NScans) % m_NScans;

    int n = m_FreeSpace[scan].n;
    RL_RectangleItem pts[n+1];

    for (int i = 0; i < n; i++) {
      pts[i].xC = m_FreeSpace[scan].xRaw[i];
      pts[i].yC = m_FreeSpace[scan].yRaw[i];
      pts[i].z = 0;
      if (scan == m_LastScan) pts[i].color = 3;
      else pts[i].color = 4+(scan%4);
      pts[i].xLen = 0.015;
      pts[i].yLen = 0.015;
      pts[i].xAngle = 0;
      pts[i].style = 0;
      pts[i].floor = 0;
    }

    // mark the pose of the scanner
    pts[n].xC = m_FreeSpace[j].xL;
    pts[n].yC = m_FreeSpace[j].yL;
    pts[n].z = 0;
    pts[n].color = 2;
    pts[n].xLen = 0.1;
    pts[n].yLen = 0.1;
    pts[n].xAngle = m_FreeSpace[j].aL;
    pts[n].style = 0;
    pts[n].floor = 0;

    rlp->addRectangles(RL_ENV_EST, pts, n+1, false);
  }
*/
}

}; // namespace HSS
