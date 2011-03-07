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
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSMotionEstimator_hh
#define HSSMotionEstimator_hh

#include "HSSScan2DMatcher.hh"

namespace HSS {

/**
 * This class performs motion estimation using oodmetry and/or scan
 * matching.
 *
 * @author Patric Jensfelt
 * @see
 */
class MotionEstimator {

public:

  MotionEstimator(bool odomOnly);

  /**
   * This function tries to estimate the motion of the robot based on
   * odometry. The motion is expressed in the
   * frame of the "old" robot pose. If this function returns false and
   * thus says that there was no motion you should keep the old
   * odometry and feed this in again to avoid getting an
   * accumulation of error when the robot is moving very slowly. The
   * threshold for motion is cannot be 0 and thus if you sample fast
   * and the robot is moving slowly it is likely that a true motion
   * will be classified as standing still sometimes.
   *
   * @param odomNew newest odometry reading
   * @param odomOld previously used new odometry reading
   * @param delta the estimated motion
   * @param Q the covariance in the motion estimate
   *
   * @return true if the platform has moved, else false
   */
  bool estimateMotion(Eigen::Vector3d &odomNew, Eigen::Vector3d &odomOld,
                      Eigen::Vector3d &delta, Eigen::Matrix3d &Q);

  /**
   * This function tries to estimate the motion of the robot based on
   * odometry and/or scan matching. The motion is expressed in the
   * frame of the "old" robot pose. If this function returns false and
   * thus says that there was no motion you should keep the old
   * odometry/scans and feed these in again to avoid getting an
   * accumulation of error when the robot is moving very slowly. The
   * threshold for motion is cannot be 0 and thus if you sample fast
   * and the robot is moving slowly it is likely that a true motion
   * will be classified as standing still sometimes.
   *
   * @param odomNew newest odometry reading
   * @param odomOld previously used new odometry reading
   * @param scanNew newest scan
   * @param scanOld previously used new scan
   * @param xsR laser position in robot frame
   * @param delta the estimated motion
   * @param Q the covariance in the motion estimate
   *
   * @return true if the platform has moved, else false
   */
  bool estimateMotion(Eigen::Vector3d &odomNew, Eigen::Vector3d &odomOld,
                      HSS::CandScan2D &scanNew, HSS::CandScan2D &scanOld,
                      Eigen::Vector3d &xsR, 
                      Eigen::Vector3d &delta, Eigen::Matrix3d &Q);

protected:
  bool m_OdomOnly;

  double m_OdomVarDist;
  double m_OdomVarDir;
  double m_OdomVarRot;
  double m_OdomVarRotDist;

  double m_StdMeasXY;
  double m_StdMeasDeg;
};

};

#endif // HSSMotionEstimator_hh
