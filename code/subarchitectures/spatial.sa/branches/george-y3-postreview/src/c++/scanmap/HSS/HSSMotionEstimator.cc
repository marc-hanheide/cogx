//
// = FILENAME
//    HSSMotionEstimator.cc
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

#include "HSSMotionEstimator.hh"
#include "HSSutils.hh"
#include "HSSScan2DMatcher.hh"

namespace HSS {

MotionEstimator::MotionEstimator(bool odomOnly)
{
  m_OdomOnly = odomOnly;

  m_OdomVarDist= 0.05; // 0.01;
  m_OdomVarDir = 0.05; // 0.01;
  m_OdomVarRot = 0.05; // 0.01;
  m_OdomVarRotDist = 0.01; // 0.005;

  m_StdMeasXY = 0.05; // 0.005;
  m_StdMeasDeg = 0.2; // 0.1;
}

bool 
MotionEstimator::estimateMotion(Eigen::Vector3d &odomNew, 
                                Eigen::Vector3d &odomOld,
                                HSS::CandScan2D &scanNew, 
                                HSS::CandScan2D &scanOld,
                                Eigen::Vector3d &xsR, 
                                Eigen::Vector3d &delta, Eigen::Matrix3d &Q)
{
  Eigen::Vector3d delta_odom(HSS::compound(HSS::icompound(odomOld),odomNew));
  Eigen::Vector3d delta_sm;

  // We determine which "state" we are in depending on the situation
  // and then how to use the information
  // 0: pure odom, 
  // 1: pure sm, 
  // 2: corridor type env where we use odometry along the corridor

  int state = 0; 

  if (m_OdomOnly) {

    state = 0;

  } else {

    int n = 0;
    Eigen::Vector3d Tguess;
    Tguess[0] = -delta_odom[1];
    Tguess[1] = delta_odom[0];
    Tguess[2] = delta_odom[2];

    Scan2DMatcher matcher;
    
    if (!matcher.scanMatch(scanOld, scanNew, Tguess, delta_sm, n, NULL)) {

      //std::cerr << "Scan matching failed\n";
      
      // We have no choice we have to rely on odometry
      state = 0;
      
    } else {
      
      // Get the result from scan matching from the sensor frame to
      // the robot frame
      delta_sm = HSS::compound(HSS::compound(xsR, delta_sm), 
                               HSS::icompound(xsR));
      
      //std::cerr << "Scan matching succeeded\n";
      
      // If scan matching gives a very small result it is probably more
      // reliable to use odometry since we get rid of the noise largely
      if (hypot(delta_sm[0], delta_sm[1]) < 1e-2 &&
          hypot(delta_odom[0], delta_odom[1]) < 1e-2 &&
          fabs(delta_sm[2]) < HSS::deg2rad(1) &&
          fabs(delta_odom[2]) < HSS::deg2rad(1)) {

        // Use odometry because both sources say that we moved very
        // little
        state = 0;
        
      } else if (scanNew.m_Corridor || scanOld.m_Corridor) {
        
        // If we are in a corridor like environment, we use dometry
        // along the corridor and scan matching across and for the
        // angle estimate
        
        state = 2;
        
      } else {
        
        // Use the scan matching result since we moved more than a
        // little bit, and it seems that we are not in a corridor like
        // environment. Pure scan matching data should be the best
        state = 1;
        
      }
    }
  }

  if (state == 0) {
    // Scan matching failed and we have to use odometry

    // If odometry says that we did not move than we say so too since
    // scan matching gives us nothing
    if (fabs(delta_odom[0]) < 1e-5 && 
        fabs(delta_odom[1]) < 1e-5 && 
        fabs(delta_odom[2]) < HSS::deg2rad(0.05)) return false;
    
    double delta_dist = hypot(delta_odom[1], delta_odom[0]);   
    double delta_dir = atan2(delta_odom[1], delta_odom[0]);
    double delta_rot = delta_odom[2];

    // Odometry uncertainty expressed in terms of distance moved,
    // direction of motion and amount rotated
    Q.setIdentity();
    Q *= 1e-12;
    Q(0,0) = m_OdomVarDist * delta_dist;
    Q(1,1) = m_OdomVarDir * fabs(delta_dir);
    Q(2,2) = m_OdomVarRot * fabs(delta_rot) + 
      m_OdomVarRotDist * fabs(delta_dist);
    
    // Jacobian to turn Q into uncertainty in delta_x,y,theta
    // delta_x = delta_dist * cos(delta_dir)
    // delta_y = delta_dist * sin(delta_dir)
    // delta_a = delta_rot
    // cos(delta_dir) = delta_x / delta_dist
    // sin(delta_dir) = delta_y / delta_dist
   
    Eigen::Matrix3d J_xya;
    J_xya.setZero();
    J_xya(0,0) = delta_odom[0] / delta_dist;
    J_xya(0,1) =-delta_odom[1];
    J_xya(1,0) = delta_odom[1] / delta_dist;
    J_xya(1,1) = delta_odom[0];
    J_xya(2,2) = 1;

    Q = J_xya * Q * J_xya.transpose();

    delta = delta_odom;
    return true;

  } else if (state == 1) {

    // If odometry says that we did not move than we say so too 
    if (fabs(delta_sm[0]) < 5e-3 && 
        fabs(delta_sm[1]) < 5e-3 && 
        fabs(delta_sm[2]) < HSS::deg2rad(0.05)) return false;

    Q.setZero();
    Q *= 1e-12;
    Q(0,0) = Q(1,1) = HSS::sqr(m_StdMeasXY);
    Q(2,2) = HSS::sqr(HSS::deg2rad(m_StdMeasDeg));

    delta = delta_sm;
    return true;

  } else if (state == 2) {

    

    // Project the motion estimates from the odometry and scan
    // matching from the robot frame to the corridor frame where the
    // corridor is define as running along the x-axis. We pick the
    // x-estimate from the odometry and the y-estimate from scan
    // matching.
    double dx_corr = cos(scanNew.m_MainDirR)*delta_odom[0] + 
      sin(scanNew.m_MainDirR)*delta_odom[1];    
    double dy_corr =-sin(scanNew.m_MainDirR)*delta_sm[0] + 
      cos(scanNew.m_MainDirR)*delta_sm[1];

    if (fabs(dx_corr) < 1e-3 && 
        fabs(dy_corr) < 1e-3 && 
        fabs(delta_sm[2]) < HSS::deg2rad(0.05)) return false;
    
    // In this case we have uncertainty specified in the corridor frame
    Q.setZero();
    Q *= 1e-12;
    Q(0,0) = m_OdomVarDist * fabs(dx_corr);
    Q(1,1) = HSS::sqr(m_StdMeasXY);
    Q(2,2) = HSS::sqr(HSS::deg2rad(m_StdMeasDeg));

    // The Jacobian that turns this Q into the frame of the robot is just the rotation matrix
    Eigen::Matrix3d J;
    J.setZero();
    J(0,0) = cos(scanNew.m_MainDirR);
    J(0,1) =-sin(scanNew.m_MainDirR);
    J(1,0) =-J(0,1);
    J(1,1) = J(0,0);
    J(2,2) = 1;

    Q = J * Q * J.transpose();

    // Transform these estimates back to the robot frame
    delta[0] = J(0,0)*dx_corr + J(0,1)*dy_corr;
    delta[1] = J(1,0)*dx_corr + J(1,1)*dy_corr;
    delta[2] = delta_sm[2];

    return true;
    
  }

  std::cerr << "WARNING: Should not ever get here, since state SHOULD be 0,1 or 2, "
            << "but it is " << state << std::endl;
  return false;
}

}; // namespace HSS

