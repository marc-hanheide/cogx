// = FILENAME
//    NewTurnMotion.cc 
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2004 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "NewTurnMotion.hpp"

#include <Utils/CureDebug.hh>
#include <Utils/HelpFunctions.hh>
#include <Utils/RoboLookProxy.h>

#ifndef DEPEND
#include <float.h>
#include <sstream>
#endif

using namespace Cure;
using namespace HelpFunctions;

#define MODE_TURN      0
#define MODE_FORWARD   1
#define MODE_BACKWARD  2

NewTurnMotion::NewTurnMotion(cast::ManagedComponent* sc,double securityDistance)
  :ds_(securityDistance),
   mode_(MODE_TURN),m_sc(sc)
{
  m_sc->debug("NewTurnMotion started"); 
  nClosePts_ = 0;
  closePts_ = new CloseObstPt[2000];

  nViolationPts_ = 0;
  violationPts_ = new CloseObstPt[2000];

  nHalf_ = n_ / 2;
  sectPerRad_ = 1.0 / (2.0 * M_PI / n_);

  freeZoneFB_ = 0.9;
  freeZoneS_ = 0.2;

  setRectangularShape(0.5, 0.45, -0.06);
}

NewTurnMotion::~NewTurnMotion()
{
  if (closePts_) {
    delete [] closePts_;
    closePts_ = 0;
  }
  if (violationPts_) {
    delete [] violationPts_;
    violationPts_ = 0;
  }
}

int
NewTurnMotion::calcMotionCommand(LocalMap &m, double aGoal,
                              double vMax, MotionAlgorithm::MotionCmd &cmd)
{
  robX_ = m.getX();
  robY_ = m.getY();
  robA_ = m.getA();

  goalA_ = aGoal;

  cmd.v = 0;
  cmd.dir = aGoal;
  cmd.w = 0;
  cmd.type = MotionAlgorithm::CMD_TYPE_STOP;

  // Check if obstacles are only in front of the robot
  clearBackward_ = m.obstacleFreeRegion(robX_ + frontDist_ * cos(robA_),
                                        robY_ + frontDist_ * sin(robA_),
                                        robX_ - ((backDist_ + freeZoneFB_) * 
                                                 cos(robA_)),
                                        robY_ - ((backDist_ + freeZoneFB_) * 
                                                 sin(robA_)),
                                        2.0 * (halfWidth_ + freeZoneS_));
                                        
  // Check if the obstacles are only behind the robot
  clearForward_ = m.obstacleFreeRegion(robX_ + ((frontDist_ + freeZoneFB_) * 
                                                cos(robA_)),
                                       robY_ + ((frontDist_ + freeZoneFB_) * 
                                                sin(robA_)),
                                       robX_ - backDist_ * cos(robA_),
                                       robY_ - backDist_ * sin(robA_),
                                       2.0 * (halfWidth_ + freeZoneS_));

  m_sc->debug("clearBackward_=%d  clearForward_=%d\n",clearBackward_,clearForward_);

  int err = MCTRL_ERROR_TURN_STUCK;

  if (!findCloseObstacles(m)) {
    // There are no close points and it is OK to turn freely
    m_sc->debug("No close obstacles\n");
    err = MCTRL_ERROR_NONE;
    cmd.type = MotionAlgorithm::CMD_TYPE_VA;
  } else {
    if (inStuckSituation()) {
      // We are stuck and cannot get out of the situation
      m_sc->debug("We are stuck\n");
    } 

    // Check if we can turn on the spot without getting any violations
    else if (canTurnOnTheSpot()) {
      mode_ = MODE_TURN;
      err = MCTRL_ERROR_NONE;
      cmd.type = MotionAlgorithm::CMD_TYPE_VA;
      m_sc->debug("Can turn on the spot\n");
    } else {

      // If we cannot turn we keep the same angle
      cmd.dir = robA_;
      
      // Check if we can change to a mode where we move
      if (changeToMoveMode()) {
        if (calcTranslationCommand(cmd.v)) {
          err = MCTRL_ERROR_NONE;
          cmd.type = MotionAlgorithm::CMD_TYPE_VW;
        }
      }
    } 
  }

  m_sc->log("mode=%d robA=%f dir=%f vel=%f err=%d type=%d\n",mode_,rad2deg(robA_),rad2deg(cmd.dir),cmd.v,err,cmd.type);

  return err;
}

bool 
NewTurnMotion::findCloseObstacles(LocalMap &m)
{
  nClosePts_ = 0;

  const double margin = 0.2;
  double searchRadius = maxRadius_ + ds_ + margin;

  minObstDist_ = 1e20;
  int minObstIndex = -1;
  for (unsigned int i = 0; i < m.nObst(); i++) {
    double d = hypot(m.obstRef(i).y - robY_, m.obstRef(i).x - robX_);
    if (d < searchRadius) {
      closePts_[nClosePts_].o = &(m.obstRef(i));      
      nClosePts_++;
      if (d < minObstDist_) {
        minObstIndex = i;
        minObstDist_ = d;
      }
    }
  }

  if (minObstIndex >= 0) {
    double a = angleDiffRad(atan2(m.obstRef(minObstIndex).y - robY_,
                                  m.obstRef(minObstIndex).x - robX_),
                            robA_);
    minObstDist_ -= E_[getSector(a)];
  }
                                

  m_sc->debug("Found %d close point(s)\n",nClosePts_);
  return (nClosePts_ > 0);
}

bool
NewTurnMotion::inStuckSituation()
{
  // Check if there are obstacles very close to the robot both in the
  // front and in the back on one side (or both) of the robot.  If the
  // obsytacles are very close bot in front and in the back we would
  // have to translate a lot to get out of the situation.
  
  double cosFront = cos(robA_ + M_PI_2);
  double sinFront = sin(robA_ + M_PI_2);
  double cosSide = cos(robA_);
  double sinSide = sin(robA_);

  bool closeFrontLeft = false;
  bool closeFrontRight = false;
  bool closeRearLeft = false;
  bool closeRearRight = false;

  // We demand that we should be able to turn at least a few degs away
  // from obstacles if they are both in the front and back not to say
  // that we are stuck. If they are closer we will have to move very
  // much to turn around

  // Half length if symmetric vehicle
  double hlen = HelpFunctions::max(frontDist_, backDist_);

  // The clearance we need on the side of the robot to be able to turn
  // 5 degs
  double minDist = hlen * tan(deg2rad(5));

  for (int i = 0; i < nClosePts_; i++) {
    // Check if the point is on the side of the robot, i.e. behind the
    // front and infront of the back
    double fdist = distPt2Line(closePts_[i].o->x, closePts_[i].o->y,
                              robX_, robY_, cosFront, sinFront);
    // We skip the points close to the middle not to be too sensitive
    // to small perturbations when determining what is in front and
    // what is behind the center
    if (fabs(fdist) < hlen && fabs(fdist) > 0.05) {

      // Distance to the side
      double sdist = distPt2Line(closePts_[i].o->x, closePts_[i].o->y,
                                 robX_, robY_, cosSide, sinSide);

      if (fabs(sdist) < halfWidth_ + ds_ + minDist) {
        if (fdist > 0) {
          // This point is in the front
          if (sdist > 0) {
            // This point is on the right side
            closeFrontRight = true;
          } else {
            // This point is on the left side
            closeFrontLeft = true;
          }
        } else {
          // This point is in the back
          if (sdist > 0) {
            // This point is on the right side
//FIXME never get stuck. the robot doesn't have the sensors on the back.

          //  closeRearRight = true;
          } else {
            // This point is on the left side
          //  closeRearLeft = true;
          }
        }
      }
    }               
  }

  if ((closeFrontLeft && closeRearLeft) ||
      (closeFrontRight && closeRearRight)) {
    m_sc->error("Too close both infront and back\n");
    return true;
  }
  
  return false;
      
}

bool
NewTurnMotion::canTurnOnTheSpot()
{
  double da = angleDiffRad(goalA_, robA_);
  const double maxLookAhead = deg2rad(10);
  if (da > maxLookAhead) da = maxLookAhead;
  if (da < -maxLookAhead) da = maxLookAhead;

  const double stepSize = deg2rad(2.5);

  int nSteps = int(da / stepSize + 0.5);

  for (int i = 0; i < nSteps; i++) {
    if (findViolations(robX_, robY_, robA_ + stepSize * i)) return false;
  }

  return true; 
}

bool
NewTurnMotion::findViolations(double rX, double rY, double rA)
{
  nViolationPts_ = 0;

  m_sc->debug("Testing xR=%f yR=%f aR=%f",rX,rY,rad2deg(rA));

  violFrontLeft_ = 0;
  violFrontRight_ = 0;
  violRearLeft_ = 0;
  violRearRight_ = 0;

  for (int i = 0; i < nClosePts_; i++) {
    double d = hypot(closePts_[i].o->y - rY, closePts_[i].o->x - rX);
    double a = angleDiffRad(atan2(closePts_[i].o->y - rY, 
                                  closePts_[i].o->x - rX), 
                            rA);
    int s = getSector(a);
    if (d < E_[s] + ds_) {
      m_sc->log("Found violation at x=%f y=%f d=%f dd=%f s=%d a=%f", closePts_[i].o->x,closePts_[i].o->y,d,d - E_[s],s,rad2deg(a));
      violationPts_[nViolationPts_].o = closePts_[i].o;
      violationPts_[nViolationPts_].a = a;
      violationPts_[nViolationPts_].d = d;
      violationPts_[nViolationPts_].s = s;
      nViolationPts_++;
//FIXME Never get stuck
      if (a > M_PI_2) ;//violRearLeft_ = 1;
      else if (a > 0) violFrontLeft_ = 1;
      else if (a > -M_PI_2) violFrontRight_ = 1;
      else ;//violRearRight_ = 1;
    }
  }

  m_sc->log("violation regions rl=%d fl=%d rr=%d fr=%d",violRearLeft_,violFrontLeft_,violRearRight_,violFrontRight_);

  m_sc->debug("Found %d violation point(s)\n",nViolationPts_);
  return (nViolationPts_ > 0);
}

bool
NewTurnMotion::changeToMoveMode()
{
  if (nViolationPts_ == 0) {
    m_sc->error("No violation points, cannot change to move mode\n");
    return false;
  }

  if ((violFrontLeft_ || violFrontRight_) && 
      (violRearLeft_ || violRearRight_)) {
    m_sc->error("Obstacles both in front and rear!!!\n");
    return false;
  }

  if (violFrontLeft_ || violFrontRight_) {
    if (mode_ == MODE_FORWARD)
      m_sc->debug("Changed to moving backward\n");
    else
       m_sc->debug("Started to move backward\n");
    mode_ = MODE_BACKWARD;
  } else {
    if (mode_ == MODE_BACKWARD)
       m_sc->debug("Changed to moving forward\n");
    else
       m_sc->debug("Started to move forward\n");
    mode_ = MODE_FORWARD;
  }

  return true;
}

bool
NewTurnMotion::calcTranslationCommand(double &vel)
{
  if (mode_ == MODE_BACKWARD) {
    if (violRearRight_ || violRearLeft_) {
      mode_ = MODE_TURN;
      vel = 0;
    } else {
      vel = -0.5 * minObstDist_;
      if (clearBackward_) {
        if (vel > -0.05) vel = -0.05;
      } else {
        if (vel > -0.01) vel = -0.01;
      }
    }
  } else if (mode_ == MODE_FORWARD) {
    if (violFrontRight_ || violFrontLeft_) {
      mode_ = MODE_TURN;
      vel = 0;
    } else {
      vel = 0.5 * minObstDist_;
      if (clearBackward_) {
        if (vel < 0.05) vel = 0.05;
      } else {
        if (vel < 0.01) vel = 0.01;
      }
    }
  } else {
     m_sc->error("Logic error,mode turn in trans calc\n");
    vel = 0;
  } 

  return true;
}

bool
NewTurnMotion::configRobotShape(int type, const std::string &params)
{
  // Make a stream that simplifies getting the parameters out from the
  // string
  std::istringstream str(params);

   m_sc->debug("type=%d params=\"%s\"\n", type,params.c_str());

  bool ret = true;

  if (type == 0) {
    double radius = 0.3;
    double centerOffset = 0;
    if ( !(str >> radius) ) {
       m_sc->error("Failed to parse robot radius, assuming radius=%f coffset=%f",radius,centerOffset);
      ret = false;
    } else {
      // Read the center offset if it is there
      str >> centerOffset;
      m_sc->error("Configured circular shaped robot\n");
    }
    setCircularShape(radius, centerOffset);
  } else if (type == 1) {
    double length = 0.5;
    double width = 0.45;
    double centerOffset = -0.06;
    if ( !(str >> length >> width >> centerOffset) ) {
      m_sc->error("Failed to parse rect shape params, using length=%f width=%f coffset=%f\n",length,width,centerOffset);
      ret = false;
    } else {
      m_sc->debug("Configured rectangular shaped robot\n");
    }
    setRectangularShape(length, width, centerOffset);
  } else if (type == 2) {
    int np;
    double length = 0.5;
    double width = 0.45;
    if ( !(str >> np) ) {
      m_sc->error("Could not even find number of verticies\n");
      setRectangularShape(length, width, 0);
      ret = false;
    } else {
      double lx[np], ly[np];
      for (int i = 0; i < np; i++) {
        if ( !(str >> lx[i] >> ly[i]) ) {
          m_sc->error("Failed to parse vertex %d\n",i);
          setRectangularShape(length, width, 0);
          ret = false;
          break;
        }
      }

      if (ret) {
        setPolygonShape(np, lx, ly);
      }
    }
  } else if (type == 3) {
    double E[n_];    
    bool failed = false;
    for (int i = 0; i < n_; i++) {
      if ( !(str >> E[i]) ) {
        m_sc->error( "Failed to get polar range shape param %d\n",i);
        m_sc->error("Using rectangular!!\n");
        failed = true;
        break;
      }
    }

    if (!failed) {
      setShapeFunction(E);
      m_sc->debug("Configured complex shaped robot\n");
    } else {        
      setRectangularShape(0.5, 0.45, -0.06);
    }

  } else {
    m_sc->error("Unknown shape type %d", type);
    ret = false;
  }

  m_sc->debug("frontDist_=%f backDist_=%f halfWidth_=%f\n",frontDist_, backDist_,halfWidth_);

  return ret;
}

void
NewTurnMotion::setCircularShape(double radius, double centerOffset)
{
  frontDist_ = 0.5 * radius + centerOffset;
  backDist_ = 0.5 * radius - centerOffset;
  halfWidth_ = 0.5 * radius;

  maxRadius_ = hypot(HelpFunctions::max(frontDist_, backDist_),
                     halfWidth_);

  MotionAlgorithm::setCircularShape(radius, centerOffset);
}

void
NewTurnMotion::setRectangularShape(double robotLength, double robotWidth, 
                                double centerOffset)
{
  frontDist_ = 0.5 * robotLength + centerOffset;
  backDist_ = 0.5 * robotLength - centerOffset;
  halfWidth_ = 0.5 * robotWidth;  

  maxRadius_ = hypot(HelpFunctions::max(frontDist_, backDist_),
                     halfWidth_);

  MotionAlgorithm::setRectangularShape(robotLength, robotWidth, centerOffset);
}

void
NewTurnMotion::setPolygonShape(int np, double *lx, double *ly) {

  MotionAlgorithm::setPolygonShape(np, lx, ly);

  double minX =  1e10;
  double maxX = -1e10;
  double minY =  1e10;
  double maxY = -1e10;

  // Loop over the sectors
  for (int i = 0; i < n_; i++) {
    double ang = M_PI - 2.0*M_PI*i/n_;
    double x = E_[i] * cos(ang);
    double y = E_[i] * sin(ang);

    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  
  frontDist_ = maxX;
  backDist_ = -minX;
  halfWidth_ = HelpFunctions::max(maxY, fabs(minY));

  maxRadius_ = hypot(HelpFunctions::max(frontDist_, backDist_),
                     halfWidth_);
}

void 
NewTurnMotion::setShapeFunction(double *E)
{
  memcpy(E_, E, sizeof(double) * n_);

  double minX =  1e10;
  double maxX = -1e10;
  double minY =  1e10;
  double maxY = -1e10;

  for (int i = 0; i < n_; i++) {
    double ang = M_PI - 2.0*M_PI*i/n_;
    double x = E_[i] * cos(ang);
    double y = E_[i] * sin(ang);

        if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }

  frontDist_ = maxX;
  backDist_ = -minX;
  halfWidth_ = HelpFunctions::max(maxY, fabs(minY));

  maxRadius_ = hypot(HelpFunctions::max(frontDist_, backDist_),
                     halfWidth_);
}

int
NewTurnMotion::getSector(double a)
{
  const double twopi = 2.0 * M_PI;
  while (a <= -M_PI) a += twopi;
  while (a >= M_PI) a -= twopi;

  int s ;
  if (a > 0) {
    s = nHalf_ - int(a * sectPerRad_ + 0.5);
  } else {
    s = nHalf_ + int(-a * sectPerRad_ + 0.5);
    if (s >= n_) s -= n_;
  }

  return s;
}

void
NewTurnMotion::displayRL(RoboLookProxy *rlp)
{
  if (rlp == NULL) return;

  rlp->clearMarkers(RL_ENV_TRUE);

  RL_MarkerItem obst[nClosePts_];
  for (int i = 0; i < nClosePts_; i++) {
    obst[i].x = closePts_[i].o->x;
    obst[i].y = closePts_[i].o->y;
    obst[i].z = 0;
    obst[i].w = 0.03;
    obst[i].color = 1;
    obst[i].floor = 0;
  }
  rlp->addMarkers(RL_ENV_TRUE, obst, nClosePts_);

  for (int i = 0; i < nViolationPts_; i++) {
    obst[i].x = violationPts_[i].o->x;
    obst[i].y = violationPts_[i].o->y;
    obst[i].z = 0;
    obst[i].w = 0.03;
    obst[i].color = 3;
    obst[i].floor = 0;
  }
  rlp->addMarkers(RL_ENV_TRUE, obst, nViolationPts_);  

  /*
  RL_MarkerItem perif[n_];
  for (int i = 0; i < n_; i++) {
    perif[i].x = robX_ + E_[i] * cos(robA_ + M_PI - 2.0 * M_PI / n_ * i);
    perif[i].y = robY_ + E_[i] * sin(robA_ + M_PI - 2.0 * M_PI / n_ * i);
    perif[i].z = 0;
    perif[i].w = 0.01;
    perif[i].color = 7;
    perif[i].floor = 0;
  }
  rlp->addMarkers(RL_ENV_TRUE, perif, n_);
  */
}
