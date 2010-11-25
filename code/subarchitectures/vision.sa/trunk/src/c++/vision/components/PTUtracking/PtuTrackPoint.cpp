
#include "cogxmath.h"
#include "Video.hpp"
#include "VideoUtils.h"
#include "PtuTrackPoint.hpp"

//#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/architecture.hpp>
#include <math.h>


using namespace boost;

using namespace std;
using namespace cogx::Math;
using namespace Video;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new PtuTrackPoint();
  }
}

void PtuTrackPoint::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--pan")) != _config.end())
  {
    istringstream str(it->second);
    str >> pan;
  }

  if((it = _config.find("--tilt")) != _config.end())
  {
    istringstream str(it->second);
    str >> tilt;
  }
  
  if((it = _config.find("--tX")) != _config.end())
  {
    istringstream str(it->second);
    str >> x;
  }
  
  if((it = _config.find("--tY")) != _config.end())
  {
    istringstream str(it->second);
    str >> y;
  }
  
  if((it = _config.find("--tZ")) != _config.end())
  {
    istringstream str(it->second);
    str >> z;
  }
}

void PtuTrackPoint::start()
{
  Ice::CommunicatorPtr ic = getCommunicator();

  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
    << ":default"
    << " -h localhost"
    << " -p " << cast::cdl::CPPSERVERPORT;

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);
  
  
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                  new MemberFunctionChangeReceiver<PtuTrackPoint>(this,
                                         &PtuTrackPoint::newRobotPose));
  
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<PtuTrackPoint>(this,
                                         &PtuTrackPoint::newRobotPose));
}


void PtuTrackPoint::runComponent()
{
	//MovePanTilt(pan, tilt, 5*M_PI/180);
}


void PtuTrackPoint::newRobotPose(const cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::RobotPose2d> > oobj = 
              getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
    
  Cure::Pose3D cp;
  cp.setTime(Cure::Timestamp(oobj->getData()->time.s, 
                             oobj->getData()->time.us));
  cp.setX(oobj->getData()->x);
  cp.setY(oobj->getData()->y);
  cp.setTheta(oobj->getData()->theta);
  
  if (cp.getTime().getDouble() <= 0) {
    log("RobotPose not ready yet, time still 0");
    return;
  }
  
  /*    
  println("ROBOTPOSE: x=%.3f y=%.3f theta=%.3f t=%.6f",
        cp.getX(), cp.getY(), cp.getTheta(), cp.getTime().getDouble());
  */
  
  trackTargetPoint(cp);
}

void PtuTrackPoint::trackTargetPoint(Cure::Pose3D cp)
{
  T_R0x = x-cp.getX();
  T_R0y = y-cp.getY();
  T_Rx = cos(cp.getTheta())*T_R0x + sin(cp.getTheta())*T_R0y;
  T_Ry = -sin(cp.getTheta())*T_R0x + cos(cp.getTheta())*T_R0y;
  phi_R = atan2(T_Ry,T_Rx);
  phi_R_abs = fabs(phi_R);
  T_R_abs=sqrt(pow(T_Rx,2)+pow(T_Ry,2));
  //0.11=distance of ptu frame to world frame along the x direction of robot frame
  //0.0121 = 0.11^2,   0.22=2*0.11
  TD = sqrt(pow(T_R_abs,2) + 0.0121 - 0.22*T_R_abs*cos(phi_R_abs));
  sin_alpha = T_R_abs*sin(phi_R_abs)/TD;
  
  if(T_Rx>=0.11)
  {
    pan = asin(sin_alpha);
  }
  else
  {
    pan = M_PI-asin(sin_alpha);
  }
  
  if(phi_R<0)
  {
    pan = -pan;
  }
  
  T_PD_abs = sqrt(pow(T_Rx-0.11,2)+pow(T_Ry,2));
  c = sqrt(pow(T_PD_abs,2)+pow(1.37-z,2));
  beta = acos(0.075/c);  //0.075=distance from tilt axis to cameras
  
  if(z<=1.37)//tilt<0, 1.37=distance from world frame to tilt axis along z direction
  {
    gamma = asin(T_PD_abs/c);
    tilt = -(M_PI-beta-gamma);
  }
  else if(z>1.37 && z<=1.445)//tilt<=0
  {
    gamma = acos(T_PD_abs/c);
    tilt = -(M_PI/2-beta-gamma);
  }
  else//tilt>0
  {   
    gamma = acos(T_PD_abs/c);
    gamma2 = M_PI/2 - gamma;
    tilt = beta-gamma2;
  }
    
  ptz::PTZPose p;
  p.pan = pan ;
  p.tilt = tilt;
  p.zoom = 0;
  m_PTUServer->setPose(p);
}

void PtuTrackPoint::setTargetPoint(double tX, double tY, double tZ)
{
  x=tX;
  y=tY;
  z=tZ;
}


void PtuTrackPoint::MovePanTilt(double pan, double tilt, double tolerance)
{
	log("Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
  ptz::PTZPose p;
  ptz::PTZReading ptuPose;
  p.pan = pan ;
  p.tilt = tilt;
  p.zoom = 0;
  m_PTUServer->setPose(p);
  bool run = true;
  ptuPose = m_PTUServer->getPose();
  double actualpose = ptuPose.pose.pan;
  while(run){
    m_PTUServer->setPose(p);
    ptuPose = m_PTUServer->getPose();
    actualpose = ptuPose.pose.pan;
    log("actualpose is: %f", actualpose);
    if (pan > actualpose){
      if (actualpose > abs(pan) - tolerance){
        log("false actualpose is: %f, %f", actualpose, abs(pan) + tolerance);
        run = false;
      }
        }
    if (actualpose > pan){
      if (actualpose < abs(pan) + tolerance)
        run = false;
        }
      if(pan == actualpose)
      run = false;
    
     //usleep(10);
  }
  log("Moved.");
}
