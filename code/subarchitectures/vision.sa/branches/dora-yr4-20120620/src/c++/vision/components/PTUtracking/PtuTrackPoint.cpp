#include "cogxmath.h"
#include "Video.hpp"
#include "VideoUtils.h"
#include "PtuTrackPoint.hpp"
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

  dontTrackSOIs = (_config.find("--dont-track-SOIs") != _config.end());
  if (dontTrackSOIs) log("Will NOT track SOIs");
  else log("Will track SOIs"); 

  trackFirstSOI = (_config.find("--track-first-SOI") != _config.end());
  if (trackFirstSOI) log("Will track only the first seen SOI");
  else log("Will track the last detected SOI"); 
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
  
  if(!dontTrackSOIs)
  {
  	addChangeFilter(createGlobalTypeFilter<VisionData::SOI>(cdl::ADD),
                  new MemberFunctionChangeReceiver<PtuTrackPoint>(this,
                                         &PtuTrackPoint::trackSOI));
  
  	addChangeFilter(createGlobalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<PtuTrackPoint>(this,
                                         &PtuTrackPoint::trackSOI));
  }
}


void PtuTrackPoint::runComponent()
{
	//MovePanTilt(pan, tilt, 5*M_PI/180);
}

void PtuTrackPoint::trackSOI(const cdl::WorkingMemoryChange &objID)
{
  println("Received SOI @ %s", objID.address.id.c_str());
  
  if(trackFirstSOI)
  {
	if(waitingForFirstSOI)
		{
			waitingForFirstSOI = false;
			firstSOIid = objID.address.id.c_str();
		}
	else if(strcmp(firstSOIid,objID.address.id.c_str())!=0) return; //strcmp() returns 0 if the strings are identical
  }

  shared_ptr<CASTData<VisionData::SOI> > oobj = 
              getWorkingMemoryEntry<VisionData::SOI>(objID.address);

  //VisionData::SOIPtr obs = new VisionData::SOI;
  //obs->boundingBox.pos.x = oobj->getData()->boundingBox.pos.x;
  //obs->boundingBox.pos.y = oobj->getData()->boundingBox.pos.y;
  //obs->boundingBox.pos.z = oobj->getData()->boundingBox.pos.z;
  
  /*
  println("SOI SEEN AT: x=%.3f y=%.3f z=%.3f",
        obs->boundingBox.pos.x, obs->boundingBox.pos.y, obs->boundingBox.pos.z);
  */

  //0.07=distance from camera's screw to it's lense
  //0.075=distance from tilt axis to cameras
  //0.11=distance of ptu frame to world frame along the x direction of robot frame
  //1.37=distance of ptu frame to world frame along the z direction of robot frame

  double x_k=oobj->getData()->boundingBox.pos.x;
  double y_k=oobj->getData()->boundingBox.pos.y;
  double z_k=oobj->getData()->boundingBox.pos.z;

  double x_ROB = 0.11 + cos(pan)*((0.07+z_k)*cos(tilt)-(0.075-y_k)*sin(tilt))+sin(pan)*x_k;
  double y_ROB = sin(pan)*((0.07+z_k)*cos(tilt)-(0.075-y_k)*sin(tilt))-cos(pan)*x_k;
  double z_ROB = 1.37 + (0.07+z_k)*sin(tilt)+(0.075-y_k)*cos(tilt);
  
  println("SOI in robot frame at: x=%.3f y=%.3f z=%.3f",
        x_ROB, y_ROB, z_ROB);

  double x_WORLD = cp.getX()+x_ROB*cos(cp.getTheta())-y_ROB*sin(cp.getTheta());
  double y_WORLD = cp.getY()+x_ROB*sin(cp.getTheta())+y_ROB*cos(cp.getTheta());
  //double z_WORLD = z_ROB;

  println("SOI in world frame at: x=%.3f y=%.3f z=%.3f",
        x_WORLD, y_WORLD, z_ROB);  

  println("pan=%.3f tilt=%.3f",
        pan, tilt);

  setTargetPoint(x_WORLD,y_WORLD,z_ROB);
  trackTargetPoint(cp);
}

void PtuTrackPoint::newRobotPose(const cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::RobotPose2d> > oobj = 
              getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
    
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
  double T_R0x = x-cp.getX();
  double T_R0y = y-cp.getY();
  double T_Rx = cos(cp.getTheta())*T_R0x + sin(cp.getTheta())*T_R0y;
  double T_Ry = -sin(cp.getTheta())*T_R0x + cos(cp.getTheta())*T_R0y;
  double phi_R = atan2(T_Ry,T_Rx);
  double phi_R_abs = fabs(phi_R);
  double T_R_abs=sqrt(pow(T_Rx,2)+pow(T_Ry,2));
  //0.11=distance of ptu frame to world frame along the x direction of robot frame
  //0.0121 = 0.11^2,   0.22=2*0.11
  double TD = sqrt(pow(T_R_abs,2) + 0.0121 - 0.22*T_R_abs*cos(phi_R_abs));
  double sin_alpha = T_R_abs*sin(phi_R_abs)/TD;
  
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
  
  double T_PD_abs = sqrt(pow(T_Rx-0.11,2)+pow(T_Ry,2));
  double c = sqrt(pow(T_PD_abs,2)+pow(1.37-z,2));
  double beta = acos(0.075/c);  //0.075=distance from tilt axis to cameras
  
  if(z<=1.37)//tilt<0, 1.37=distance from world frame to tilt axis along z direction
  {
    double gamma = asin(T_PD_abs/c);
    tilt = -(M_PI-beta-gamma);
  }
  else if(z>1.37 && z<=1.445)//tilt<=0
  {
    double gamma = acos(T_PD_abs/c);
    tilt = -(M_PI/2-beta-gamma);
  }
  else//tilt>0
  {   
    double gamma = acos(T_PD_abs/c);
    double gamma2 = M_PI/2 - gamma;
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
