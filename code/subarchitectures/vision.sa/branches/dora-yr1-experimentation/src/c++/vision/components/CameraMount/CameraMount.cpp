/**
 * Provides 3D poses of cameras.
 * Poses can change if cameras are mounted on pan-tilt unit or arm.
 *
 * Michael Zillich
 * Oct 2009
 */

#include "cogxmath.h"
#include "Video.hpp"
#include "VideoUtils.h"
#include "CameraMount.hpp"

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
    return new CameraMount();
  }
}

void CameraMount::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--pt_zero_pose")) != _config.end())
  {
    istringstream str(it->second);
    str >> ptZeroPose;
  }

  if((it = _config.find("--cam_ids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if((it = _config.find("--cam_poses")) != _config.end())
  {
    Pose3 pose;
    istringstream str(it->second);
    for(size_t i = 0; i < camIds.size(); i++)
    {
      str >> pose;
      camPoses.push_back(pose);
    }
  }
}

void CameraMount::start()
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
}

void CameraMount::runComponent()
{
  camWMIds.resize(camPoses.size());
  for(size_t i = 0; i < camPoses.size(); i++)
  {
    CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
    initCameraParameters(camParms->cam);
    // only fill pose and time (let video server fill the rest)
    // we only know about poses
    camParms->cam.id = camIds[i];
    camParms->cam.pose = camPoses[i];
    camParms->cam.time = getCASTTime();
    camWMIds[i] = newDataID();
    addToWorkingMemory(camWMIds[i], camParms);
  }

  while(isRunning())
  {
    vector<Pose3> camPosesToEgo;
	  ptz::PTZReading ptz = m_PTUServer->getPose();
    calculatePoses(ptz, camPosesToEgo);
    for(size_t i = 0; i < camPoses.size(); i++)
    {
      CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
      initCameraParameters(camParms->cam);
      // only fill pose and time (let video server fill the rest)
      // we only know about poses
      camParms->cam.id = camIds[i];
      camParms->cam.pose = camPosesToEgo[i];
      camParms->cam.time = ptz.time;
      overwriteWorkingMemory(camWMIds[i], camParms);
    }
    // HACK: should get rid of need for sleep
    // We will hardly be moving the pan tilt unit at first, so long update
    // interval should not be a problem.
    sleepComponent(1000);
  }
}

void CameraMount::calculatePoses(ptz::PTZReading &ptz, vector<Pose3> &poses)
{
  // robot ego system: x points forward, y points to left wheel, z points up
  // kinematic chain of poses: robot ego -> pt zero pose -> pan -> tilt ->
  // camera
  // pan and tilt are assumed to rotate around origin of pan-tilt unit (which is
  // not entirely true in practice)
  Pose3 posePan, poseTilt;
  setZero(posePan.pos);
  fromRotZ(posePan.rot, ptz.pose.pan);
  // note: positive tilt angle is tilting "up" which is negative rotation around
  // y axis -> need minus
  setZero(poseTilt.pos);
  fromRotY(poseTilt.rot, -ptz.pose.tilt);

  poses.resize(camPoses.size());
  for(size_t i = 0; i < camPoses.size(); i++)
  {
    transform(poseTilt, camPoses[i], poses[i]);
    transform(posePan, poses[i], poses[i]);
    transform(ptZeroPose, poses[i], poses[i]);
  }
}

