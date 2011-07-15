/**
 * Provides 3D poses of cameras.
 * Poses can change if cameras are mounted on pan-tilt unit or arm.
 *
 * Michael Zillich
 * Oct 2009
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

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

static bool equals(ptz::PTZReading &a, ptz::PTZReading &b, double eps)
{
  return equals(a.pose.pan, b.pose.pan, eps) && equals(a.pose.tilt, b.pose.tilt, eps);
}

double CameraMount::FIXED_POSITION_TOLERANCE = 0.015;

CameraMount::CameraMount()
{
  usePTZ = false;
  fixedPanTilt.pose.pan = 0.;
  fixedPanTilt.pose.tilt = 0.;
  fixedPanTilt.pose.zoom = 0.;
  setIdentity(ptBasePose);
  setIdentity(ptPanPose);
  setIdentity(ptTiltPose);
  ptzServerComponent = "";
}

void CameraMount::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;
  bool have_fixed_pan_tilt = false;

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if((it = _config.find("--pt_base_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptBasePose);
    usePTZ = true;
  }

  if((it = _config.find("--pt_pan_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptPanPose);
    usePTZ = true;
  }

  if((it = _config.find("--pt_tilt_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptTiltPose);
    usePTZ = true;
  }

  if((it = _config.find("--cam_poses_xml")) != _config.end())
  {
    istringstream str(it->second);
    string filename;
    while(str >> filename)
    {
      Pose3 pose;
      readXML(filename, pose);
      camPoses.push_back(pose);
    }
    usePTZ = true;
  }

  if((it = _config.find("--fixed_cam_poses_xml")) != _config.end())
  {
    istringstream str(it->second);
    string filename;
    while(str >> filename)
    {
      Pose3 pose;
      readXML(filename, pose);
      camFixedPoses.push_back(pose);
    }
  }

  if((it = _config.find("--fixed_pan_tilt")) != _config.end())
  {
    istringstream str(it->second);
    str >> fixedPanTilt.pose.pan >> fixedPanTilt.pose.tilt;
    have_fixed_pan_tilt = true;
  }

  if((it = _config.find("--ptzserver")) != _config.end())
  {
    ptzServerComponent = it->second;
  }

  if(camIds.size() == 0)
    throw runtime_error("no cam IDs given, need at least one");
  if(camPoses.size() != 0 && camIds.size() != camPoses.size())
    throw runtime_error("number of camera IDs must match number of camera poses");
  if(camFixedPoses.size() != 0 && camIds.size() != camFixedPoses.size())
    throw runtime_error("number of camera IDs must match number of camera fixed poses");
  if(!usePTZ && camFixedPoses.size() == 0)
    throw runtime_error("if you are not using the PTZ you must supply fixed camera poses");
  if(camFixedPoses.size() != 0 && !have_fixed_pan_tilt)
    throw runtime_error("if you supply fixed camera poses you also must supply a fixed pan/tilt position");
}

void CameraMount::start()
{
  if(usePTZ)
  {
    std::string ptzServerHost = "localhost";
    int ptzServerPort = cast::cdl::CPPSERVERPORT;

    if (ptzServerComponent.length() > 0) {
      cast::cdl::ComponentDescription desc =
        getComponentManager()->getComponentDescription(ptzServerComponent);

      ptzServerHost = desc.hostName;
      ptzServerPort = cast::languageToPort(desc.language);
    }

    Ice::CommunicatorPtr ic = getCommunicator();

    Ice::Identity id;
    id.name = "PTZServer";
    id.category = "PTZServer";

    std::ostringstream str;
    str << ic->identityToString(id) 
      << ":default"
      << " -h " << ptzServerHost
      << " -p " << ptzServerPort;

    Ice::ObjectPrx base = ic->stringToProxy(str.str());    
    m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);
  }
}

void CameraMount::runComponent()
{
  int size = camIds.size();
  bool camsAddedToWM[size];
    for(unsigned i=0; i<size; i++)
      camsAddedToWM[i] = false;
  camWMIds.resize(camIds.size());
  for(size_t i = 0; i < camIds.size(); i++)
    camWMIds[i] = newDataID();
  while(isRunning())
  {
    vector<Pose3> camPosesToEgo(camIds.size());
    cdl::CASTTime time;
    if(usePTZ)
    {
      ptz::PTZReading ptz = m_PTUServer->getPose();
      if(camFixedPoses.size() != 0 && equals(ptz, fixedPanTilt, FIXED_POSITION_TOLERANCE))
      {
        camPosesToEgo = camFixedPoses;
      }
      else
      {
        calculatePoses(ptz, camPosesToEgo);
      }
      time = ptz.time;
    }
    else
    {
      camPosesToEgo = camFixedPoses;
      time = getCASTTime();
    }
    for(size_t i = 0; i < camIds.size(); i++)
    {
      CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
      initCameraParameters(camParms->cam);
      // only fill pose and time (let video server fill the rest)
      // we only know about poses
      camParms->cam.id = camIds[i];
      camParms->cam.pose = camPosesToEgo[i];
      camParms->cam.time = time;
      if(camsAddedToWM[i])
      {
        overwriteWorkingMemory(camWMIds[i], camParms);
      }
      else
      {
        addToWorkingMemory(camWMIds[i], camParms);
        camsAddedToWM[i] = true;
      }
    }
    // HACK: should get rid of need for sleep
    // We will hardly be moving the pan tilt unit at first, so long update
    // interval should not be a problem.
    sleepComponent(1000);
  }
}

void CameraMount::calculatePoses(ptz::PTZReading &ptz, vector<Pose3> &camPosesToEgo)
{
  // robot ego system: x points forward, y points to left wheel, z points up
  // kinematic chain of poses: robot ego -> pt zero pose -> pan -> tilt ->
  // camera
  Pose3 panRot, tiltRot;
  vector<Pose3> poses;

  setZero(panRot.pos);
  fromRotZ(panRot.rot, ptz.pose.pan);
  // note: positive tilt angle is tilting "up" which is negative rotation around
  // y axis -> need minus
  setZero(tiltRot.pos);
  fromRotY(tiltRot.rot, -ptz.pose.tilt);

  poses.resize(camPoses.size());
  camPosesToEgo.resize(camPoses.size());
  for(size_t i = 0; i < camPoses.size(); i++)
  {
    transform(tiltRot, camPoses[i], poses[i]);
    transform(ptTiltPose, poses[i], poses[i]);
    transform(panRot, poses[i], poses[i]);
    transform(ptPanPose, poses[i], poses[i]);
    transform(ptBasePose, poses[i], camPosesToEgo[i]);
  }
}

