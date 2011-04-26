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

void CameraMount::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  setIdentity(ptZeroPose);
  if((it = _config.find("--pt_zero_pose")) != _config.end())
  {
    istringstream str(it->second);
    str >> ptZeroPose;
  }
  if((it = _config.find("--pt_zero_pose_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptZeroPose);
  }
  if((it = _config.find("--pt_base_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptBasePose);
  }
  if((it = _config.find("--pt_pan_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptPanPose);
  }
  if((it = _config.find("--pt_tilt_xml")) != _config.end())
  {
    string filename = it->second;
    readXML(filename, ptTiltPose);
  }

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if((it = _config.find("--cam_poses")) != _config.end())
  {
    istringstream str(it->second);
    for(size_t i = 0; i < camIds.size(); i++)
    {
      Pose3 pose;
      str >> pose;
      camPoses.push_back(pose);
    }
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
      log("read pose:\n" + toString(pose));
    }
  }

  isFixed = false;
  if((it = _config.find("--fixed")) != _config.end())
  {
    isFixed = true;
  }

  if(camIds.size() != camPoses.size())
    throw runtime_error("number of camera IDs must match number of camera poses");
}

void CameraMount::start()
{
  if(!isFixed)
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
}

void CameraMount::runComponent()
{
  if(!isFixed)
  {
    log("using PTU");
    vector<Pose3> camPosesToEgo;
    ptz::PTZReading ptz;

    // initial pose: pan, tilt = 0
    ptz.pose.pan = 0.;
    ptz.pose.tilt = 0.;
    ptz.pose.zoom = 0.;
    ptz.time = getCASTTime();
    calculatePoses(ptz, camPosesToEgo);

    // add camera parameters to WM, with initial poses
    camWMIds.resize(camPoses.size());
    for(size_t i = 0; i < camPoses.size(); i++)
    {
      CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
      initCameraParameters(camParms->cam);
      // only fill pose and time (let video server fill the rest)
      // we only know about poses
      camParms->cam.id = camIds[i];
      camParms->cam.pose = camPosesToEgo[i];
      camParms->cam.time = ptz.time;
      camWMIds[i] = newDataID();
      addToWorkingMemory(camWMIds[i], camParms);
    }

    // continuosly (with a nasty sleep) get pan, tilt values and
    // update poses
    while(isRunning())
    {
	    ptz = m_PTUServer->getPose();
      calculatePoses(ptz, camPosesToEgo);
      for(size_t i = 0; i < camPoses.size(); i++)
      {
        CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
        initCameraParameters(camParms->cam);
        // only fill pose and time (let video server fill the rest)
        // we only know about poses
        camParms->cam.id = camIds[i];
        camParms->cam.pose = camPosesToEgo[i];
        log("cam pose in robot ego:" + toString(camParms->cam.pose));
        camParms->cam.time = ptz.time;
        overwriteWorkingMemory(camWMIds[i], camParms);
      }
      // HACK: should get rid of need for sleep
      // We will hardly be moving the pan tilt unit at first, so long update
      // interval should not be a problem.
      sleepComponent(1000);
    }
  }
  else
  {
    log("using fixed cam position and no PTU");
    // add camera parameters to WM, with initial poses
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

    // continuosly (with a nasty sleep) update poses
    while(isRunning())
    {
      for(size_t i = 0; i < camPoses.size(); i++)
      {
        CameraParametersWrapperPtr camParms = new CameraParametersWrapper;
        initCameraParameters(camParms->cam);
        // only fill pose and time (let video server fill the rest)
        // we only know about poses
        camParms->cam.id = camIds[i];
        camParms->cam.pose = camPoses[i];
        log("cam pose in robot ego:" + toString(camParms->cam.pose));
        camParms->cam.time = getCASTTime();
        overwriteWorkingMemory(camWMIds[i], camParms);
      }
      // HACK: should get rid of need for sleep
      sleepComponent(1000);
    }
  }
}

void CameraMount::calculatePoses(ptz::PTZReading &ptz, vector<Pose3> &poses)
{
  // robot ego system: x points forward, y points to left wheel, z points up
  // kinematic chain of poses: robot ego -> pt zero pose -> pan -> tilt ->
  // camera
  Pose3 panRot, tiltRot;
  setZero(panRot.pos);
  fromRotZ(panRot.rot, -ptz.pose.pan);
  // note: positive tilt angle is tilting "up" which is negative rotation around
  // y axis -> need minus
  setZero(tiltRot.pos);
  fromRotY(tiltRot.rot, -ptz.pose.tilt);

  poses.resize(camPoses.size());
  for(size_t i = 0; i < camPoses.size(); i++)
  {
    transform(tiltRot, camPoses[i], poses[i]);
    transform(ptTiltPose, poses[i], poses[i]);
    transform(panRot, poses[i], poses[i]);
    transform(ptPanPose, poses[i], poses[i]);
    transform(ptBasePose, poses[i], poses[i]);
    
    log(toString(poses[i]));
    double r, p, y;
    toRPY(poses[i].rot, r, p, y);
    log("RPY: %f %f %f", r, p, y);
  }
}

