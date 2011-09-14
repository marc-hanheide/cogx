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
using namespace ptz;

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
  ref_cam_id = -1;
}

void CameraMount::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;
  bool have_fixed_pan_tilt = false;
  // NOTE: OpenCV calibration procedures return the inverse poses, e.g. the pose
  // of the calibration pattern (=world) to the camera, while we need the camera
  // to world poses. Use this flag if you suspect that to be the case.
  // Unfortunately there is no single clear convention.
  bool invert_poses = false;

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

  if((it = _config.find("--reference_cam_id")) != _config.end())
  {
    istringstream str(it->second);
    str >> ref_cam_id;
  }

  if((it = _config.find("--invert_poses")) != _config.end())
  {
    invert_poses = true;
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
  if(camPoses.size() == 0 && camFixedPoses.size() == 0)
    throw runtime_error("you must provide either --cam_poses_xml or --cam_fixed_poses_xml");
  if(!usePTZ && camFixedPoses.size() == 0)
    throw runtime_error("if you are not using the PTZ you must supply fixed camera poses");
  if(camFixedPoses.size() != 0 && !have_fixed_pan_tilt)
    throw runtime_error("if you supply fixed camera poses you also must supply a fixed pan/tilt position");

  if(invert_poses)
  {
    for(size_t i = 0; i < camPoses.size(); i++)
      inverse(camPoses[i], camPoses[i]);
    for(size_t i = 0; i < camFixedPoses.size(); i++)
      inverse(camFixedPoses[i], camFixedPoses[i]);
  }
}

void CameraMount::start()
{
  if(usePTZ)
  {
    m_PTUServer = getIceServer<PTZInterface>(ptzServerComponent);
  }
}

void CameraMount::runComponent()
{
  size_t size = camIds.size();
  bool camsAddedToWM[size];

  for(size_t i = 0; i < size; i++)
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
        calculateFixedPoses(camPosesToEgo);
      else
        calculatePoses(ptz, camPosesToEgo);
      time = ptz.time;
    }
    else
    {
      calculateFixedPoses(camPosesToEgo);
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
    sleepComponent(200);
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
    // if my pose is relative to the reference cam
    if(ref_cam_id != -1 && ref_cam_id != camIds[i])
      transform(camPoses[ref_cam_id], camPoses[i], poses[i]);
    else
      poses[i] = camPoses[i];
    transform(tiltRot, poses[i], poses[i]);
    transform(ptTiltPose, poses[i], poses[i]);
    transform(panRot, poses[i], poses[i]);
    transform(ptPanPose, poses[i], poses[i]);
    transform(ptBasePose, poses[i], camPosesToEgo[i]);
  }
}

void CameraMount::calculateFixedPoses(std::vector<cogx::Math::Pose3> &camPosesToEgo)
{
  camPosesToEgo.resize(camFixedPoses.size());
  for(size_t i = 0; i < camFixedPoses.size(); i++)
  {
    // if my pose is relative to the reference cam
    if(ref_cam_id != -1 && ref_cam_id != camIds[i])
      transform(camFixedPoses[ref_cam_id], camFixedPoses[i], camPosesToEgo[i]);
    else
      camPosesToEgo[i] = camFixedPoses[i];
  }
}


