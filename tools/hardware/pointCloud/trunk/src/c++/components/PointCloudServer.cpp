/**
 * @file PointCloudServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the cast-framework
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <limits.h>
#include <sstream>
#include <opencv/highgui.h>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "Video.hpp"
#include "PTZ.hpp"

#include "PointCloudServer.h"

// a useful default max disparity
#define DEF_MAX_DISP 64

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PointCloudServer();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;


// #ifdef __APPLE__ //nah: OS X doesn't have CLOCK_MONOTONIC

// static double gethrtime_d()
// {
// 	timeval tv;
// 	int ret = gettimeofday(&tv, NULL);  
// 	if(ret != 0) {
//     	return 0;
// 	}
// 	return (double)tv.tv_sec + 1e-6*(double)tv.tv_usec;
// }


// #else

// static double gethrtime_d()
// {
//   struct timespec ts;
//   int ret;
// #ifdef CLOCK_MONOTONIC_HR
//   ret = clock_gettime(CLOCK_MONOTONIC_HR, &ts);
// #else
//   ret = clock_gettime(CLOCK_MONOTONIC, &ts);
// #endif
//   if(ret != 0)
//     return 0;
//   return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;
// }

// #endif

/// ************** Point Cloud Server Interface ************** ///
void PointCloudServerI::getPoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points, const Ice::Current&)
{
  ptCloudSrv->getPoints(transformToGlobal, imgWidth, points, false);
}

void PointCloudServerI::getCompletePoints(bool transformToGlobal, int imgWidth, PointCloud::SurfacePointSeq& points, const Ice::Current&)
{
  ptCloudSrv->getPoints(transformToGlobal, imgWidth, points, true);
}

void PointCloudServerI::getRectImage(Ice::Int camId, int imgWidth, Video::Image& image, const Ice::Current&)
{
  ptCloudSrv->getRectImage(camId, imgWidth, image);
}

void PointCloudServerI::getDisparityImage(int imgWidth, Video::Image& image, const Ice::Current&)
{
  ptCloudSrv->getDisparityImage(imgWidth, image);
}

void PointCloudServerI::getDepthMap(cast::cdl::CASTTime &time, vector<int>& data, const Ice::Current&)
{
  ptCloudSrv->getDepthMap(time, data);
}

bool PointCloudServerI::getCameraParameters(Ice::Int camId, Video::CameraParameters& cam, const Ice::Current&)
{
  return ptCloudSrv->getCameraParameters(camId, cam);
}

bool PointCloudServerI::isPointInViewCone(const cogx::Math::Vector3& point, const Ice::Current&)
{
  return ptCloudSrv->isPointInViewCone(point);
}

// @author: mmarko
bool PointCloudServerI::isPointVisible(const cogx::Math::Vector3& point, const Ice::Current&)
{
  return ptCloudSrv->isPointVisible(point);
}

void PointCloudServerI::getRangePoints(Laser::Scan2d &KRdata, const Ice::Current&)
{
  ptCloudSrv->getRangePoints(KRdata);
}

/// ************** Point Cloud Server ************** ///
PointCloudServer::PointCloudServer()
{}

PointCloudServer::~PointCloudServer()
{}

void PointCloudServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  // first let the base class configure itself
  CASTComponent::configure(_config);
  
  map<string,string>::const_iterator it;
    
  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  } else log("configure: Warning: No 'camids' specified!\n");
  
  // note: it is ok to not specify these, defaults will be chosen in that case
  if((it = _config.find("--camconfigs")) != _config.end())
  {
    istringstream str(it->second);
    string file;
    while(str >> file)
    {
      Video::CameraParameters pars;
      // calibration files can be either monocular calibration files (e.g.
      // "flea0.cal", "flea1.cal") or SVS stereo calib files (e.g.
      // "stereo.ini:L", "stereo.ini:R"). Note that in the latter case the
      // actual file name is extended with a ":" (to indicate we have a stereo
      // case) and a L or R indicating we refer to the LEFT or RIGHT camera of
      // the stereo rig.
      if(file.find(":") == string::npos)
      {
        // monocular files can be either .cal (INI style) or .xml (from OpenCV)
        if(file.find(".xml") == string::npos)
          loadCameraParameters(pars, file);
        else
          loadCameraParametersXML(pars, file);
      }
      else
      {
        // stereo case
        size_t pos = file.find(":");
        if(pos >= file.size() - 1)
          throw runtime_error(exceptionMessage(__HERE__,
                "please indicate camera L or R after ':' in config '%s'", file.c_str()));
        char side = file[pos + 1];
        string pure_filename(file, 0, pos);
        if(side == 'L')
          loadCameraParametersFromSVSCalib(pars, pure_filename, LEFT);
        else if(side == 'R')
          loadCameraParametersFromSVSCalib(pars, pure_filename, RIGHT);
        else
          throw runtime_error(exceptionMessage(__HERE__,
                "camera '%c' invalid in config '%s', must be either :L or :R",
                side, file.c_str()));
      }
      camPars.push_back(pars);
    }
  }
  
  // in case no camera config files were given, assume default configs
  if(camPars.size() == 0)
  {
    log("configure: Warning: No 'camconfigs' specified!");
    camPars.resize(camIds.size());
    for(size_t i = 0; i < camPars.size(); i++)
      initCameraParameters(camPars[i]);
  }
  
  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != camPars.size())
    throw runtime_error(exceptionMessage(__HERE__, "numbers of camera IDs %d and camera config files %d do not match",
                                         (int) camIds.size(), (int) camPars.size()));
  if(camIds.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no camera IDs given"));

  // fill camIds and current time stamp into cam parameters
  // TODO: avoid this double ids at some point
  for(size_t i = 0; i < camPars.size(); i++)
  {
    camPars[i].id = camIds[i];
    camPars[i].time = getCASTTime();
  }
  
  hPointCloudServer = new PointCloudServerI(this);
  registerIceServer<PointCloud::PointCloudInterface, PointCloudServerI>(hPointCloudServer);
}

void PointCloudServer::start()
{
  suspendReading=false;
  // add change filter for camera mount
  addChangeFilter(createLocalTypeFilter<Video::CameraParametersWrapper>(cdl::ADD),
      new MemberFunctionChangeReceiver<PointCloudServer>(this, &PointCloudServer::receiveCameraParameters));

  addChangeFilter(createLocalTypeFilter<Video::CameraParametersWrapper>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PointCloudServer>(this, &PointCloudServer::receiveCameraParameters));

  addChangeFilter(createTypeFilter<ptz::SetPTZPoseCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<PointCloudServer>(this, &PointCloudServer::receivePTZCommand));

  addChangeFilter(createTypeFilter<ptz::SetPTZPoseCommand>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PointCloudServer>(this, &PointCloudServer::receivePTZCommand));

  addChangeFilter(createTypeFilter<ptz::SetPTZPoseCommand>(cdl::DELETE),
      new MemberFunctionChangeReceiver<PointCloudServer>(this, &PointCloudServer::receivePTZCommand));
}

void PointCloudServer::receivePTZCommand(const cdl::WorkingMemoryChange & _wmc)
{
  try {
    ptz::SetPTZPoseCommandPtr ptzCmd = getMemoryEntry<ptz::SetPTZPoseCommand>(_wmc.address);
    if (_wmc.operation == cdl::DELETE) {
      println("reading is working again (after deletion)");
      suspendReading=false;
    } else {
      if (ptzCmd->comp==ptz::COMPINIT) {
        println("reading is suspended because PTZ is moving");
        suspendReading=true;
      } else {
        println("reading is working again (after status update)");
        suspendReading=false;
      }
    }
  } catch (const CASTException& e) {
        println("reading is working again (after exception)");
        suspendReading=false;
  }
}


/**
 * @brief Receive camera parameters from the camera mount
 */
void PointCloudServer::receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc)
{
//   log("received camera parameters\n");
  Video::CameraParametersWrapperPtr newCam = getMemoryEntry<Video::CameraParametersWrapper>(_wmc.address);
  // find the camera paramters that need updating and update pose and time stamp
  // Note that we don't change any other (instrinsic) parameters yet as we
  // assume these fixed. At a later stage (with zoom cameras) also the intrinsic
  // parameters might change.
  // Note: if we don't find a camera with matching id in our list, no problem,
  // then these parameters were meant for another video server.
  for(size_t i = 0; i < camPars.size(); i++)
  {
    if(newCam->cam.id == camPars[i].id)
    {
      camPars[i].pose = newCam->cam.pose;
      camPars[i].time = newCam->cam.time;
//       log("transformToGlobal with pose.pos: %4.4f / %4.4f / %4.4f\n",  camPars[i].pose.pos.x,  camPars[i].pose.pos.y,  camPars[i].pose.pos.z);
      break;
    }
  }
}


// @author: mmarko
bool PointCloudServer::isPointVisible(const cogx::Math::Vector3& point)
{
  if (camPars.size() < 1) return false;
  for(size_t i = 0; i < camPars.size(); i++) {
    if (! Video::isPointVisible(camPars[i], point)) {
      //log("Point is NOT visible in camera %d", camPars[i].id);
      return false;
    }
    //log("Point is visible in camera %d", camPars[i].id);
  }
  return true;
}

}

