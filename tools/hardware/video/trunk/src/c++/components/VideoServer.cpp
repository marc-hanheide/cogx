/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <sstream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cogxmath.h>
#include "VideoServer.h"

namespace cast
{

using namespace std;
using namespace cogx::Math;
using namespace Video;

Ice::Int VideoServerI::getNumCameras(const Ice::Current&)
{
  return (Ice::Int)vidSrv->getNumCameras();
}

void VideoServerI::getImageSize(Ice::Int& width, Ice::Int& height, const Ice::Current&)
{
  int w = 0, h = 0;
  vidSrv->getImageSize(w, h);
  width = (Ice::Int)w;
  height = (Ice::Int)h;
}

Ice::Int VideoServerI::getFramerateMilliSeconds(const Ice::Current&)
{
  return (Ice::Int)vidSrv->getFramerateMilliSeconds();
}

void VideoServerI::getImage(Ice::Int camId, Image& image, const Ice::Current&)
{
  vidSrv->getImage(camId, image);
}

void VideoServerI::getImages(ImageSeq& images, const Ice::Current&)
{
  vidSrv->getImages(images);
}

void VideoServerI::getScaledImages(Ice::Int width, Ice::Int height,
      ImageSeq& images, const Ice::Current&)
{
  vidSrv->getScaledImages(width, height, images);
}

void VideoServerI::startReceiveImages(const std::string& receiverComponentId,
    const vector<Ice::Int> &camIds, Ice::Int width, Ice::Int height,
    const Ice::Current&)
{
  vidSrv->startReceiveImages(receiverComponentId, camIds, width, height);
}

void VideoServerI::stopReceiveImages(const std::string& receiverComponentId,
    const Ice::Current&)
{
  vidSrv->stopReceiveImages(receiverComponentId);
}


/**
 * Configure options common to all video servers.
 */
void VideoServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  CASTComponent::configure(_config);

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  // note: it is ok to not specify these, defaults will be chosen in that case
  if((it = _config.find("--camconfigs")) != _config.end())
  {
    istringstream str(it->second);
    string file;
    while(str >> file)
    {
      CameraParameters pars;
      // calibration files can be either monocular calibration files (e.g.
      // "flea0.cal", "flea1.cal") or SVS stereo calib files (e.g.
      // "stereo.ini:L", "stereo.ini:R"). Note that in the latter case the
      // actual file name is extended with a ":" (to indicate we have a stereo
      // case) and a L or R indicating we refer to the LEFT or RIGHT camera of
      // the stereo rig.
      if(file.find(":") == string::npos)
      {
        // monocular case
        loadCameraParameters(pars, file);
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

  if((it = _config.find("--swapredblue")) != _config.end())
  {
    istringstream str(it->second);
    str >> boolalpha >> swapRB;
  }

  // in case no camera config files were given, assume default configs
  if(camPars.size() == 0)
  {
    camPars.resize(camIds.size());
    for(size_t i = 0; i < camPars.size(); i++)
      initCameraParameters(camPars[i]);
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != camPars.size())
    throw runtime_error(exceptionMessage(__HERE__,
      "numbers of camera IDs %d and camera config files %d do not match",
      (int)camIds.size(), (int)camPars.size()));
  if(camIds.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no camera IDs given"));

  // fill camIds and current time stamp into cam parameters
  // TODO: avoid this double ids at some point
  for(size_t i = 0; i < camPars.size(); i++)
  {
    camPars[i].id = camIds[i];
    camPars[i].time = getCASTTime();
  }

  // got all configuration into, now do actual configuration
  VideoInterfacePtr servant = new VideoServerI(this);
  registerIceServer<VideoInterface, VideoInterface>(servant);
}

void VideoServer::start()
{
  addChangeFilter(createLocalTypeFilter<CameraParametersWrapper>(cdl::ADD),
      new MemberFunctionChangeReceiver<VideoServer>(this,
        &VideoServer::receiveCameraParameters));

  addChangeFilter(createLocalTypeFilter<CameraParametersWrapper>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VideoServer>(this,
        &VideoServer::receiveCameraParameters));
}

void VideoServer::receiveCameraParameters(const cdl::WorkingMemoryChange &
    _wmc)
{
  CameraParametersWrapperPtr newCam = getMemoryEntry<CameraParametersWrapper>(_wmc.address);
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
      break;
    }
  }
}

void VideoServer::startReceiveImages(const std::string &receiverComponentId,
    const vector<int> &camIds, int width, int height)
{
  lockComponent();
  ImageReceiver rcv;
  rcv.imgWidth = width;
  rcv.imgHeight = height;
  rcv.camIds = camIds;
  rcv.receiverComponentId = receiverComponentId;
  rcv.videoClient = getIceServer<VideoClientInterface>(rcv.receiverComponentId);
  imageReceivers.push_back(rcv);
  unlockComponent();
}

void VideoServer::stopReceiveImages(const std::string &receiverComponentId)
{
  lockComponent();
  for(vector<ImageReceiver>::iterator i = imageReceivers.begin();
      i != imageReceivers.end(); i++)
  {
    if(i->receiverComponentId == receiverComponentId)
    {
      imageReceivers.erase(i);
      break;
    }
  }
  unlockComponent();
}

int VideoServer::getNumCameras() const
{
  return (int)camIds.size();
}

void VideoServer::getImage(int camId, Video::Image &img)
{
  lockComponent();
  // width and height = 0 means no resizing
  retrieveFrame((int)camId, 0, 0, img);
  if(swapRB)
    SwapRedBlueChannel(img);
  unlockComponent();
}

void VideoServer::getImages(std::vector<Video::Image> &images)
{
  lockComponent();
  // providing width = height = 0 means using native image size
  retrieveFrames(0, 0, images);
  if(swapRB)
    for(size_t i = 0; i < images.size(); i++)
      SwapRedBlueChannel(images[i]);
  unlockComponent();
}

void VideoServer::getScaledImages(int width, int height, std::vector<Video::Image> &images)
{
  lockComponent();
  retrieveFrames(width, height, images);
  if(swapRB)
    for(size_t i = 0; i < images.size(); i++)
      SwapRedBlueChannel(images[i]);
  unlockComponent();
}

void VideoServer::runComponent()
{
  vector<Image> frames;
  int cnt = 100;
  while(isRunning())
  {
    // TODO: If I could have this lock after grabFrames() I could avoid the
    // stupid sleep.
    lockComponent();
    grabFrames();
    for(size_t i = 0; i < imageReceivers.size(); i++)
    {
      retrieveFrames(imageReceivers[i].camIds,
        imageReceivers[i].imgWidth, imageReceivers[i].imgHeight, frames);
      if(swapRB)
        for(size_t i = 0; i < frames.size(); i++)
          SwapRedBlueChannel(frames[i]);
      imageReceivers[i].videoClient->receiveImages(frames);
    }
    unlockComponent();
    // HACK: to let getImages() have chance to lockComponent()
    sleepComponent(10);
    cnt--;
    if(cnt == 0)
    {
      int fr = getFramerateMilliSeconds();
      debug("grabbing with %d ms per frame (%.2f frames per second)",
          fr, (fr > 0. ? 1000./fr : 0.));
      cnt = 100;
    }
  }
}

}

