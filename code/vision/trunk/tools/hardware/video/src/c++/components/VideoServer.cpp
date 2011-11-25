/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <sstream>
#include <cogxmath.h>
#include "VideoServer.h"

namespace cast
{

using namespace std;
using namespace cogx::Math;

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

void VideoServerI::getImage(Ice::Int camId, Video::Image& image, const Ice::Current&)
{
  vidSrv->retrieveFrame((int)camId, image);
}

void VideoServerI::getImages(Video::ImageSeq& images, const Ice::Current&)
{
  // prividing width = height = 0 means using native image size
  vidSrv->retrieveFrames(0, 0, images);
}

void VideoServerI::getScaledImages(Ice::Int width, Ice::Int height,
      Video::ImageSeq& images, const Ice::Current&)
{
  vidSrv->retrieveFrames(width, height, images);
}

VideoServer::VideoServer()
{
  iceVideoName = "";
  iceVideoPort = cdl::CPPSERVERPORT;
}

void VideoServer::setupMyIceCommunication()
{
  Ice::Identity id;
  id.name = iceVideoName;
  id.category = "VideoServer";
  getObjectAdapter()->add(new VideoServerI(this), id);
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
      Video::CameraParameters pars;
      // calibration files can be either monocular calibration files (e.g.
      // "flea0.cal", "flea1.cal") or SVS stereo calib files (e.g.
      // "stereo.ini:L", "stereo.ini:R"). Note that in the latter case the
      // actual file name is extended with a ":" (to indicate we have a stereo
      // case) and a L or R indicating we refer to the LEFT or RIGHT camera of
      // the stereo rig.
      if(file.find(":") == string::npos)
      {
        // monocular case
        Video::loadCameraParameters(pars, file);
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
          Video::loadCameraParametersFromSVSCalib(pars, pure_filename, LEFT);
        else if(side == 'R')
          Video::loadCameraParametersFromSVSCalib(pars, pure_filename, RIGHT);
        else
          throw runtime_error(exceptionMessage(__HERE__,
                "camera '%c' invalid in config '%s', must be either :L or :R",
                side, file.c_str()));
      }
      camPars.push_back(pars);
    }
  }

  if((it = _config.find("--videoname")) != _config.end())
  {
    iceVideoName = it->second;
  }

  /*if((it = _config.find("--videoport")) != _config.end())
  {
    istringstream str(it->second);
    str >> iceVideoPort;
  }*/

  // in case no camera config files were given, assume default configs
  if(camPars.size() == 0)
  {
    camPars.resize(camIds.size());
    for(size_t i = 0; i < camPars.size(); i++)
      Video::initCameraParameters(camPars[i]);
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != camPars.size())
    throw runtime_error(exceptionMessage(__HERE__,
      "numbers of camera IDs %d and camera config files %d do not match",
      (int)camIds.size(), (int)camPars.size()));
  if(iceVideoName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
  if(camIds.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no camera IDs given"));

}

void VideoServer::start()
{
  setupMyIceCommunication();
}

void VideoServer::runComponent()
{
  while(isRunning())
  {
    grabFrames();
    int fr = getFramerateMilliSeconds();
    log("grabbing with %d ms per frame (%.2f frames per second)",
        fr, (fr > 0. ? 1000./fr : 0.));
    // need this fucking sleep to avoid grabFrames() hogging the mutex and thus
    // excluding retrieveFrames()
    // (note: 2009-02-13: first appearance of the f-word in the new code :)
    //sleepComponent(10);
  }
}

}
