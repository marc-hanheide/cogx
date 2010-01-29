/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "PointGreyServer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PointGreyServer();
  }
}

namespace cast
{

using namespace std;

PointGreyServer::MeanRate::MeanRate(int size)
  : mean(size)
{
  gettimeofday(&prev, 0);
}

/**
 * returns the current rate as 1/(current time - last time) or 0 if current time
 * == last time
 */
float PointGreyServer::MeanRate::calculateCurrentRate()
{
  timeval now;

  gettimeofday(&now, 0);
  // from http://www.delorie.com/gnu/docs/glibc/libc_428.html
  // Perform the carry for the later subtraction by updating y.
  if(now.tv_usec < prev.tv_usec)
  {
    int nsec = (prev.tv_usec - now.tv_usec) / 1000000 + 1;
    prev.tv_usec -= 1000000 * nsec;
    prev.tv_sec += nsec;
  }
  if(now.tv_usec - prev.tv_usec > 1000000)
  {
    int nsec = (now.tv_usec - prev.tv_usec) / 1000000;
    prev.tv_usec += 1000000 * nsec;
    prev.tv_sec -= nsec;
  }
  // tv_usec is certainly positive.
  float diffSeconds = (float)(now.tv_sec - prev.tv_sec);
  float diffMicros = (float)(now.tv_usec - prev.tv_usec);
  float totalDiffSeconds = diffSeconds  + diffMicros / 1000000.;
  prev = now;
  //return fpclassify(totalDiffSeconds) != FP_ZERO ? 1./totalDiffSeconds : 0.;
  // return frame rate in milliseconds
  return totalDiffSeconds*1000.;
}


PointGreyServer::PointGreyServer()
{
  width = height = 0;
  cameras = 0;
}

PointGreyServer::~PointGreyServer()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    cameras[i]->StopCapture();
    cameras[i]->Disconnect();
    delete cameras[i];
  }
  delete[] cameras;
}

void PointGreyServer::LogCameraInfo(FlyCapture2::CameraInfo* pCamInfo)
{
  log("\n*** CAMERA INFORMATION ***\n"
      "Serial number - %u\n"
      "Camera model - %s\n"
      "Camera vendor - %s\n"
      "Sensor - %s\n"
      "Resolution - %s\n"
      "Firmware version - %s\n"
      "Firmware build time - %s\n",
      pCamInfo->serialNumber,
      pCamInfo->modelName,
      pCamInfo->vendorName,
      pCamInfo->sensorInfo,
      pCamInfo->sensorResolution,
      pCamInfo->firmwareVersion,
      pCamInfo->firmwareBuildTime );
}

FlyCapture2::VideoMode PointGreyServer::selectVideoMode(int &_width, int &_height)
{
  // note: we use only width to select the video mode and assume height to be
  // 3/4 of width
  // If we have several colour formats to choose from, we always select colour
  // over greyscale and select the one with hightest colour resolution, e.g. RGB
  // over YUV422
  if(_width == 160)
  {
    _height = 120;
    return FlyCapture2::VIDEOMODE_160x120YUV444;
  }
  if(_width == 320)
  {
    _height = 240;
    return FlyCapture2::VIDEOMODE_320x240YUV422;
  }
  if(_width == 640)
  {
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
  if(_width == 800)
  {
    _height = 600;
    return FlyCapture2::VIDEOMODE_800x600RGB;
  }
  if(_width == 1024)
  {
    _height = 768;
    return FlyCapture2::VIDEOMODE_1024x768RGB;
  }
  if(_width == 1280)
  {
    _height = 960;
    return FlyCapture2::VIDEOMODE_1280x960RGB;
  }
  else
  {
    // the default
    _width = 640;
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
}

FlyCapture2::FrameRate PointGreyServer::selectFrameRate(int &_fps)
{
  if(_fps == 7)
    return FlyCapture2::FRAMERATE_7_5;
  if(_fps == 15)
    return FlyCapture2::FRAMERATE_15;
  if(_fps == 30)
    return FlyCapture2::FRAMERATE_30;
  if(_fps == 60)
    return FlyCapture2::FRAMERATE_60;
  if(_fps == 120)
   return FlyCapture2::FRAMERATE_120;
  // the default
  _fps = 15;
  return FlyCapture2::FRAMERATE_15;
}

void PointGreyServer::init() throw(runtime_error)
{
  unsigned int numAvailableCameras;
  FlyCapture2::Error error;

  error = busMgr.GetNumOfCameras(&numAvailableCameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());

  if(numAvailableCameras < (size_t)getNumCameras())
    throw runtime_error("PointGreyServer: insufficient number of cameras detected");
  cameras = new FlyCapture2::Camera*[(size_t)getNumCameras()];
  retrievedImages.resize((size_t)getNumCameras());
  grabTimes.resize((size_t)getNumCameras());

  // Connect to all detected cameras and attempt to set them to
  // a common video mode and frame rate
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    cameras[i] = new FlyCapture2::Camera();

    FlyCapture2::PGRGuid guid;
    error = busMgr.GetCameraFromIndex(i, &guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());

    // Connect to a camera
    error = cameras[i]->Connect(&guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());

    // Get the camera information
    FlyCapture2::CameraInfo camInfo;
    error = cameras[i]->GetCameraInfo(&camInfo);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
    LogCameraInfo(&camInfo); 

    FlyCapture2::VideoMode mode = selectVideoMode(width, height);
    FlyCapture2::FrameRate rate = selectFrameRate(fps);
    error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }
  error = FlyCapture2::Camera::StartSyncCapture(getNumCameras(),
      (const FlyCapture2::Camera**)cameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
}

void PointGreyServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    str >> width >> height;
  }

  if((it = _config.find("--framerate_fps")) != _config.end())
  {
    istringstream str(it->second);
    str >> fps;
  }

  // do some initialisation based on configured items
  init();
}

void PointGreyServer::grabFramesInternal()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    FlyCapture2::Error error;
    error = cameras[i]->RetrieveBuffer(&retrievedImages[i]);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }

  cdl::CASTTime time = getCASTTime();
  for(size_t i = 0; i < grabTimes.size(); i++)
    grabTimes[i] = time;
  framerateMillis.insert();
}

void PointGreyServer::grabFrames()
{
  grabFramesInternal();
}

void PointGreyServer::retrieveFrameInternal(int camIdx, int width, int height,
    Video::Image &frame)
{
  // no size given, use native size
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    copyImage(retrievedImages[camIdx], frame);
  }
  else
  {
    // NOTE: this is very wasteful! And should only be a temporary solution!
    copyImage(retrievedImages[camIdx], frame);
    IplImage *tmp = 0; 
    IplImage *tmp_resized = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    convertImageToIpl(frame, &tmp);
    cvResize(tmp, tmp_resized);
    convertImageFromIpl(tmp_resized, frame);
    cvReleaseImage(&tmp_resized);
    cvReleaseImage(&tmp);
  }

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];
}

void PointGreyServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void PointGreyServer::retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames)
{
  frames.resize((size_t)getNumCameras());
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void PointGreyServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void PointGreyServer::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
}

int PointGreyServer::getFramerateMilliSeconds()
{
  return framerateMillis.getRate();
}

/**
 * note: If img is of appropriate size already, no memory allocation takes
 * place.
 */
void PointGreyServer::copyImage(const FlyCapture2::Image &flyImg,
    Video::Image &img) throw(runtime_error)
{
  if(flyImg.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
    throw runtime_error("PointGreyServer: can only handle RGB8 image format");
  if(flyImg.GetStride() != flyImg.GetCols()*3)
    throw runtime_error("PointGreyServer: can only handle images with no padding");
  if(flyImg.GetDataSize() != flyImg.GetCols()*flyImg.GetRows()*3)
    throw runtime_error("PointGreyServer: can only handle images with size 3*w*h bytes");

  img.width = flyImg.GetCols();
  img.height = flyImg.GetRows();
  img.data.resize(img.width*img.height*3);
  memcpy(&img.data[0], flyImg.GetData(), img.data.size());
}

}

