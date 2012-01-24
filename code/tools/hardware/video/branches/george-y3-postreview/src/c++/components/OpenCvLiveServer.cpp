/**
 * @file OpenCvLiveServer.h
 * @author Michael Zillich, Andreas Richtsfeld
 * @date February 2009, 2010
 * @brief Image server, based on openCV, to grab live images from multiple cameras.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "OpenCvLiveServer.h"

// define this to use memcpy() instead of looping over image lines and pixels
// memcpy() is a LOT faster, but will screw up the images if the memory layout
// (byte per pixel, bytes per line) is not exactly the same.
#define FAST_DIRTY_CONVERSION

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::OpenCvLiveServer();
  }
}

namespace cast
{

using namespace std;


OpenCvLiveServer::OpenCvLiveServer()
{
  bayerCvt = CV_COLORCVT_MAX;
  framerateMillis = 0;
  captureSize.width = 0;
  captureSize.height = 0;
}

OpenCvLiveServer::~OpenCvLiveServer()
{
  for(size_t i = 0; i < captures.size(); i++)
    cvReleaseCapture(&captures[i]);
}

void OpenCvLiveServer::getResolution(int camIdx, CvSize &size)
{
  int i = camIdx;
  size.width = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
  size.height = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
}

bool OpenCvLiveServer::setResolution(int camIdx, CvSize &size)
{
  int i = camIdx;
  int w = 0, h = 0;
  bool gotRequestedSize = true;

  // first try the simple set property
  cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH, size.width);
  cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT, size.height);

  // check if that worked
  w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
  h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
  // if not, use firewire specific set mode
  if(w != size.width || h != size.height)
  {
    log("cvSetCaptureProperty(WIDTH/HEIGHT) didn't work, trying cvSetCaptureProperty(MODE)");
    if(size.width == 320)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_320x240_YUV422);
    else if(size.width == 640)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_640x480_YUV411);
    else if(size.width == 800)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_800x600_MONO8);
    else if(size.width == 1024)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1024x768_RGB8);
    else if(size.width == 1280)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1280x960_RGB8);
    else if(size.width == 1600)
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1600x1200_RGB8);
    else
      println("Warning: video resolution %d x %d not supported!\n",
          size.width, size.height);

    // now check again
    w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
    h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);

    // if setting still did not work, use whatever the cameras are set to
    if(w != size.width || h != size.height)
    {
      log("Warning: failed to set resolution to %d x %d, using %d x %d!\n",
          size.width, size.height, w, h);
      size.width = w;
      size.height = h;
      gotRequestedSize = false;
    }
  }
  return gotRequestedSize;
}

/**
 * @param ids  list of camera IDs
 * @param dev_nums  list of device numbers (typically 0, 1) corresponding to
 *                  camera IDs
 * @param bayer  Some cameras (e.g. Point Grey Flea) return the raw Bayer
 *               pattern rather than YUV or RGB. For these we have to perform
 *               Bayer ro RGB conversion. This parameter indicates the
 *               order of R,G and B pixels in the Bayer pattern: one of
 *               "BGGR" "GBBR" "RGGB" "GRRB" or "" (for no conversion).
 */
void OpenCvLiveServer::init(int dev_class, const vector<int> &dev_nums,
  const string &bayer) throw(runtime_error)
{
  if(dev_nums.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__,
	  "must specify at least one camera"));
  if(dev_nums.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of devices %d does not match number of camera IDs %d",
          (int)dev_nums.size(), (int)camIds.size()));

  captures.resize(dev_nums.size());
  grabTimes.resize(dev_nums.size());
  retrievedImages.resize(dev_nums.size());
  for(size_t i = 0; i < dev_nums.size(); i++)
    retrievedImages[i] = 0;
  for(size_t i = 0; i < dev_nums.size(); i++)
  {
    captures[i] = cvCreateCameraCapture(dev_class + dev_nums[i]);
    if(captures[i] == 0)
      throw runtime_error(exceptionMessage(__HERE__,
        "failed to create capture for video device %d", dev_nums[i]));
    if(bayer.empty())
    {
      bayerCvt = CV_COLORCVT_MAX;
    }
    else
    {
      if(bayer == "BGGR")
        bayerCvt = CV_BayerBG2RGB;
      else if(bayer == "GBBR")
        bayerCvt = CV_BayerGB2RGB;
      else if(bayer == "RGGB")
        bayerCvt = CV_BayerRG2RGB;
      else if(bayer == "GRRB")
        bayerCvt = CV_BayerGR2RGB;
      else
        throw runtime_error(exceptionMessage(__HERE__,
            "invalid bayer order '%s', must be one of 'BGGR' 'GBBR' 'RGGB' 'GRRB'",
            bayer.c_str()));
      // the default in opencv is CV_CAP_PROP_CONVERT_RGB=1 which causes
      // cameras with bayer encoding to be converted from mono to rgb
      // without using the bayer functions. CV_CAP_PROP_CONVERT_RGB=0
      // keeps the original format.
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_CONVERT_RGB, 0.0);
    }
  }

  // if capture size was not set by config, use what is currently set in the
  // cameras
  if(captureSize.width == 0 || captureSize.width == 0)
  {
    // get currently set resolution of first camera and set it to all other
    // cameras so all cameras will have same resolution
    getResolution(0, captureSize);
    for(size_t i = 1; i < captures.size(); i++)
      if(!setResolution(i, captureSize))
        throw runtime_error(exceptionMessage(__HERE__,
              "failed to set all cameras to identical resolutions"));
    log("using currently set video resolution %d x %d\n",
        captureSize.width, captureSize.height);
  }
  else
  {
    log("setting video resolution to %d x %d\n",
        captureSize.width, captureSize.height);
    // set resolution for first camera, where we don't care whether our
    // requested resolution was actually set
    setResolution(0, captureSize);
    // set for all other cameras to the same resolution (whatever it was)
    // and insist they are the same
    for(size_t i = 1; i < captures.size(); i++)
      if(!setResolution(i, captureSize))
        throw runtime_error(exceptionMessage(__HERE__,
              "failed to set all cameras to identical resolutions"));
    log("using available video resolution %d x %d\n", captureSize.width, captureSize.height);
  }

  // frames per second
  double fps = cvGetCaptureProperty(captures[0], CV_CAP_PROP_FPS);

  if(fps > 0.)
    // milliseconds per frame
    framerateMillis = (int)(1000./fps);
  else
    // just some huge value (better than 0. as that might result in divides by
    // zero somewhere)
    framerateMillis = 1000000;

  // to make sure we have images in the capture's buffer
  grabFramesInternal();
}

void OpenCvLiveServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  vector<int> dev_nums;
  int dev_class = CV_CAP_ANY;
  string bayer;
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);

  if((it = _config.find("--devclass")) != _config.end())
  {
    if(it->second == "FIREWIRE")
      dev_class = CV_CAP_IEEE1394;
    else if(it->second == "VIDEO4LINUX" || it->second == "USB")
      dev_class = CV_CAP_V4L2;
    else
    {
      ostringstream msg;
      msg << "unknown device class '" << it->second << "'";
      throw runtime_error(msg.str());
    }
  }

  if((it = _config.find("--devnums")) != _config.end())
  {
    istringstream str(it->second);
    int dev;
    while(str >> dev)
      dev_nums.push_back(dev);
  }
  else
  {
    // assume 0 as default device
    dev_nums.push_back(0);
  }

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    str >> captureSize.width >> captureSize.height;
  }

  // if cameras return raw Bayer patterns
  if((it = _config.find("--bayer")) != _config.end())
  {
    bayer = it->second;
  }

  // do some initialisation based on configured items
  init(dev_class, dev_nums, bayer);
}

void OpenCvLiveServer::grabFramesInternal()
{
  for(size_t i = 0; i < captures.size(); i++)
  {
    // grab image into internal storage of the capture device
    cvGrabFrame(captures[i]);
    // and invalidate the corresponding retrieved image
    retrievedImages[i] = 0;
  }

  cdl::CASTTime time = getCASTTime();
  for(size_t i = 0; i < grabTimes.size(); i++)
    grabTimes[i] = time;
  timeStats.increment();
  // frames per second
  if(timeStats.getRate() > 0.)
    // milliseconds per frame
    framerateMillis = (int)(1000./timeStats.getRate());
}

void OpenCvLiveServer::grabFrames()
{
  grabFramesInternal();
}

void OpenCvLiveServer::retrieveFrameInternal(int camIdx, int width, int height,
    Video::Image &frame)
{
  // note: calling cvRetrieveFrame() only when really needed reduces system
  // load (on Core 2 Duo 2.4GHz) from 18% to virtually 0%
  if(retrievedImages[camIdx] == 0)
    retrievedImages[camIdx] = cvRetrieveFrame(captures[camIdx]);

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) ||
     (width == captureSize.width && height == captureSize.height))
  {
    copyImage(retrievedImages[camIdx], frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, captureSize.width, captureSize.height);
  }
  else
  {
    char id[16];
    sprintf(id, "frame%d", camIdx);
    // use image cache to avoid allocate/deallocating all the time
    IplImage *tmp = m_imageCache.getImage(id, width, height, IPL_DEPTH_8U, 3);
    cvResize(retrievedImages[camIdx], tmp);
    copyImage(tmp, frame);

    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
  }
}

void OpenCvLiveServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void OpenCvLiveServer::retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames)
{
  frames.resize(getNumCameras());
  for(int i = 0; i < getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void OpenCvLiveServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void OpenCvLiveServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  log("OpenCvLiveServer::retrieveHRFrames: not yet implemented.\n");
}

void OpenCvLiveServer::getImageSize(int &width, int &height)
{
  width = captureSize.width;
  height = captureSize.height;
}

int OpenCvLiveServer::getFramerateMilliSeconds()
{
  return framerateMillis;
}

/**
 * note: If img is of appropriate size already, no memory allocation takes
 * place.
 */
void OpenCvLiveServer::copyImage(const IplImage *iplImg, Video::Image &img)
  throw(runtime_error)
{
  int channels = 3;  // Image frame always has RGR24
  IplImage *tmp = 0;

  assert(iplImg != 0);
  if(!haveBayer())
  {
    if(iplImg->nChannels != channels)
      throw runtime_error(exceptionMessage(__HERE__,
        "can only handle colour images - the video seems to be grey scale"));
  }
  else
  {
    // HACK: how do we know the correct colour code?
    tmp = cvCreateImage(cvSize(iplImg->width, iplImg->height), IPL_DEPTH_8U, 3);
    if(tmp == 0)
      throw runtime_error(exceptionMessage(__HERE__,
            "failed to allocate image buffer"));
    cvCvtColor(iplImg, tmp, bayerCvt);
    iplImg = tmp;
  }

  img.width = iplImg->width;
  img.height = iplImg->height;
  img.data.resize(iplImg->width*iplImg->height*iplImg->nChannels);
  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and
  // line padding.
  if(iplImg->depth == (int)IPL_DEPTH_8U || iplImg->depth == (int)IPL_DEPTH_8S)
  {
#ifndef FAST_DIRTY_CONVERSION
    int x, y;  // c;
    for(y = 0; y < iplImg->height; y++)
      for(x = 0; x < iplImg->width; x++)
      {
        //for(c = 0; c < channels; c++)
        //  img.data[channels*(y*img.width + x) + c] =
        //    iplImg->imageData[y*iplImg->widthStep + channels*x + c];
        img.data[channels*(y*img.width + x) + 0] =
           iplImg->imageData[y*iplImg->widthStep + channels*x + 2];
        img.data[channels*(y*img.width + x) + 1] =
           iplImg->imageData[y*iplImg->widthStep + channels*x + 1];
        img.data[channels*(y*img.width + x) + 2] =
           iplImg->imageData[y*iplImg->widthStep + channels*x + 0];
      }
#else
    memcpy(&img.data[0], iplImg->imageData, iplImg->height*iplImg->widthStep);
#endif
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
      "can only handle 8 bit colour values"));

  if(haveBayer())
    cvReleaseImage(&tmp);
}

/**
 * @brief This function is only for the PointGrey server available: experimental mode.
 * @param width Image width
 * @param height Image height
 * @param offsetX Offset in x- direction for Format7 mode.
 * @param offsetY Offset in y- direction for Format7 mode.
 * @param mode Image grabbing mode for the Format7 mode.
 * @param fps Requested framerate [1/s]
 */
void OpenCvLiveServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
  log("only for the PointGrey server available: abort.");
}

bool OpenCvLiveServer::inFormat7Mode()
{
  return false;
}

/**
 * @brief Get the server name
 * @param name Server name (PointGreyServer / OpenCvImgSeqServer / OpenCvLiveServer)
 */
const std::string OpenCvLiveServer::getServerName()
{
  const std::string str("OpenCvLiveServer");
  return str;
}

}

