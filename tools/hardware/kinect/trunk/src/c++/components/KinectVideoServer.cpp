/**
 * @file KinectVideoServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Video server for the kinect sensor.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
//#include <VideoUtils.h>
#include "KinectVideoServer.h"

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
    return new cast::KinectVideoServer();
  }
}

namespace cast
{

using namespace std;

KinectVideoServer::Timer::Timer() : count(0), rate(0.), lastRate(0.), changeThresh(0.), sigChange(false)
{
}

void KinectVideoServer::Timer::increment()
{
  // increment
  count++;

  // if this is the first time
  if(count == 1)
  {
    // start timer
    gettimeofday(&startTime, 0);
  }
  else
  {
    // current time
    timeval now;
    gettimeofday(&now, 0);
    timeval start(startTime);

    // from http://www.delorie.com/gnu/docs/glibc/libc_428.html
    // Perform the carry for the later subtraction by updating y.
    if(now.tv_usec < start.tv_usec)
    {
      int nsec = (start.tv_usec - now.tv_usec) / 1000000 + 1;
      start.tv_usec -= 1000000 * nsec;
      start.tv_sec += nsec;
    }
    if(now.tv_usec - start.tv_usec > 1000000)
    {
      int nsec = (now.tv_usec - start.tv_usec) / 1000000;
      start.tv_usec += 1000000 * nsec;
      start.tv_sec -= nsec;
    }

    // tv_usec is certainly positive.
    double diffSeconds = (double)(now.tv_sec - start.tv_sec);
    double diffMicros = (double)(now.tv_usec - start.tv_usec);

    double totalDiffSeconds = diffSeconds  + (diffMicros / 1000000.);

    // get new rate
    rate = (double)count/totalDiffSeconds;

    // diff the old and new rates
    double diffRate = abs(lastRate - rate);

    // compare diff to a percentage of the old rate
    if(diffRate > changeThresh)
    {
      sigChange = true;
      lastRate = rate;
      // get 3% of the old rate... HACK WARNING CONSTANT
      changeThresh = lastRate * 0.03;
    }
    else
    {
      sigChange = false;
    }
  }
}

KinectVideoServer::KinectVideoServer()
{
}

KinectVideoServer::~KinectVideoServer()
{
  printf("KinectVideoServer::~KinectVideoServer.\n");
//   for(size_t i = 0; i < captures.size(); i++)
//     cvReleaseCapture(&captures[i]);
  delete kinect;
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera
 * @param size video resolution
 */
void KinectVideoServer::getResolution(int camIdx, CvSize &size)
{
  kinect->GetColorVideoSize(size);
}


/**
 * @brief Set resolution for a given camera
 * @param camIdx which camera
 * @param size requested video resolution, on exit contains the actually
 *        set resolution, which might differ dependig on the cameras capabilities
 * @return true if the requested resolution could be set, false if another
 *        reslution was chosen
 */
bool KinectVideoServer::setResolution(int camIdx, CvSize &size)
{
  printf("KinectVideoServer::setResolution: Warning: Not yet implemented! Defined by kinect camera calibration file!\n");
//   int i = camIdx;
//   int w = 0, h = 0;
//   bool gotRequestedSize = true;
// 
//   // first try the simple set property
//   cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH, size.width);
//   cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT, size.height);
// 
//   // check if that worked
//   w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
//   h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
//   // if not, use firewire specific set mode
//   if(w != size.width || h != size.height)
//   {
//     log("cvSetCaptureProperty(WIDTH/HEIGHT) didn't work, trying cvSetCaptureProperty(MODE)");
//     if(size.width == 320)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_320x240_YUV422);
//     else if(size.width == 640)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_640x480_YUV411);
//     else if(size.width == 800)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_800x600_MONO8);
//     else if(size.width == 1024)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1024x768_RGB8);
//     else if(size.width == 1280)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1280x960_RGB8);
//     else if(size.width == 1600)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1600x1200_RGB8);
//     else
//       println("Warning: video resolution %d x %d not supported!\n",
//           size.width, size.height);
// 
//     // now check again
//     w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
//     h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
// 
//     // if setting still did not work, use whatever the cameras are set to
//     if(w != size.width || h != size.height)
//     {
//       log("Warning: failed to set resolution to %d x %d, using %d x %d!\n",
//           size.width, size.height, w, h);
//       size.width = w;
//       size.height = h;
//       gotRequestedSize = false;
//     }
//   }
//   return gotRequestedSize;
  return false; /// TODO remove
}

/**
 * @brief Init the Kinect driver.
 * @param ids  list of camera IDs
 * @param dev_nums  list of device numbers (typically 0, 1) corresponding to camera IDs
 */
void KinectVideoServer::init(const vector<int> &dev_nums) throw(runtime_error)
{
  CvSize size;
  const char* name = kinectConfig.c_str();
  kinect = new Kinect::Kinect(name);
  kinect->GetColorVideoSize(size);
  captureSize.width = size.width;
  captureSize.height = size.height;

  if(dev_nums.size() == 0)									/// TODO Sinnlos!!!
    throw runtime_error(exceptionMessage(__HERE__,
          "must specify at least one camera"));
  if(dev_nums.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of devices %d does not match number of camera IDs %d",
          (int)dev_nums.size(), (int)camIds.size()));

  grabTimes.resize(dev_nums.size());
  retrievedImages.resize(dev_nums.size());
  for(size_t i = 0; i < dev_nums.size(); i++)
    retrievedImages[i] = 0;
}

void KinectVideoServer::configure(const map<string,string> & _config) throw(runtime_error)
{ 
  vector<int> dev_nums;
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);
  
  if((it = _config.find("--kconfig")) != _config.end())
  {  
    istringstream str(it->second);
    str >> kinectConfig;
  }
  else printf("KinectVideoServer::configure: Warning: No Kinect calibration file specified.\n");

  if((it = _config.find("--devnums")) != _config.end())
  {
    istringstream str(it->second);
    int dev;
    while(str >> dev)
      dev_nums.push_back(dev);
  }
  else
    dev_nums.push_back(0);  // assume 0 as default device

    
//  if((it = _config.find("--imgsize")) != _config.end())
//  {
//    istringstream str(it->second);
//     str >> captureSize.width >> captureSize.height;
//  }


  // do some initialisation based on configured items
  init(dev_nums);										/// TODO Do we have more than one device?

  kinect->StartCapture(0);
  log("Capturing started.");
}

void KinectVideoServer::grabFramesInternal()  // grab images is already implemented in the kinect driver
{}

void KinectVideoServer::grabFrames()
{
//   grabFramesInternal();
}

void KinectVideoServer::retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame)
{
  if(retrievedImages[camIdx] == 0)
  {
    kinect->NextFrame();
    if(!kinect->GetColorImage(&retrievedImages[camIdx]))
    {
      printf("KinectVideoServer::retrieveFrameInternal: Warning: Could not get Color image from kinect sensor.\n");
      return;
    }
  }
  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) || (width == captureSize.width && height == captureSize.height))
  {
    copyImage(retrievedImages[camIdx], frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set native capture size)
    
//     changeImageSize(frame.camPars, captureSize.width, captureSize.height);								// TODO we have to change the size?
  }
  else throw runtime_error(exceptionMessage(__HERE__, "image size conversion not yet implemented!"));    

  cvReleaseImage(&retrievedImages[camIdx]);
}

void KinectVideoServer::retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void KinectVideoServer::retrieveFrames(int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(getNumCameras());
  for(size_t i = 0; i < getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void KinectVideoServer::retrieveFrame(int camId, int width, int height, Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void KinectVideoServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  printf("KinectVideoServer::retrieveHRFrames: not yet implemented.\n");
}

void KinectVideoServer::getImageSize(int &width, int &height)
{
  width = captureSize.width;
  height = captureSize.height;
}

int KinectVideoServer::getFramerateMilliSeconds()
{
  printf("KinectVideoServer::getFramerateMilliSeconds: not yet implemented!\n");
  return -1;
}

/**
 * @brief Copy image from iplImage to a cast-Image, to send it via wm.
 * note: If img is of appropriate size already, no memory allocation takes place.
 * @param iplImg Grabbed iplImage
 * @param img New created cast image
 */
void KinectVideoServer::copyImage(const IplImage *iplImg, Video::Image &img) throw(runtime_error)
{
  int channels = 3;  // Image frame always has RGR24
//   IplImage *tmp = 0;
// 
//   assert(iplImg != 0);
//   if(!haveBayer())
//   {
//     if(iplImg->nChannels != channels)
//       throw runtime_error(exceptionMessage(__HERE__,
//         "can only handle colour images - the video seems to be grey scale"));
//   }
//   else
//   {
//     // HACK: how do we know the correct colour code?
//     tmp = cvCreateImage(cvSize(iplImg->width, iplImg->height), IPL_DEPTH_8U, 3);
//     if(tmp == 0)
//       throw runtime_error(exceptionMessage(__HERE__, "failed to allocate image buffer"));
//     cvCvtColor(iplImg, tmp, bayerCvt);
//     iplImg = tmp;
//   }
// printf("KinectServer::copyImage: 2\n"); 

  img.width = iplImg->width;
  img.height = iplImg->height;
  if(iplImg->nChannels != channels) 
  {
    printf("KinectServer::copyImage: Error: wrong number of channels.\n"); 
    return;
  }

  img.data.resize(iplImg->width*iplImg->height*iplImg->nChannels);

  // note: this neat triple loop might be somewhat slower than a memcpy, but
  // makes sure images are copied correctly irrespective of memory layout and line padding.
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
  else throw runtime_error(exceptionMessage(__HERE__, "can only handle 8 bit colour values"));
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
void KinectVideoServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
  log("KinectVideoServer: Warning: only for the PointGrey server available: abort.");
}

bool KinectVideoServer::inFormat7Mode()
{
  return false;
}

/**
 * @brief Get the server name
 * @param name Server name (PointGreyServer / OpenCvImgSeqServer / OpenCvLiveServer / KinectVideoServer)
 */
const std::string KinectVideoServer::getServerName()
{
  const std::string str("KinectVideoServer");
  return str;
}

}

