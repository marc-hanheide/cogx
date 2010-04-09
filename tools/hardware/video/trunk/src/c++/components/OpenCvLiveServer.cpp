/**
 * @author Michael Zillich
 * @date February 2009
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

OpenCvLiveServer::Timer::Timer()
: count(0),
  rate(0.),
  lastRate(0.),
  changeThresh(0.),
  sigChange(false)
{
}

void OpenCvLiveServer::Timer::increment()
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

OpenCvLiveServer::OpenCvLiveServer()
{
  bayerCvt = CV_COLORCVT_MAX;
  framerateMillis = 0;
  width = height = 0;
}

OpenCvLiveServer::~OpenCvLiveServer()
{
  for(size_t i = 0; i < captures.size(); i++)
    cvReleaseCapture(&captures[i]);
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


  // HACK
	int w=0, h=0;
  for(size_t i = 0; i < dev_nums.size(); i++)
	{
		println("[OpenCvLiveServer::init] setting width x height  %d x %d\n", width, height);

		cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH, width);
		cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT, height);

		w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
		h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);

		if(w!=width || h!=height)
		{
			println("[OpenCvLiveServer::init] cvSetCaptureProperty(WIDTH, HEIGHT) didn't work, trying cvSetCaptureProperty(MODE)");

			if(width == 320) {
				cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_320x240_YUV422);
			}
			if(width == 640)
				cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_640x480_YUV411);
			if(width == 800)
				cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_800x600_MONO8);
			if(width == 1024)
				printf("[OpenCvLiveServer::init] Warning: setting video resolution to %d is not supported by this OpenCV implementation!\n", width);
			if(width == 1280)
				cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1280x960_RGB8);
			if(width == 1600)
				cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1600x1200_RGB8);

			w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
			h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);

			if(w!=width)
				printf("[OpenCvLiveServer::init] Warning: setting video resolution not supported by this OpenCV implementation!\n");
		}
		// HACK END

		width = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
		height = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
		// frames per second
		double fps = cvGetCaptureProperty(captures[i], CV_CAP_PROP_FPS);

		if(fps > 0.)
			// milliseconds per frame
			framerateMillis = (int)(1000./fps);
		else
			// just some huge value (better than 0. as that might result in divides by
			// zero somewhere)
			framerateMillis = 1000000;
	}

  // to make sure we have images in the capture's buffer
  grabFramesInternal();
}

void OpenCvLiveServer::configure(const map<string,string> & _config)
  throw(runtime_error)
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
    str >> width >> height;
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
  timer.increment();
  // frames per second
  if(timer.getRate() > 0.)
    // milliseconds per frame
    framerateMillis = (int)(1000./timer.getRate());
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
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    copyImage(retrievedImages[camIdx], frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, this->width, this->height);
  }
  else
  {
    IplImage *tmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvResize(retrievedImages[camIdx], tmp);
    copyImage(tmp, frame);
    // TODO: avoid allocate/deallocating all the time
    cvReleaseImage(&tmp);
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
  for(size_t i = 0; i < getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void OpenCvLiveServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void OpenCvLiveServer::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
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

}

