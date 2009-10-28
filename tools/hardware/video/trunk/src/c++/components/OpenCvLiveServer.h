/**
 * @author Michael Zillich
 * @date Februrary 2009
 */

#ifndef OPEN_CV_LIVE_SERVER_H
#define OPEN_CV_LIVE_SERVER_H

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/time.h>
#include <cv.h>
#include <highgui.h>
#include <dc1394/types.h>	// apt-get libdc1394-13-dev || libdc1394-22-dev
#include <dc1394/control.h>	// apt-get libdc1394-13-dev || libdc1394-22-dev
#include "VideoServer.h"

namespace cast
{

/**
 * Video device simply wrapping the OpenCV capture API.
 */
class OpenCvLiveServer : public VideoServer
{
private:
  /**
   * Class for measuring how many things happen per second.
   * author: Nick Hawes
   */
  class Timer
  {
  public:
    Timer();
    void increment();
    double getRate() const
    {
      return rate;
    }
    bool rateChange() const
    {
      return sigChange;
    }

  private:
    int count;
    double rate;
    double lastRate;
    double changeThresh;
    timeval startTime;
    bool sigChange;
  };

  /**
   * the actual wrapped OpenCV captures.
   */
  std::vector<CvCapture*> captures;

  /**
   * pointers to retrieved images, point to internal memory of the capture
   * device, do not delete!
   */
  std::vector<IplImage*> retrievedImages;

  /**
   * time stamps when Ipl images were captured.
   */
  std::vector<cast::cdl::CASTTime> grabTimes;

  /**
   * format for Bayer to RGB conversion, CV_COLORCVT_MAX for no conversion
   */
  int bayerCvt;  

  int framerateMillis;
  int width;
  int height;
	
  /**
   * Timer to measure actual frame rate.
   */
  Timer timer;

  void init(int dev_class, const std::vector<int> &dev_nums,
      const std::string &bayer) throw(std::runtime_error);
  /**
   * Converts an IplImage to a system Image format.
   */
  void copyImage(const IplImage *iplImg, Video::Image &img)
    throw(std::runtime_error);
  bool haveBayer() {return bayerCvt != CV_COLORCVT_MAX;}
  void grabFramesInternal();
  void retrieveFrameInternal(int camIdx, int width, int height,
      Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);

public:
  OpenCvLiveServer();
  virtual ~OpenCvLiveServer();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);
  virtual void grabFrames();
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
  // void enableUndistortion(const float dist[4]);
  // void disableUndistortion();
};

}

#endif

