/**
 * @file KinectVideoServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Video server for the kinect sensor.
 */

#ifndef KINECT_VIDEO_SERVER_H
#define KINECT_VIDEO_SERVER_H

#include <vector>
#include <sys/time.h>

#include "VideoServer.h"
#include "Kinect.h"

namespace cast
{

/**
 * @brief Video device simply wrapping the Kinect API.
 */
class KinectVideoServer : public VideoServer
{
private:
  /**
   * @brief Class for measuring how many things happen per second. \n
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

  std::string kinectConfig;                     ///< Kinect configuration file
  CvSize captureSize;                           ///< Size of captured images from kinect
  Kinect::Kinect *kinect;                       ///< The kinect hardware interface.
  std::vector<IplImage*> retrievedImages;       ///< pointers to retrieved images for each device (dev_nums)

  std::vector<cast::cdl::CASTTime> grabTimes;   ///< time stamps when Ipl images were captured			TODO Unused!

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);
  
  void init(const std::vector<int> &dev_nums) throw(std::runtime_error);
  void copyImage(const IplImage *iplImg, Video::Image &img) throw(std::runtime_error);

  void grabFramesInternal();   											// TODO unused
  void retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);
  virtual void retrieveHRFrames(std::vector<Video::Image> &frames);

protected:
  void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);

  
public:
  KinectVideoServer();
  virtual ~KinectVideoServer();
  
  virtual void grabFrames();											// TODO unused
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
  virtual void changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize);
  virtual bool inFormat7Mode();
  virtual const std::string getServerName();

  // void enableUndistortion(const float dist[4]);
  // void disableUndistortion();
};

}

#endif

