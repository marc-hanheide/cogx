/**
 * @author Michael Zillich
 * @date Februrary 2009
 */

#ifndef POINT_GREY_SERVER_H
#define POINT_GREY_SERVER_H

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/time.h>
#include <flycapture/FlyCapture2.h>
#include "VideoServer.h"

namespace cast
{

/**
 * Video device wrapping the Point Grey flycapture 2.0 API
 */
class PointGreyServer : public VideoServer
{
private:
  class SlidingMean
  {
  private:
    size_t window_size;
    size_t i1, i2;
    std::vector<float> values;
    float mean;

    size_t inc(size_t i) const
    {
      assert(values.size() > 0);
      return i < values.size() - 1 ? i + 1 : 0;
    }

  public:
    SlidingMean(int size = 10)
    {
      window_size = size;
      values.reserve(window_size);
      i1 = i2 = 0;
      mean = 0.;
    }
    void insert(float f)
    {
      // if window is full
      if(values.size() == window_size)
      {
        mean -= values[i1]/(float)values.size();
        i1 = inc(i1);
        i2 = inc(i2);
        values[i2] = f;
        mean += values[i2]/(float)values.size();
      }
      else
      {
        mean *= (float)values.size();
        values.push_back(f);
        // note: values.size() is guaranteed to be > 0 here
        i2 = values.size() - 1;
        mean += values[i2];
        mean /= (float)values.size();
      }
    }
    float getMean() const {return mean;}
  };

  class MeanRate
  {
  private:
    SlidingMean mean;
    timeval prev;

    float calculateCurrentRate();

  public:
    MeanRate(int size = 10);
    void insert() {mean.insert(calculateCurrentRate());}
    float getRate() const {return mean.getMean();}
  };

  FlyCapture2::BusManager busMgr;
  FlyCapture2::Camera **cameras;
  std::vector<FlyCapture2::Image> retrievedImages;

  /** time stamps when Ipl images were captured.  */
  std::vector<cast::cdl::CASTTime> grabTimes;

  /** measured mean frame rate */
  MeanRate framerateMillis;
  int width;
  int height;
  int fps;

  FlyCapture2::VideoMode selectVideoMode(int &_width, int &_height);
  FlyCapture2::FrameRate selectFrameRate(int &_fps);
  void init() throw(std::runtime_error);
  void LogCameraInfo(FlyCapture2::CameraInfo* pCamInfo);
  void copyImage(const FlyCapture2::Image &flyImg,
      Video::Image &img) throw(std::runtime_error);
  void grabFramesInternal();
  void retrieveFrameInternal(int camIdx, int width, int height,
      Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);

public:
  PointGreyServer();
  virtual ~PointGreyServer();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);
  virtual void grabFrames();
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
};

}

#endif

