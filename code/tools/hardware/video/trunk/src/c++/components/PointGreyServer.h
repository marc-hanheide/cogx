/**
 * @file PointGrayServer.h
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2010, Februar 2009
 * @version 0.1
 * @brief Video server for the PointGray stereo cameras.
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
 * @brief Video device wrapping the Point Grey flycapture 2.0 API
 */
class PointGreyServer : public VideoServer
{
private:

	/**
	 * @brief Class SlidingMean?
	 */
  class SlidingMean
  {
  private:
    size_t window_size;												///< window size
    size_t i1, i2;														///< TODO 
    std::vector<float> values;								///< TODO
    float mean;																///< TODO

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

	/**
	 * @brief Class MeanRate
	 */
  class MeanRate
  {
  private:
    SlidingMean mean;																	///< TODO
    timeval prev;																			///< TODO

    float calculateCurrentRate();

  public:
    MeanRate(int size = 10);
    void insert() {mean.insert(calculateCurrentRate());}
    float getRate() const {return mean.getMean();}
  };


  FlyCapture2::BusManager busMgr;											///< FlyCapture2 bus manager
  FlyCapture2::Camera **cameras;											///< FlyCapture2 cameras
  std::vector<FlyCapture2::Image> retrievedImages;		///< FlyCapture2 retrieved images from different cameras
  std::vector<cast::cdl::CASTTime> grabTimes;					///< Time stamps when Ipl images were captured.
  MeanRate framerateMillis;														///< Measured mean frame rate
  int width;																					///< Image width
  int height;																					///< Image height
	int offsetX;																				///< Offset for VideoMode7
	int offsetY; 																				///< Offset for VideoMode7
  int fps;																						///< Frames per second
  int videoMode;																			///< Mode 0/1: pruned/resized
  int paketSize;																			///< Paket size for VideoMode7
	bool useVideoMode7;																	///< Use VideoMode7
  bool setAutomaticPropertyAdjustment;								///< Automatic property adjustment between different cameras.

  FlyCapture2::VideoMode selectVideoMode(int &_width, int &_height);
  FlyCapture2::FrameRate selectFrameRate(int &_fps);
  void init() throw(std::runtime_error);

  void LogCameraInfo(FlyCapture2::CameraInfo* pCamInfo);
	void LogCameraConfig();
	void LogPropertyInfo(FlyCapture2::PropertyInfo* pPropInfo);
	void GetPropertyInfo(int camId, FlyCapture2::PropertyType propType, FlyCapture2::PropertyInfo* pPropInfo);
	void LogProperty(FlyCapture2::Property* pProp);
	void LogPropertyValues(FlyCapture2::Property* pProp);
	void GetProperty(int camId, FlyCapture2::PropertyType propType, FlyCapture2::Property* pProp);
	void SetPropertyManual(int camId, FlyCapture2::PropertyType propType, bool manual);
	void SetPropertyValue(int camId, FlyCapture2::PropertyType propType, int valueA, int valueB, float absValue);
	void SetAutomaticPropertyAdjustment();
	void SetAllPropertiesManual();
	void CopyAllPropertyValues();

	void SetFormat7Properties(int w, int h, int offX, int offY, int vMode, int pSize);
	bool IsCurrentlyInFormat7(int camId);
	bool GetFormat7ImageParametersFromCamera(FlyCapture2::Mode mode, unsigned int* pLeft, unsigned int* pTop, unsigned int* pWidth, unsigned int* pHeight);
	void SetVideoMode7(int camId);

  void copyImage(const FlyCapture2::Image &flyImg, Video::Image &img) throw(std::runtime_error);
  void grabFramesInternal();
  void retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);

public:
  PointGreyServer();
  virtual ~PointGreyServer();
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void grabFrames();
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
	virtual void changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize);
};

}

#endif

