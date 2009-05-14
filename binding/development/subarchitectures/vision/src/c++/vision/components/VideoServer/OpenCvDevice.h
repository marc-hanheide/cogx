/**
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef OPEN_CV_DEVICE_H
#define OPEN_CV_DEVICE_H

#include <string>
#include <opencv/highgui.h>
#include <balt/interface/BALTTimer.hpp>
#include "VideoDevice.h"

/**
 * Video device simply using the OpenCV capture API.
 */
class OpenCvDevice : public VideoDevice
{
private:
  std::vector<int> cam_ids;
  std::vector<CvCapture*> captures;
  std::vector<IplImage*> latest_images;  // pointers to latest images, point to internal memory, do not delete
  int bayer_cvt;  // format for Bayer to RGB conversion, CV_COLORCVT_MAX for
                  // no conversion
  std::vector<FrameworkBasics::BALTTime> times;
  // bool undistort;
  // float dist_parms[4];

  void ConvertFrame(IplImage *src, Vision::ImageFrame *frame);
  bool HaveFrames();
  bool HaveBayer() {return bayer_cvt != CV_COLORCVT_MAX;}

public:
  OpenCvDevice(const std::vector<int> &ids, const std::vector<int> &dev_nums,
      const std::string &bayer);
  virtual ~OpenCvDevice();
  virtual void GrabFrames();
  virtual void RetrieveFrames(std::vector<Vision::ImageFrame*> &frames);
  virtual void RetrieveFrame(int camNum, Vision::ImageFrame *frame);
  // virtual void RetrieveFrame(int camNum, FrameworkBasics::BALTTime time,
  //   Vision::ImageFrame *frame);
  virtual int NumCameras();
  virtual void GetCameraIds(std::vector<int> &ids);
  virtual void GetImageSize(int &width, int &height);
  virtual int GetFramerate();
  // void EnableUndistortion(const float dist[4]);
  // void DisableUndistortion();
};

#endif

