/**
 * @file Kinect.h
 * @author Richtsfeld Andreas
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the OpenNI Driver for the Kinect sensor for use inside of cast-framework
 */

#ifndef Z_KINECT_H
#define Z_KINECT_H

#include "Capture.h"

#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <IceUtil/IceUtil.h>

#define KINECT_CAST_LOGGING
#ifdef KINECT_CAST_LOGGING
#include <castutils/CastLoggerMixin.hpp>
#endif
#include <castutils/Timers.hpp>

namespace Kinect
{

struct KinectPerson
{
  int id;
  int distance;
  int size;
  KinectPerson(int _id, int _distance, int _size) {
    id = _id;
    distance = _distance;
    size = _size;
  }
};

class Kinect
#ifdef KINECT_CAST_LOGGING
  : public castutils::CCastLoggerMixin
#endif
{
private:
  bool ni_pause;                        ///< Set pause
  
  int rgbWidth;                         ///< Width of color image
  int rgbHeight;                        ///< Heigth of color image
  int depWidth;                         ///< Width of depth image
  int depHeight;                        ///< Heigth of depth image

  float centerX, centerY;               ///< center of the image -0.5f
  double pixel_size;                    ///< pixel size of the kinect camera
  XnUInt64 depth_focal_length_SXGA;     ///< 
  float depth_focal_length_SXGA_;       ///< 
  float depthScale;                     ///< 
  float depthFocalLength;               ///< 
  float constant;                       ///< constant factor for world point calculation (from focal length)

  XnUInt64 shadow_value;                ///< Return value for shadow point
  XnUInt64 no_sample_value;             ///< Return value for no sample

  // @author: mmarko
  // Lock before readFrame and global data access
  IceUtil::RWRecMutex m_kinectMutex;

  // @author: mmarko
  // Limit the number of calls to readFrame
  castutils::CMilliTimer m_grabTimer;
  int m_frameMilliseconds;

  // @author: mmarko
  // Palette for the RGB depth image
  std::vector<long> m_depthColors;            ///< Key colors to be used in the palette
  std::vector<double> m_depthColorRanges;     ///< Positions of key colors in the palette: 0.0 .. 1.0
  std::vector<unsigned char> m_depthPalette;  ///< Interpolated palette; b-g-r triplets

private:
  bool Init(const char *kinect_xml_file);
  void MapMetaData2IplImage(const MapMetaData* pImageMD, IplImage **iplImg);
  void DepthMetaData2IplImage(const DepthMetaData* pDepthMD, IplImage **iplImg);
  void rgbUndistort(cv::Mat src);
  void depUndistort(cv::Mat src);
  cv::Point3f DepthToWorld(int x, int y, int depthValue);
  cv::Point3f WorldToColorInternal(unsigned x, unsigned y);

  void calculateDepthPalette();

public:
  // @author: mmarko
  // Set the colors and positions from which the palette will be calculated.
  // Vectors rgbColors and positions have the same number of entries.
  // eg.: (0xff0000, 0x00ff00, 0x0000ff), (0.0, 0.4, 1.0)
  // At least two colors have to be given. If less than two colors are in rgbColors, the default
  // palette will be used.
  // The positions will be adjusted to the interval (0.0, 1.0). The sizes of
  // the vectors rgbColors and positions will be adjusted internally.
  void setDepthColors(const std::vector<long> &rgbColors, const std::vector<double> &positions);
  
public:
  Kinect(const char *kinect_xml_file);
#ifdef KINECT_CAST_LOGGING
  Kinect(cast::CASTComponent* pComponent, const char *kinect_xml_file);
#endif
  ~Kinect();

  void StartCapture(int delay);
  void StopCapture();
  bool GetFrame(IplImage **iplImg, IplImage **iplDepthImg);
  bool NextFrame();  
  bool GetColorImage(IplImage **rgbIplImg);
  bool GetDepthImageRgb(IplImage **rgbIplImg, bool useHsv=false);
  bool GetDepthImageGs(IplImage **gsIplImg);
  bool GetImages(cv::Mat &rgbImg, cv::Mat &depImg);
  int GetRgbImageWidth() {return rgbWidth;}
  int GetDepthImageWidth() {return depWidth;}
  bool GetDetectedPersons(std::vector<KinectPerson>& persons, double minRelativePersonArea=0);

private:
  void pullData();
  
private:
  cv::Mat grayImage;                    ///< captured gray image (bayer pattered)
  cv::Mat rgbImage;                     ///< captured rgb image
  cv::Mat depImage;                     ///< captured depth image

public:
  int frameNumber;

  const DepthMetaData* getNextDepthMD();

  std::pair<const DepthMetaData*, const ImageGenerator*> getNextFrame();
  cv::Point3f Get3dWorldPoint(unsigned x, unsigned y);
  cv::Point3f WorldToColor(unsigned x, unsigned y);
  void Get3dWorldPointCloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud, int scale = 1);
  void TogglePause() {ni_pause = !ni_pause;}
  void GetColorVideoSize(CvSize &size) {size = cvSize(rgbWidth, rgbHeight);}
  void GetDepthVideoSize(CvSize &size) {size = cvSize(depWidth, depHeight);}
};

class KinectFactory
{
  static std::map<int,Kinect*> s_kinects;
  // TODO: map of types (int, std::shared_ptr<Kinect>) would be better!
  // Instead of this we use KinectFactoryCleanup.
  friend class KinectFactoryCleanup;

public:
  // Get a Kinect object to communicate with the kinect device.
  // ATM the library can only work with one device, but multiple Kinect objects
  // can be created. This is not desired since the Kinect object implements
  // locking to serialize access to the device and using multiple objects to
  // use the same device will return bad data.
  //
  // When a new Kinect object is created it will be initialized from kinect_xml_file.
  static Kinect* getKinect(const char* kinect_xml_file, int deviceId=0);
#ifdef KINECT_CAST_LOGGING
  static Kinect* getKinect(cast::CASTComponent* pComponent, const char* kinect_xml_file, int deviceId=0);
#endif
};

/**
 * @brief Transform depth point to world coordinates.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param depthValue Raw depth value from the sensor.
 * @return Returns the 3d point in the world coordinate system.
 */
inline cv::Point3f Kinect::DepthToWorld(int x, int y, int depthValue)
{
  cv::Point3f result;
  // NOTE: the depth image was mapped to the color image, so we
  // have to use the intrinsic parameters of the color camera here.
  // These are the magic numbers 525, 320, 240
  result.x = (x - 320) * depthValue * 0.001 / 525;
  result.y = (y - 240) * depthValue * 0.001 / 525;
  result.z = depthValue * 0.001;
  return result;
}

/**
 * @brief Get the color for world points.
 * @param x x-coordinate in depth image
 * @param y y-coordinate in depth image
 * @return Returns the color as 3d point.
 */
inline cv::Point3f Kinect::WorldToColorInternal(unsigned x, unsigned y)
{
  uchar *ptr = rgbImage.data;
  cv::Point3f col;
  col.x = ptr[(y*rgbWidth + x)*3];
  col.y = ptr[(y*rgbWidth + x)*3 +1];
  col.z = ptr[(y*rgbWidth + x)*3 +2];
  return col;
}

}

#endif

