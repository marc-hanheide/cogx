/**
 * @file KinectSeqServer.cpp
 * @author Richtsfeld Andreas
 * @date July 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor, playing pcl-recorded streams.
 */

#ifndef KINECT_SEQ_SERVER_H
#define KINECT_SEQ_SERVER_H

#include <vector>
#include <sys/time.h>

#include "PointCloudServer.h"
#include "VideoUtils.h"
#include <cast/core/CASTTimer.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <sensor_msgs/PointCloud2.h>

namespace cast
{

typedef union
{
  struct
  {
    unsigned char b;  // Blue channel
    unsigned char g;  // Green channel
    unsigned char r;  // Red channel
    unsigned char a;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


struct CSequenceInfo
{
  // List of filename templates, one for each camera.
  // The list is space-delimited to be compatible with --files in configure().
  std::string fileTemplates;

  // Start index, end index, step.
  int start;
  int end;
  int step;

  // True, if the sequence should play continuously.
  // TODO: make int to add bidirectional looping; rename to loopmode, add loopdirection
  bool loop;

  // How many times to repeat each frame.
  // Framerate should be set in configure().
  int repeatFrame;

  // Integer factor for downsampling.
  // NOTE: ignored for now!
  int downsampleFactor;

  CSequenceInfo();
  void clear();
  void checkLimits();
  void parseConfig(const std::map<std::string,std::string> & _config);
  void parseConfig(const std::string& _configStr);
  void setInfo(Video::VideoSequenceInfo& info);
};

  
/**
 * @brief Playing recorded Kinect sequences.
 */
class KinectSeqServer : public PointCloudServer
{
  
private:
  bool noContinousGrabing;                      ///< grab the frames not continously with framerateMillis: new frame after every call to getPoints()
  bool haveImages;                              ///< we have already images (point clouds)
  std::vector<std::string> filenames;           ///< filenames of the stored images

  int framerateMillis;                          ///< framerate in milliseconds
  bool m_displayImage;                          ///< display image

  IplImage* grabbedImage;                       ///< pointer to grabbed image
  IplImage* scaledImage;                        ///< pointer to scaled image
  pcl::PointCloud<pcl::PointXYZ/*RGB*/> pcl_cloud;  ///< pcl point cloud with color   TODO Turned color of, to load pcd-files without color: Take instead color from image!

  std::vector<cast::cdl::CASTTime> grabTimes;   ///< time stamps when Ipl images were captured.
  int width, height;                            ///< width and heigth of the saved images 

  /// SEQUENCE MAPS
  int frameCnt;
  bool loopSequence;                            ///< Whether to loop image sequence or return empty images at end of sequence.
  int frameRepeatCnt;                           ///< Number of times each frame is repeated; default = 1
  int frameRepeatPos;

  IceUtil::Monitor<IceUtil::Mutex> m_sequenceMonitor;   ///< This monitor is used to sync grabFramesInternal and switchSequence threads.

  std::map<std::string, CSequenceInfo> leadInMap;
  std::map<std::string, CSequenceInfo> sequenceMap;
  std::map<std::string, CSequenceInfo> leadOutMap;
  std::string m_currentSequenceName;
  std::string m_nextSequenceName;
  enum SequenceStage {
    stLeadIn, stLoop, stLeadOut
  };
  SequenceStage m_sequenceStage;

  void initFileTemplates(const std::vector<std::string> &fileTemplates, int first, int last, int inc) throw(std::runtime_error);
  void switchSequence(CSequenceInfo& seq);
  void installSequence(const std::string& name);
  void tryNextSequence();
  /// SEQUENCE MAPS END
  
  int numFiles;                                 ///< Number of filenames (numCams + kinect = 2)
  int getNumFiles() {return numFiles;}          ///< One more than cams (kinect point cloud)
  int numFrames() {return filenames.size();}    ///< Number of frames

  void constructFilenames(const vector<string> &fileTemplates, int first, int last, int inc);
  void obtainImageSize() throw(std::runtime_error);
  void grabFramesInternal();

protected:
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW               /// for 32-bit systems for pcl mandatory

  KinectSeqServer();
  virtual ~KinectSeqServer();
  
  // *********************************** Point Cloud Server *********************************** //
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  void getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth);
  void getRangePoints(Laser::Scan2d &KRdata);
  void receiveImages(const std::vector<Video::Image>& images);
  bool getCameraParameters(Ice::Int side, Video::CameraParameters& camPars);;
  void saveNextFrameToFile();
};

}

#endif

