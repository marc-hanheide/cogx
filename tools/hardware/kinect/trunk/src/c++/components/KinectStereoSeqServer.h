/**
 * @file KinectStereoSeqServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor and the stereo setup.
 * Reads sequences of kinect and stereo data from files.
 */

#ifndef KINECT_STEREO_SEQ_SERVER_H
#define KINECT_STEREO_SEQ_SERVER_H

#include <vector>
#include <sys/time.h>

#include <cast/core/CASTComponent.hpp>
#include "PointCloud.hpp"
#include "PointCloudServer.h"
#include "Math.hpp"
#include "StereoCamera.h"
#include "FileReaderWriter.cpp"
#include "Video.hpp"

namespace cast
{

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
 * @brief Kinect and stereo sequence server.
 */
class KinectStereoSeqServer : public PointCloudServer
{
private:
  bool noContinousGrabing;                      ///< grab the frames not continously with framerateMillis: new frame after every call to getPoints()
  bool haveImages;                              ///< we have already images (point clouds)
  std::vector<std::string> filenames;           ///< filenames of the stored images

  int framerateMillis;                          ///< framerate in milliseconds

  /// all the grabbed/calculated images/clouds
  std::vector<IplImage*> grabbedImages;         ///< pointers to grabbed images for (left/right stereo and kinect image == 3)
  IplImage* scaledStereoImages[2];              ///< pointers to scaled stereo images
  IplImage* rectifiedStereoImages[2];           ///< pointers to rectified stereo images (also scaled!)
  IplImage* rectifiedGrayStereoImages[2];       ///< pointers to rectified stereo images (also scaled!)
  IplImage* disparityImg;                       ///< Disparity image (also scaled)
  cv::Mat_<cv::Point3f> cloud;                  ///< point cloud from the file TODO Change to cv::Vec4f matrix with color!
  cv::Mat_<cv::Point3f> colCloud;               ///< color values to the point cloud from the file

  /**
   * time stamps when Ipl images were captured.
   */
  std::vector<cast::cdl::CASTTime> grabTimes;
  int width, height;                            /// width and heigth of the saved images
  
  /// for stereoProcessing!
  std::vector<StereoCamera*> stereoCams;        ///<  Stereo parameters, one for each resolution we offer
  std::vector<int> maxDisps;                    ///< maximum disparity range we want to search, for each resolution we offer
  std::vector<CvSize> stereoSizes;              ///< We offer different resolutions for high speed / low accuracy and vice versa
  /// for stereoProcessing end!
  
  int numFiles;                                 ///< Number of filenames (numCams + kinect)
  int numCams;                                  ///< Number of camera images (stereo left/right and kinect = 3)
  int getNumFiles() {return numFiles;}          ///< One more than cams (kinect point cloud)
  int getNumCams() {return numCams;}
  int numFrames() {return filenames.size()/getNumFiles();}
    
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


#ifdef HAVE_GPU_STEREO
  CensusGPU *census;                            ///< The GPU stereo matching code
  int medianSize;                               ///< Size of median filter for specle removeal in the disparity image. (0 = not median filtering)
#endif

  bool doDisplay;                               ///< Display the stereo images
  bool logImages;                               ///< Log stereo images

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);
  int findClosestResolution(int imgWidth);
 
  void constructFilenames(const vector<string> &fileTemplates, int first, int last, int inc);
  void obtainImageSize() throw(std::runtime_error);
  void grabFramesInternal();

protected:

  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  void runComponent();
  virtual void start();

public:
  KinectStereoSeqServer();
  virtual ~KinectStereoSeqServer();
  
  // *********************************** Point Cloud Server Functions *********************************** //
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

