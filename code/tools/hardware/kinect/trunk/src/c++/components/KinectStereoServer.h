/**
 * @file KinectStereoServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor and the stereo setup.
 */

#ifndef KINECT_STEREO_SERVER_H
#define KINECT_STEREO_SERVER_H

#include <vector>
#include <sys/time.h>
#include <VideoClient.h>

#include "PointCloud.hpp"
#include "PointCloudServer.h"
#include "Kinect.h"
#include "Math.hpp"
#include "StereoCamera.h"

namespace cast
{

/**
 * @brief Kinect and stereo server.
 */
class KinectStereoServer : public PointCloudServer,
                           public VideoClient
{
private:

  /**
   * All the images needed for stereo processing
   */
  class ImageSet
  {
  public:
    IplImage *colorImg[2];
    IplImage *rectColorImg[2];
    IplImage *rectGreyImg[2];
    IplImage *disparityImg;

    ImageSet()
    {
      for(int i = 0; i <= 1; i++)	// left/right
      {
        colorImg[i] = 0;
        rectColorImg[i] = 0;
        rectGreyImg[i] = 0;
      }
      disparityImg = 0;
    }
    ~ImageSet()
    {
      for(int i = 0; i <= 1; i++)	// left/right
      {
        cvReleaseImage(&colorImg[i]);
        cvReleaseImage(&rectColorImg[i]);
        cvReleaseImage(&rectGreyImg[i]);
      }
      cvReleaseImage(&disparityImg);
    }
    /**
     * dispFormat can be IPL_DEPTH_32F or IPL_DEPTH_16S
     */
    void init(CvSize size, int dispFormat)
    {
      for(int i = 0; i <= 1; i++)	// left/right
      {
        colorImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 3);
        rectColorImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 3);
        rectGreyImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 1);
      }
      assert(dispFormat == IPL_DEPTH_32F || dispFormat == IPL_DEPTH_16S);
      disparityImg = cvCreateImage(size, dispFormat, 1);
    }
  };
  
  std::string kinectConfig;                     ///< Kinect configuration file
  CvSize captureSize;                           ///< Size of captured images from kinect
  Kinect::Kinect *kinect;                       ///< The kinect hardware interface.
//  std::vector<IplImage*> retrievedImages;       ///< pointers to retrieved images for each device (dev_nums)

  int framerateMillis;                          ///< framerate in milliseconds 					TODO Unused
//   Timer timer;                                 ///< Timer to measure actual frame rate.
//  cast::cdl::CASTTime grabTime;   ///< time stamps when Ipl images were captured				TODO Unused, because no grabFrames()


//  std::vector<int> camIds;                      ///< Camera IDs for getting left and right images  /// TODO shifted to PointCloudServer
  std::string videoServerName;                  ///< component ID of the video server to connect to
  Video::VideoInterfacePrx videoServer;         ///< our ICE proxy to the video server
  std::vector<CvSize> stereoSizes;              ///< We offer different resolutions for high speed / low accuracy and vice versa
  std::vector<int> maxDisps;                    ///< maximum disparity range we want to search, for each resolution we offer
  std::vector<StereoCamera*> stereoCams;        ///<  Stereo parameters, one for each resolution we offer
  std::vector<ImageSet> imgSets;                ///< Sets of images (rectified, unrectified, disparity), one for each resolution we offer

#ifdef HAVE_GPU_STEREO
  CensusGPU *census;                            ///< The GPU stereo matching code
  int medianSize;                               ///< Size of median filter for specle removeal in the disparity image. (0 = not median filtering)
#endif

  bool doDisplay;                               ///< Display the stereo images
  bool logImages;                               ///< Log stereo images

  int findClosestResolution(int imgWidth);
  void stereoProcessing(StereoCamera *stereoCam, ImageSet &imgSet, const std::vector<Video::Image>& images);

  void getResolution(int camIdx, CvSize &size);
  bool setResolution(int camIdx, CvSize &size);
  void init() throw(std::runtime_error);

protected:
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  void start();
//   void runComponent();

  
public:
  KinectStereoServer();
  virtual ~KinectStereoServer();
  
  // *********************************** Point Cloud Server *********************************** //
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  bool getCameraParameters(Ice::Int side, Video::CameraParameters& camPars);
  void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

