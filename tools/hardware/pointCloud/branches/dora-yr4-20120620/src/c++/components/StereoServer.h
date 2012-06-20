/**
 * @file StereoServer.h
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the cast-framework implemented as stereo server.
 */

#ifndef STEREO_SERVER_H
#define STEREO_SERVER_H

#include <cast/core/CASTComponent.hpp>
#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <VideoClient.h>
#include "StereoCamera.h"
#include "Math.hpp"
#include "PointCloud.hpp"
#include "PointCloudServer.h"

#ifdef HAVE_GPU_STEREO
#include "gpustereo/CensusGPU.h"
#endif

namespace cast
{

class StereoServer : public PointCloudServer,
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
      for(int i = cogx::Math::LEFT; i <= cogx::Math::RIGHT; i++)
      {
        colorImg[i] = 0;
        rectColorImg[i] = 0;
        rectGreyImg[i] = 0;
      }
      disparityImg = 0;
    }
    ~ImageSet()
    {
      for(int i = cogx::Math::LEFT; i <= cogx::Math::RIGHT; i++)
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
      for(int i = cogx::Math::LEFT; i <= cogx::Math::RIGHT; i++)
      {
        colorImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 3);
        rectColorImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 3);
        rectGreyImg[i] = cvCreateImage(size, IPL_DEPTH_8U, 1);
      }
      assert(dispFormat == IPL_DEPTH_32F || dispFormat == IPL_DEPTH_16S);
      disparityImg = cvCreateImage(size, dispFormat, 1);
    }
  };

  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;

  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;

  /**
   * We offer different resolutions for high speed / low accuracy and vice versa
   */
  std::vector<CvSize> stereoSizes;

  /**
   * maximum disparity range we want to search, for each resolution we offer
   */
  std::vector<int> maxDisps;

  /**
   * Stereo parameters, one for each resolution we offer
   */
  std::vector<StereoCamera*> stereoCams;

  /**
   * Sets of images (rectified, unrectified, disparity), one for each resolution we offer
   */
  std::vector<ImageSet> imgSets;

#ifdef HAVE_GPU_STEREO
  /**
   * The GPU stereo matching code.
   */
  CensusGPU *census;

  /**
   * Size of median filter for specle removeal in the disparity image.
   * 0 = not median filtering
   */
  int medianSize;
#endif

  bool doDisplay;   // display the stereo images
  bool logImages;   // log the images to the disk

  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  void runComponent();

  int findClosestResolution(int imgWidth);

  /**
   * the actual stereo processing
   * Note: we epect images to be of size 2 and image 0 to be the left
   * and image 1 to be the right image.
   */
  void stereoProcessing(StereoCamera *stereoCam, ImageSet &imgSet, const std::vector<Video::Image>& images);

public:
  StereoServer();
  virtual ~StereoServer();

  void getPoints(bool transformToGlobal, int imgWidth, std::vector<PointCloud::SurfacePoint> &points, bool complete);
  void getRectImage(int side, int imgWidth, Video::Image& image);
  void getDisparityImage(int imgWidth, Video::Image& image);
  bool getCameraParameters(int side, Video::CameraParameters& camPars);
  void receiveImages(const std::vector<Video::Image>& images);
  virtual bool isPointVisible(const cogx::Math::Vector3&);
};

}

#endif

