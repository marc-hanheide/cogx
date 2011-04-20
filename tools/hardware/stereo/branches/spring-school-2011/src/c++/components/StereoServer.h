/**
 * @author Michael Zillich
 * @date June 2009
 */

#ifndef STEREO_SERVER_H
#define STEREO_SERVER_H

#include <stdexcept>
#include <vector>
#include <map>
#include <string>
#include <opencv/cv.h>
#include <cast/core/CASTComponent.hpp>
#include <VideoClient.h>
#include "StereoCamera.h"
#include "Math.hpp"
#include "Stereo.hpp"
#ifdef HAVE_GPU_STEREO
#include "gpustereo/CensusGPU.h"
#endif

namespace cast
{

class StereoServer;

/**
 * Ice interface to a stereo server.
 */
class StereoServerI : public Stereo::StereoInterface
{
private:
  StereoServer *stereoSrv;

public:
  StereoServerI(StereoServer *_stereo) : stereoSrv(_stereo) {}

  virtual void getPoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points, const Ice::Current&);

  // returns dense point cloud, with 0 values???
  virtual void getCompletePoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points, const Ice::Current&);

  virtual void getRectImage(Ice::Int side, int imgWidth, Video::Image& image, const Ice::Current&);

  virtual void getDisparityImage(int imgWidth, Video::Image& image, const Ice::Current&);
};

class StereoServer : public CASTComponent,
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
      for(int i = LEFT; i <= RIGHT; i++)
      {
        colorImg[i] = 0;
        rectColorImg[i] = 0;
        rectGreyImg[i] = 0;
      }
      disparityImg = 0;
    }
    ~ImageSet()
    {
      for(int i = LEFT; i <= RIGHT; i++)
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
      for(int i = LEFT; i <= RIGHT; i++)
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
   * Camera IDs for getting left and right images
   * this must be a vector of length 2 with camIds[LEFT] and camIds[RIGHT] the
   * ids of the left and right cameras respectively
   */
  std::vector<int> camIds;

  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;

  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;

  /**
   * the ICE stereo server instance
   */
  Stereo::StereoInterfacePtr hStereoServer;

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

  bool doDisplay;
  bool logImages;

  /**
   * Create Ice video interface.
   */
  void setupMyIceCommunication();

  void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);

  virtual void start();

  virtual void runComponent();

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

  /**
   * Returns the 3D point cloud.
   * @param transformToGlobal If true use the camera's pose to return points in global coordinates.
   *        Otherwise return in left camera coordinates.
   * @param imgWidth Specifies at which image resolution stereo matching should be performed.
   *        The nearest available resolution will be used.
   * @param points The 3D points with their color.
   * @param complete If false pixels with no available disparity are discarded. So there is
   *        no correspondence of the resulting point cloud and the rectangular image pixel grid.
   *        If true pixels with no available disparity will result in points with (0, 0, 0).
   *        The rectangular grid structure of points is thus maintained, which can help in finding
   *        nearest neighours etc.
   */
  void getPoints(bool transformToGlobal, int imgWidth, std::vector<VisionData::SurfacePoint> &points,
    bool complete);

  void getRectImage(int side, int imgWidth, Video::Image& image);

  void getDisparityImage(int imgWidth, Video::Image& image);

  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

