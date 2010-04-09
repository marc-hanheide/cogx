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
#include <cast/core/CASTComponent.hpp>
#include <VideoClient.h>
#include "StereoCamera.h"
#include "Math.hpp"
#include "Stereo.hpp"
#include "gpustereo/CensusGPU.h"

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

  /**
   * Returns the 3D point cloud.
   */
  virtual void getPoints(bool transformToGlobal, VisionData::SurfacePointSeq& points, const Ice::Current&);

  /**
   * Returns part of the 3D point cloud inside given SOI.
   */
  virtual void getPointsInSOI(bool transformToGlobal, const VisionData::SOIPtr &soi,
      VisionData::SurfacePointSeq& points, const Ice::Current&);

  virtual void getRectImage(Ice::Int side, Video::Image& image, const Ice::Current&);

  virtual void getDisparityImage(Video::Image& image, const Ice::Current&);
};

class StereoServer : public CASTComponent,
                     public VideoClient
{
private:
  /**
   * Ice (servant) name for video interface
   */
  std::string iceStereoName;
  /**
   * Ice port for video interface
   */
  int iceStereoPort;

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
   * Stereo parameters
   */
  StereoCamera stereoCam;

  /**
   * The GPU stereo matching code.
   */
  CensusGPU *census;

  // stereo works better/faster with smaller images, so we might want to use a
  // smaller resolution
  int stereoWidth;
  int stereoHeight;

  /**
   * Size of median filter for specle removeal in the disparity image.
   * 0 = not median filtering
   */
  int medianSize;

  /**
   * maximum disparity range we want to search
   */
  int maxDisp;

  IplImage *colorImg[2];
  IplImage *rectColorImg[2];
  IplImage *rectGreyImg[2];
  IplImage *disparityImg;

  bool doDisplay;
  bool logImages;

  /**
   * Create Ice video interface.
   */
  void setupMyIceCommunication();

  void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

  virtual void start();

  virtual void runComponent();

public:
  StereoServer();
  virtual ~StereoServer();

  /**
   * Returns the 3D point cloud.
   */
  void getPoints(bool transformToGlobal, std::vector<VisionData::SurfacePoint> &points);

  /**
   * Returns part of the 3D point cloud inside given SOI.
   */
  void getPointsInSOI(bool transformToGlobal, const VisionData::SOI &soi,
      std::vector<VisionData::SurfacePoint> &points);

  void getRectImage(int side, Video::Image& image);
  
  void getDisparityImage(Video::Image& image);

  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

