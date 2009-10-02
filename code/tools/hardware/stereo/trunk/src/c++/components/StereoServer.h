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
  virtual void getPoints(VisionData::SurfacePointSeq& points, const Ice::Current&);

  /**
   * Returns part of the 3D point cloud inside given SOI.
   */
  virtual void getPointsInSOI(const VisionData::SOIPtr &soi,
      VisionData::SurfacePointSeq& points, const Ice::Current&);
};

class StereoServer : public VideoClient,
                     virtual public CASTComponent
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
   */
  std::vector<int> camIds;

  /**
   * Stereo parameters
   */
  StereoCamera stereoCam;

  /**
   * The GPU stereo matching code.
   */
  CensusGPU census;

  /**
   * Size of median filter for specle removeal in the disparity image.
   * 0 = not median filtering
   */
  int medianSize;

  IplImage *colorImg[2];
  IplImage *rectColorImg[2];
  IplImage *rectGreyImg[2];
  IplImage *disparityImg;

  bool doDisplay;

  /**
   * Create Ice video interface.
   */
  void setupMyIceCommunication();

public:
  StereoServer();
  virtual ~StereoServer();

  void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

  virtual void start();

  virtual void runComponent();

  /**
   * Returns the 3D point cloud.
   */
  void getPoints(std::vector<VisionData::SurfacePoint> &points);

  /**
   * Returns part of the 3D point cloud inside given SOI.
   */
  void getPointsInSOI(const VisionData::SOI &soi,
      std::vector<VisionData::SurfacePoint> &points);
};

}

#endif

