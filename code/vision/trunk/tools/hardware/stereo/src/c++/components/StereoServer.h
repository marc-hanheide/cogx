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
  virtual void getPoints(Stereo::Vector3Seq& points, const Ice::Current&);
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
  void getPoints(std::vector<cogx::Math::Vector3> &points);
};

}

#endif

