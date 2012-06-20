/**
 * @author Michael Zillich
 * @date June 2009
 */

#ifndef STEREO_CLIENT_H
#define STEREO_CLIENT_H

#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <cast/core/CASTComponent.hpp>
#include "Stereo.hpp"

namespace cast
{

/**
 * Client to a StereoServer.
 * Inherit from this class if You want to connect to stereo servers.
 * You will have to call configureServerCommunication() and
 * startServerCommunication() in that order from somewhere in Your code,
 * probably from the configure() and start() methods of Your CAST component.
 */
class StereoClient
{
private:
  std::string stereoServerHost;
  std::string stereoServerName;
  int stereoServerPort;
  Stereo::StereoInterfacePrx stereoServer;

protected:
  void startStereoCommunication(CASTComponent &owner) throw(std::runtime_error);

  void configureStereoCommunication(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

public:
  StereoClient();

  /**
   * Returns the 3D point cloud.
   */
  void getPoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points);

  void getCompletePoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points);

  void getRectImage(int side, int imgWidth, Video::Image& image);

  void getDisparityImage(int imgWidth, Video::Image& image);
};

}

#endif


