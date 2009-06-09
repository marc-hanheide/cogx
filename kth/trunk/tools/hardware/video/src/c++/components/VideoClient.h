/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef VIDEO_CLIENT_H
#define VIDEO_CLIENT_H

#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <cast/core/CASTComponent.hpp>
#include "Video.hpp"

namespace cast
{

/**
 * Client to a VideoServer.
 * Inherit from this class if You want to connect to video servers.
 * You will have to call configureServerCommunication() and
 * startServerCommunication() in that order from somewhere in Your code,
 * probably from the configure() and start() methods of Your CAST component.
 */
class VideoClient
{
private:
  std::string videoServerHost;
  std::string videoServerName;
  int videoServerPort;
  Video::VideoInterfacePrx videoServer;

protected:
  void startVideoCommunication(CASTComponent &owner) throw(std::runtime_error);

  void configureVideoCommunication(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

public:
  VideoClient();

  /**
   * Returns number of cameras this device manages.
   */
  int getNumCameras();

  /**
   * Returns the devices image size, must be > 0.
   * Note that all images have the same size (and colour format). This class is
   * not intended to manage video sources of different sizes/formats.
   */
  void getImageSize(int& width, int& height);

  /**
   * Returns the frame rate in [ms], must be > 0.
   * Note that all video sources have the same frame rate.
   */
  int getFramerateMilliSeconds();

  /**
   * Get just image from one source.
   * \param camId  (in) which video source (camera)
   * \param image (out) image, including timestamp
   */
  void getImage(int camId, Video::Image& image);

  /**
   * Get images from all sources.
   * \param images  (out) array of images, as many as num cameras,
   *                      including timestamps
   */
  void getImages(Video::ImageSeq& images);
};

}

#endif


