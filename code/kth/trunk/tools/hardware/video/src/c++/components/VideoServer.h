/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef VIDEO_SERVER_H
#define VIDEO_SERVER_H

#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <cast/core/CASTComponent.hpp>
#include <VideoUtils.h>
#include "Video.hpp"

namespace cast
{

class VideoServer;

/**
 * Ice interface to a video server.
 */
class VideoServerI : public Video::VideoInterface
{
private:
  VideoServer *vidSrv;

public:
  VideoServerI(VideoServer *_vid) : vidSrv(_vid) {}

  /**
   * Returns number of cameras this device manages.
   */
  virtual Ice::Int getNumCameras(const Ice::Current&);

  /**
   * Returns the devices image size, must be > 0.
   * Note that all images have the same size (and colour format). This class is
   * not intended to manage video sources of different sizes/formats.
   */
  virtual void getImageSize(Ice::Int& width, Ice::Int& height, const Ice::Current&);

  /**
   * Returns the frame rate in [ms], must be > 0.
   * Note that all video sources have the same frame rate.
   */
  virtual Ice::Int getFramerateMilliSeconds(const Ice::Current&);

  /**
   * Get just image from one source.
   * \param camId  (in) which video source (camera)
   * \param frame  (out) image
   */
  virtual void getImage(Ice::Int camId, Video::Image& image, const Ice::Current&);

  /**
   * Get images from all sources.
   * \param frames  (out) images
   */
  virtual void getImages(Video::ImageSeq& images, const Ice::Current&);
};

/**
 * Video Server to capture images from one or several (typically two) sources
 * (= cameras).
 *
 * (In most cases basically just a wrapper around OpenCV capture functions.)
 *
 * If several sources are used, they all have the same image format and
 * synchronisation between sources is handled by this class.
 * If several sources are used, calls to Grab() shall aways fill the
 * provided array in the same order.
 */
class VideoServer : virtual public CASTComponent
{
private:
  /**
   * Ice (servant) name for video interface
   */
  std::string iceVideoName;
  /**
   * Ice port for video interface
   */
  int iceVideoPort;

  /**
   * Create Ice video interface.
   */
  void setupMyIceCommunication();

protected:
  /**
   * Camera IDs for the video sources: camIds[sourceNum]
   * e.g. camIds[0] = 3; camIds[1] = 4;
   */
  std::vector<int> camIds;

  /**
   * Camera parameters for the video sources
   */
  std::vector<Video::CameraParameters> camPars;

public:
  VideoServer();
  virtual ~VideoServer() {}

  void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

  virtual void start();

  virtual void runComponent();

  /**
   * Grabs frames from all sources and stores them internally, including
   * timestamps.
   * This is typically very fast, little copying etc.
   */
  virtual void grabFrames() = 0;

  /**
   * Retrieves previously grabbed frames from all sources.
   * \param frames  (out) array of images, as many as num cameras,
   *                      including timestamps
   */
  virtual void retrieveFrames(std::vector<Video::Image> &frames) = 0;

  /**
   * Retrieve just frame from one source.
   * \param camId  (in) which video source (camera)
   * \param frame  (out) image, including timestamp
   */
  virtual void retrieveFrame(int camId, Video::Image &frame) = 0;

  /**
   * Returns number of cameras this device manages.
   * Note: not const (as one might assume) as we don't know what derived
   * classes will have to do internally.
   */
  virtual int getNumCameras() = 0;

  /**
   * Returns the devices image size, must be > 0.
   * Note that all images have the same size (and colour format). This class is
   * not intended to manage video sources of different sizes/formats.
   * Note: not const (as one might assume) as we don't know what derived
   * classes will have to do internally.
   */
  virtual void getImageSize(int &width, int &height) = 0;

  /**
   * Returns the frame rate in [ms], must be > 0.
   * Note that all video sources have the same frame rate.
   * Note: not const (as one might assume) as we don't know what derived
   * classes will have to do internally.
   */
  virtual int getFramerateMilliSeconds() = 0;
};

}

#endif

