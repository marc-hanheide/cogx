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
#include <cast/architecture/ManagedComponent.hpp>
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

  /**
   * Get images from some sources.
   * \param camIds  (in) which video sources (cameras)
   * \param frames  (out) images
   */
  virtual void getScaledImages(Ice::Int width, Ice::Int height,
      Video::ImageSeq& images, const Ice::Current&);

  virtual void startReceiveImages(const std::string& receiverComponentId,
      const std::vector<int> &camIds, Ice::Int width, Ice::Int height,
      const Ice::Current&);

  virtual void stopReceiveImages(const std::string& receiverComponentId,
      const Ice::Current&);
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
class VideoServer : virtual public ManagedComponent
{
protected:
  class ImageReceiver
  {
  public:
    /**
     * our ICE proxy to the video client
     */
    Video::VideoClientInterfacePrx videoClient;
    /**
     * component ID of the client
     */
    std::string receiverComponentId;
    /**
     * image size of received images, 0 means the videos native image size
     */
    int imgWidth, imgHeight;
    /**
     * which images to receive
     */
    std::vector<int> camIds;
  };

  /**
   * Camera IDs for the video sources: camIds[sourceNum]
   * e.g. camIds[0] = 3; camIds[1] = 4;
   */
  std::vector<int> camIds;

  /**
   * Camera parameters for the video sources
   */
  std::vector<Video::CameraParameters> camPars;

  /**
   * list of clients which have registered via some startReceiveImage() and want
   * to receive images from us
   */
  std::vector<ImageReceiver> imageReceivers;

  /**
   * As is often the case with OpenCV capture devices, red and blue channel
   * might be swapped. Set this to true if needed to correct for that.
   */
  bool swapRB;

  /**
   * return the index of the camera for a given camera ID
   */
  size_t getCamIndex(int camId) throw(std::runtime_error)
  {
    for(size_t i = 0; i < camIds.size(); i++)
      if(camId == camIds[i])
        return i;
    throw std::runtime_error(exceptionMessage(__HERE__, "video has no camera %d", camId));
  }

  void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);

  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);

  virtual void start();

  virtual void runComponent();

public:
  VideoServer() : swapRB(false) {}
  virtual ~VideoServer() {}

  /**
   * Grabs frames from all sources and stores them internally, including
   * timestamps.
   * This is typically very fast, little copying etc.
   */
  virtual void grabFrames() = 0;

  /**
   * Retrieves previously grabbed frames for given camera IDs.
   * \param camIds (in) camera IDs
   * \param width (in) size of output image. If 0 then use native image size of
   *                   video.
   * \param height (in)
   * \param frames  (out) array of images, as many as size of camIds,
   *                      including timestamps
   */
  virtual void retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames) = 0;

  /**
   * Retrieves previously grabbed frames from all sources.
   * \param width (in) size of output image. If 0 then use native image size of
   *                   video.
   * \param height (in)
   * \param frames  (out) array of images, as many as num cameras,
   *                      including timestamps
   */
  virtual void retrieveFrames(int width, int height,
      std::vector<Video::Image> &frames) = 0;

  /**
   * Retrieve previously grabbed frame from just one source.
   * \param camId  (in) which video source (camera)
   * \param width (in) size of output image. If 0 then use native image size of
   *                   video.
   * \param height (in)
   * \param frame  (out) image, including timestamp
   */
  virtual void retrieveFrame(int camId, int width, int height,
      Video::Image &frame) = 0;

  /**
   * Returns number of cameras this device manages.
   */
  int getNumCameras() const;

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

  void getImage(int camId, Video::Image &img);
  void getImages(std::vector<Video::Image> &images);
  void getScaledImages(int width, int height, std::vector<Video::Image> &images);
  void startReceiveImages(const std::string &receiverComponentId,
      const std::vector<int> &camIds, int width, int height);
  void stopReceiveImages(const std::string &receiverComponentId);
};

}

#endif

