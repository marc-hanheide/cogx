/**
 * @file VideoServer.h
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2010, Februar 2009
 * @version 0.1
 * @brief Video server: Manage capturing of videos from different sources (PointGrey, OpenCV, ImgSequences, Kinect).
 */

#ifndef VIDEO_SERVER_H
#define VIDEO_SERVER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <stdexcept>
#include <vector>
#include <string>
#include <map>
#include <VideoUtils.h>
#include "Video.hpp"

namespace cast
{

class VideoServer;

/**
 * @brief Ice interface to a video server.
 */
class VideoServerI : public Video::VideoInterface
{
private:
  VideoServer *vidSrv;

public:
  VideoServerI(VideoServer *_vid) : vidSrv(_vid) {}

  /**
   * @brief Returns number of cameras this device manages.
   */
  virtual Ice::Int getNumCameras(const Ice::Current&);

  /**
   * @brief Returns the devices image size, must be > 0.
   * Note that all images have the same size (and colour format). This class is
   * not intended to manage video sources of different sizes/formats.
   */
  virtual void getImageSize(Ice::Int& width, Ice::Int& height, const Ice::Current&);

  /**
   * @brief Fills the camera parameters for camera camId.
   * @return Returns true if the camera exists, false otherwise.
   */
  bool getCameraParameters(Ice::Int camId, Video::CameraParameters& camPars, const Ice::Current&);

  /**
   * @brief Returns the frame rate in [ms], must be > 0.
   * Note that all video sources have the same frame rate.
   */
  virtual Ice::Int getFramerateMilliSeconds(const Ice::Current&);

  /**
   * @brief Get just image from one source.
   * @param camId  (in) which video source (camera)
   * @param image  (out) image
   */
  virtual void getImage(Ice::Int camId, Video::Image& image, const Ice::Current&);

  /**
   * @brief Get images from all sources.
   * @param images  (out) images
   */
  virtual void getImages(Video::ImageSeq& images, const Ice::Current&);

  /**
   * @brief Get images from all sources with different resolutions.
   * @param width Width of image
   * @param height Height of image
   * @param images  (out) images
   */
  virtual void getImages(Ice::Int width, Ice::Int height,
      Video::ImageSeq& images, const Ice::Current&);

  /**
   * @brief Get scaled images.
   * @param width Width of image
   * @param height Height of image
   * @param images Video images (sequence)
   */
  virtual void getScaledImages(Ice::Int width, Ice::Int height,
      Video::ImageSeq& images, const Ice::Current&);

  /**
   * @brief Get high resolution (HR) images.
   * @param images Video images (sequence)
   */
  virtual bool getHRImages(std::vector<Video::Image> &images, const Ice::Current&);

  /**
   * @brief Start receiving images from some sources.
   * @param receiverComponentId Name of receiver component
   * @param camIds  (in) which video sources (cameras)
   * @param width Width of image
   * @param height Height of image
   */
  virtual void startReceiveImages(const std::string& receiverComponentId,
      const std::vector<int> &camIds, Ice::Int width, Ice::Int height,
      const Ice::Current&);

  /**
   * @brief Stop receiving images.
   * @param receiverComponentId Name of receiver component
   */
  virtual void stopReceiveImages(const std::string& receiverComponentId,
      const Ice::Current&);

  /**
   * @brief Change the properties in the PointGreyServer for the Format7 mode.
   * @param width Image width
   * @param height Image height
   * @param offsetX Offset in x- direction for Format7 mode.
   * @param offsetY Offset in y- direction for Format7 mode.
   * @param mode Image grabbing mode for the Format7 mode.
   * @param fps Requested framerate [1/s]
   */
  virtual void changeFormat7Properties(Ice::Int width, Ice::Int height, Ice::Int offsetX, Ice::Int offsetY, Ice::Int mode, Ice::Int fps, const Ice::Current&);

  /**
   * @brief Returns true, if cameras are in Format7 mode.
   * @return Returns true, if PG cameras are in Format7 mode.
   */
  virtual bool inFormat7Mode(const Ice::Current&);

  /**
   * @brief Get name of video server.
   * @return Returns the name of the server.
   */
  virtual std::string getServerName(const Ice::Current&);
};




/**
 * @brief Video Server to capture images from one or several (typically two) sources (= cameras). \n
 *
 * (In most cases basically just a wrapper around OpenCV capture functions.)\n
 *
 * If several sources are used, they all have the same image format and \n
 * synchronisation between sources is handled by this class. \n
 * If several sources are used, calls to Grab() shall aways fill the \n
 * provided array in the same order. \n
 */
class VideoServer : virtual public ManagedComponent
{
protected:
  class ImageReceiver
  {
  public:
    /**
     * @brief Our ICE proxy to the video client
     */
    Video::VideoClientInterfacePrx videoClient;
    /**
     * @brief Component ID of the client
     */
    std::string receiverComponentId;
    /**
     * @brief Which images of which camera (id) to receive from.
     */
    std::vector<int> camIds;
    /**
     * @brief Image size of received images, 0 means the videos native image size
     */
    int imgWidth, imgHeight;
    /**
     * @brief Properties for Format7 mode for PointGrey cameras.
     */
    int offsetX, offsetY, mode, fps;
  };

  /**
   * @brief Camera IDs for the video sources: camIds[sourceNum]
   * e.g. camIds[0] = 3; camIds[1] = 4;
   */
  std::vector<int> camIds;

  /**
   * @brief Camera parameters for the video sources
   */
  std::vector<Video::CameraParameters> camPars;

  /**
   * @brief List of clients which have registered via some startReceiveImage() and want
   * to receive images from us
   */
  std::vector<ImageReceiver> imageReceivers;

  /**
   * @brief As is often the case with OpenCV capture devices, red and blue channel
   * might be swapped. Set this to true if needed to correct for that.
   */
  bool swapRB;

  /**
   * @brief Real transferred frames per second. 
   * TODO Only for PointGrey server.
   */
  int realFps;

  Video::CMilliTimer m_timer;

  /**
   * @brief Return the index of the camera for a given camera ID
   */
  size_t getCamIndex(int camId) throw(std::runtime_error)
  {
    for(size_t i = 0; i < camIds.size(); i++)
      if(camId == camIds[i])
        return i;
    throw std::runtime_error(exceptionMessage(__HERE__, "video has no camera %d", camId));
  }


  void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);
  virtual void configure(const std::map<std::string,std::string> & _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();


public:
  VideoServer() : swapRB(false) {}
  virtual ~VideoServer() {}

  /**
   * @brief Grabs frames from all sources and stores them internally, including timestamps. \n
   * This is typically very fast, little copying etc.
   */
  virtual void grabFrames() = 0;

  /**
   * @brief Retrieves previously grabbed frames for given camera IDs.
   * @param camIds (in) camera IDs
   * @param width (in) Width of image: If 0 then use native image size of video.
   * @param height (in) Height of image.
   * @param frames  (out) Array of images, as many as size of camIds, including timestamps
   */
  virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames) = 0;

  /**
   * @brief Retrieves previously grabbed frames from all sources.
   * @param width (in) Width of image: If 0 then use native image size of video.
   * @param height (in) Height of image.
   * @param frames  (out) array of images, as many as num cameras, including timestamps
   */
  virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames) = 0;

  /**
   * @brief Retrieve previously grabbed frame from just one source.
   * @param camId  (in) which video source (camera)
   * @param width (in) Width of image: If 0 then use native image size of video.
   * @param height (in) Height of image.
   * @param frame  (out) image, including timestamp
   */
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame) = 0;

  /**
   * @brief Retrieve previously grabbed frames.
   * @param frames (out) images, including timestamp
   */
  virtual void retrieveHRFrames(std::vector<Video::Image> &frames) = 0;

  /**
   * @brief Returns number of cameras this device manages.
   * @return Returns the number of cameras
   */
  int getNumCameras() const;

  /**
   * @brief Returns the devices image size, must be > 0. \n
   * Note that all images have the same size (and colour format). This class is \n
   * not intended to manage video sources of different sizes/formats. \n
   * Note: not const (as one might assume) as we don't know what derived \n
   * classes will have to do internally.
   */
  virtual void getImageSize(int &width, int &height) = 0;

  /**
   * @brief Fills the camera parameters for camera camId.
   * @return Returns true if the camera exists, false otherwise.
   */
  virtual bool getCameraParameters(int camId, Video::CameraParameters& camPars);

  /**
   * @brief Returns the frame rate in [ms], must be > 0. \n
   * Note that all video sources have the same frame rate. \n
   * Note: not const (as one might assume) as we don't know what derived \n
   * classes will have to do internally.
   */
  virtual int getFramerateMilliSeconds() = 0;

  /**
   * @brief Change the properties in the PointGreyServer for the Format7 mode.
   * @param width Image width
   * @param height Image height
   * @param offsetX Offset in x- direction for Format7 mode.
   * @param offsetY Offset in y- direction for Format7 mode.
   * @param mode Image grabbing mode for the Format7 mode.
   * @param fps Requested framerate [1/s]
   */
  virtual void changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize);

  /**
   * @brief Checks if the PointGreyServer is active and in Format7 mode.
   * @return Returns true if PointGreyServer is active and in Format7 mode.
   */
  virtual bool inFormat7Mode();

  /**
   * @return Returns the server name (PointGreyServer / OpenCvImgSeqServer / OpenCvLiveServer)
   */
  virtual const std::string getServerName() = 0;


  void getImage(int camId, Video::Image &img);
  void getImages(std::vector<Video::Image> &images);
  void getImages(int width, int height, std::vector<Video::Image> &images);                  /// TODO unneccessary (== getScaledImages)
  void getScaledImages(int width, int height, std::vector<Video::Image> &images);            /// TODO This is unneccessary!!!
  bool getHRImages(std::vector<Video::Image> &images);
  void startReceiveImages(const std::string &receiverComponentId, const std::vector<int> &camIds, int width, int height);
  void stopReceiveImages(const std::string &receiverComponentId);
};

}

#endif

