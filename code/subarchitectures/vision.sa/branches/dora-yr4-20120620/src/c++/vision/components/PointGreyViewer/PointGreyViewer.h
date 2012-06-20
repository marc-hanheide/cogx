/**
 * @file PointGreyViewer.h
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2010, Februar 2009
 * @version 0.1
 * @brief Just receives stereo images and displays them.
 * A dummy component showing how to get images. 
 */


#ifndef POINT_GRAY_VIEWER_H
#define POINT_GRAY_VIEWER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>

namespace cast
{

/**
 * @brief Class PointGrayViewer: Receives stereo images and displays them.
 */
class PointGreyViewer : public ManagedComponent,
                        public VideoClient
{
private:

	std::vector<int> camIds;								///< Which cameras to get images from
  /**
   * @brief component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * @brief our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  /**
   * @brief wether we are currently receiving images from the server
   */
  bool receiving;

	/**
	 * @brief Test getImage functions instead of pushing images with receiveImages().
	 */
	bool getImage;

protected:
  /**
   * @brief called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * @brief called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * @brief called by the framework upon deletion of the component
   */
  virtual void destroy();
  /**
   * @brief Our run loop. Essentially just wait for key strokes.
   */
  virtual void runComponent();

public:
  PointGreyViewer() {}
  virtual ~PointGreyViewer() { cvDestroyAllWindows();}
  /**
   * @brief The callback function for images pushed by the image server. \n
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

