/**
 * @file StereoFlapDetector.h
 * @author Andreas Richtsfeld
 * @date September 2009
 * @version 0.1
 * @brief Detecting flaps with stereo for cogx implementation in cast.
 */

#ifndef STEREO_FLAP_DETECTOR_H
#define STEREO_FLAP_DETECTOR_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <VideoUtils.h>

#include "StereoCore.hh"
#include "Pose3.h"
// #include "Vector3.h"
#include "Vector3.hh"
#include "VecMath.hh"

namespace cast
{

class StereoFlapDetector : public VideoClient, public ManagedComponent
{
private:

	/**
	 * @brief Stereo core
	 */
  Z::StereoCore *score;

  /**
   * @brief Which camera to get images from
   */
//   int camId;
  std::vector<int> camIds;

  /**
   * @brief component ID of the video server to connect to
   */
  std::string videoServerName;

  /**
   * @brief our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;

	/**
	 * @brief Left and right stereo image from video server
	 */
  Video::Image image_l, image_r;

  /**
   * @brief Receive a changed detection command, written to the working memory
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

  /**
   * @brief Convert a flap to a visual object 
   */
	bool Flap2VisualObject(VisionData::VisualObjectPtr &obj, Z::Flap3D &f);

  /**
   * @brief Extract camera parameters from video server.
   */
	bool GetCameraParameter(const Video::Image & image);

  /**
   * @brief Define a flap coordinate system (center of gravity)
   */
	void DefineFlapCoordsys(Z::Flap3D &flap, Pose3 &pose);

  bool cmd_detect;						///< detection command
	bool cmd_single;						///< single detection command
	bool showStereoImage;				///< show openCV image

protected:
  /**
   * @brief Called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * @brief Called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * @brief Called by the framework to start compnent run loop
   */
  virtual void runComponent();

  /**
   * @brief Called to start processing of one image
   */
	virtual void processImage(const Video::Image &image_l, const Video::Image &image_r);


public:
  StereoFlapDetector() /*: camId(0)*/ {}
  virtual ~StereoFlapDetector() {}

  /**
   * @brief The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



