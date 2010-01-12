/**
 * @file StereoDetector.h
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 */

#ifndef STEREO_DETECTOR_H
#define STEREO_DETECTOR_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <VideoUtils.h>
#include <vector>

#include "StereoCore.hh"
#include "Pose3.h"
#include "StereoBase.h"

namespace cast
{

class StereoDetector : public VideoClient, public ManagedComponent
{
private:

	/**
	 * @brief Stereo core
	 */
  Z::StereoCore *score;

  /**
   * @brief Which camera to get images from
   */
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
	 * @brief Left and right stereo image from video server. Original images.
	 */
  Video::Image image_l, image_r;
// 	IplImage *iplImage_l, *iplImage_r, *iplImage_org_l, *iplImage_org_r;

  /**
   * @brief Receive a changed detection command, written to the working memory
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

  /**
   * @brief Extract camera parameters from video server.
   */
	bool GetCameraParameter(const Video::Image & image);

  /**
   * @brief If debug option is set, stop processing and wait for keybord input.
   */
	bool ProcessDebugOptions(Z::StereoCore *score, IplImage *iplImage_l, IplImage *iplImage_r, IplImage *iplImage_results);

  /**
   * @brief Read the SOIs from the working memory and display it.
   */
	void ReadSOIs();

	/**
	 * @brief Delete all visual objects from the working memory. 
	 * The IDs are stored in the vector "objectIDs".
	 */
	void DeleteVisualObjects();

  bool cmd_detect;											///< detection command
	bool cmd_single;											///< single detection commmand
	bool showImages;											///< show openCV image
	bool debug;														///< debug the stereo detector

	std::vector<std::string> objectIDs;		///< IDs of the currently stored visual objects


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
  StereoDetector() {}
  virtual ~StereoDetector();

  /**
   * @brief The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



