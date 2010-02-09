/**
 * @file StereoDetector.h
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 */

#ifndef STEREO_DETECTOR_H
#define STEREO_DETECTOR_H

#include "DoxyMain.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <VideoUtils.h>
#include <vector>

#include "StereoCore.hh"
#include "Pose3.h"
#include "StereoBase.h"
#include "Except.hh"

namespace cast
{

/**
	* @class StereoDetector
	* @brief Implementation of the stereo detector as cast component.
	*/
class StereoDetector : public VideoClient, public ManagedComponent
{
private:

	Z::StereoCore *score;										///< Stereo core
	std::vector<int> camIds;								///< Which cameras to get images from
	std::string videoServerName;						///< Component ID of the video server to connect to
	std::string camconfig;									///< Config name of camera config file
	Video::VideoInterfacePrx videoServer;		///< ICE proxy to the video server
	Video::Image image_l, image_r;					///< Left and right stereo image from video server. Original images.
	IplImage *iplImage_l, *iplImage_r;			///< Converted left and right stereo images (openCV ipl-images)
	bool gotImage;													///< True, when image is available
	bool cmd_detect;												///< Detection command
	bool cmd_single;												///< Single detection commmand
	bool debug;															///< Debug mode
	bool single;														///< Single shot mode for the stereo detector
	bool showImages;												///< Show stereo images in openCV window
	bool showDetected;											///< Show detected features in stereo images
	bool showMasked;												///< Show masked features in stereo images
	bool showMatched;												///< Show matched features in stereo images
	std::vector<std::string> objectIDs;			///< IDs of the currently stored visual objects
	int VOtoWrite; 													///< Identifier for visual objects to write (==> same as overlays)
	int overlays;														///< Identifier for result overlays:\n
																					///     1 = all\n
																					///     2 = flaps\n
																					///     3 = rectangles\n
																					///     4 = closures\n
																					///     5 = ellipses
																					///     6 = flaps_ari
																					///     7 = cubes

  /**
   * @brief Show both stereo images in the openCV windows.
   * @param convertNewIpl Convert image again into iplImage to delete overlays.
   */
	void ShowImages(bool convertNewIpl);

	/**
	 * @brief Delete working memory and (re)write different visual objects from the stereo detector.
	 * @param VOtoWrite Identifier to write
	 */
	void WriteVisualObjects(int VOtoWrite);

	/**
	 * @brief Write visual objects of different type to the working memory
	 * @param type Type to write
	 */
	void WriteToWM(Z::StereoBase::Type type);

  /**
   * @brief Receive a changed detection command, written to the working memory
   * @param wmc Working memory address. 
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

  /**
   * @brief Call single shot mode for debugging.
   */
	void SingleShotMode();

  /**
   * @brief Read the SOIs from the working memory and display it.
   */
	void ReadSOIs();

	/**
	 * @brief Delete all visual objects from the working memory. 
	 * The IDs are stored in the vector "objectIDs".
	 */
	void DeleteVisualObjectsFromWM();


protected:
  /**
   * @brief Called by the framework to configure our component
   * @param config Config TODO
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
	virtual void processImage();


public:
  StereoDetector() {}
  virtual ~StereoDetector();

  /**
   * @brief The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   * @param images Vector of images from video server.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



