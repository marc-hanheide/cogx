/**
 * @file StereoDetector.h
 * @author Andreas Richtsfeld, Michael Zillich
 * @date October 2009, 2010
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
#include "Gestalt.hh"
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
	bool debug;															///< Debug mode																				/// TODO gibt es noch einen Debug mode?
	bool single;														///< Single shot mode for the stereo detector
	int detail;															///< Degree of detail for showing features
	bool showImages;												///< Show stereo images in openCV window
	bool showDetected;											///< Show detected features in stereo images
	bool showSingleGestalt;									///< Show single Gestalts
	int showID;															///< ID of single Gestalt to show
	bool showAllStereo;											///< Show all stereo features
	bool showMasked;												///< Show masked features in stereo images
	bool showMatched;												///< Show matched features in stereo images
	std::vector<std::string> objectIDs;			///< IDs of the currently stored visual objects
	Z::Gestalt::Type showType;							///< Show this type of Gestalt
	int overlays;														///< Identifier for result overlays

 
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
	void processImage();
	void ShowImages(bool convertNewIpl);
	void WriteVisualObjects();
	void WriteToWM(Z::StereoBase::Type type);
	void SingleShotMode();

	void MouseEvent();

	void ReadSOIs();
	void DeleteVisualObjectsFromWM();
// 	void MouseHandler(int event, int x, int y, int flags, void* param);
// 	void RightMouseHandler(int event, int x, int y, int flags, void* param);


protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  StereoDetector() {}
  virtual ~StereoDetector();
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



