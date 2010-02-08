/**
 * @file ObjectDetector.h
 * @author Andreas Richtsfeld
 * @date April 2009
 * @version 0.1
 * @brief Management component for running the basic object detector (vs3)
 */

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <Vs3Interface.h>
#include <VisionUtils.h>
#include <Pose3.h>
#include <CylinderDefinition.hh>

namespace cast
{

class ObjectDetector : public VideoClient,
                      public ManagedComponent
{
private:
  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;

	/**
	 *	Image from video server
	 */
  Video::Image m_image;

  /**
   *	Interface for working with vision system 3 (vs3)
   */
  Vs3Interface *vs3Interface;

  unsigned detail;					///< Degree of detail for drawing
  unsigned type;						///< Gestalt type for drawing

  bool cmd_detect;					///< detection command
	bool cmd_single;					///< single detection command
	int num_cubes;						///< number of cubes, written to working memory
	int num_cylinders;				///< number of cylinders, written to working memory
  unsigned frame_counter;		///< counter of image frames

  /**
   * Receive a changed detection command, written to the working memory
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

  /**
   * Convert a object-detector cube to a cast visual object 
   */
	bool Cube2VisualObject(VisionData::VisualObjectPtr &obj, Z::CubeDef &cd);

  /**
   * Extract camera parameters from video server.
   */
	bool GetCameraParameter(const Video::Image & image);

  /**
   * Get cylinders as visual object.
   */
	void GetCylinders();


protected:
  /**
   * Called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * Called by the framework after configuration, before run loop
   */
  virtual void start();

  /**
   * Called to start processing of one image
   */
	virtual void processImage(const Video::Image &image);

public:
  ObjectDetector() : camId(0) {}
  virtual ~ObjectDetector() {}

  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

