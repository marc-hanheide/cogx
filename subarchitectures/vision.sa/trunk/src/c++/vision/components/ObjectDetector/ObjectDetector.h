/**
 * @file ObjectDetector.h
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Management component for running simple object detector (vs3)
 */

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include <Vs3Interface.h>
#include <VisionUtils.h>
#include <Pose3.h>


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
   *	Interface for working with vision system 3 (vs3)
   */
  Vs3Interface *vs3Interface;

  unsigned detail;					///< Degree of detail for drawing
  unsigned type;						///< Gestalt type for drawing

  bool cmd_detect;					///< detection command
	int num_cubes;						///< number of cubes
  unsigned frame_counter;		///< counter of image frames

  /**
   * Receive a changed detection command, written to the working memory
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

  /**
   * Receive the changed visual objects, written to the working memory from the tracker
   */
	void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

  /**
   * Convert a object-detector cube to a cast visual object 
   */
	bool Cube2VisualObject(VisionData::VisualObjectPtr &obj, Z::CubeDef &cd);

  /**
   * Extract camera parameters from video server.
   */
	bool GetCameraParameter(const Video::Image & image);


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
   * Called by the framework to start compnent run loop
   */
  virtual void runComponent();

  /**
   * Called to start processing of one image
   */
	virtual void processImage();


public:
  ObjectDetector() : camId(0) {}
  virtual ~ObjectDetector() {}
};

}

#endif



