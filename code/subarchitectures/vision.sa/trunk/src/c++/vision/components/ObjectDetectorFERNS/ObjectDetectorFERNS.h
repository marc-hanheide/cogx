/**
 * @author Michael Zillich
 * @date February 2009
 *
 * This component just wraps the FERNS tracker by
 * Authors: Vincent Lepetit (http://cvlab.epfl.ch/~lepetit)
 *          Mustafa Ozuysal (http://cvlab.epfl.ch/~oezuysal)
 *          Julien  Pilet   (http://cvlab.epfl.ch/~jpilet)
 */

#ifndef OBJECT_DETECTOR_FERNS_H
#define OBJECT_DETECTOR_FERNS_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>
#include "mcv.h"
#include "planar_pattern_detector_builder.h"
#include "template_matching_based_tracker.h"

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{

class ObjectDetectorFERNS : public VideoClient,
                            public ManagedComponent
{
private:
  /**
   * different modes the FERNS stuff can operate in
   */
  enum Mode
  {
    DETECT_AND_TRACK = 0,  // Detect when tracking fails or for initialization
                           // then track.
    DETECT_ONLY            // Detect only
  };

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
   * mode (detection, tracking, detection and tracking ...) for the FERNS stuff
   */
  Mode mode;
  /**
   * full pathnames of model images
   */
  std::vector<std::string> model_images;
  /**
   * just filename bases (without path and without suffix like .jpg)
   */
  std::vector<std::string> model_labels;
  /**
   * FERNS based detector of planar objects
   */
  std::vector<planar_pattern_detector *> detectors;
  /**
   * a template matching based tracker, to be used after successful detection
   */
  std::vector<template_matching_based_tracker *> trackers;
  /**
   * remember whether we could track in the last frame
   */
  std::vector<bool> last_frame_ok;
  /**
   * WM Id of our tracked objects
   */
  std::vector<std::string> objWMIds;
  /**
   * Whether to open own (OpenCV) windows and display stuff
   * Default is false.
   */
  int doDisplay;
  /**
   * show quadrangle of detected/tracked planar object
   * Default is false.
   */
  bool show_tracked_locations;
  /**
   * show individual interest points
   * Default is false.
   */
  bool show_keypoints;

#ifdef FEAT_VISUALIZATION
  cogx::display::CDisplayClient m_display;
#endif

  /**
   * Display detection results.
   */
  void drawResults(IplImage * frame);
  /**
   * Load models for object detection/tracking.
   * When the models are used for the first time, a the detector needs to be
   * trained, which can take several minutes.
   * Subsequent uses of the trained models only require some dozen seconds to
   * load and decompress the stored trained models.
   */
  void setupFERNS() throw(std::runtime_error);
  /**
   * called internally by all detectObject methods
   * @param frame (in) image in which to detect the object
   * @param i (in) index of the model we want do detect
   */
  void detectObject_Internal(IplImage * frame, size_t i);
  /**
   * detect the given object
   * note: if the given object is not in our database, do nothing
   * @param frame (in) image in which to detect the object
   * @param label (in) label of the model we want do detect
   */
  void detectObject(IplImage * frame, const std::string & label);
  /**
   * detect the given list of objects
   * note: if the given objects are not in our database, do nothing
   * @param frame (in) image in which to detect the object
   * @param labels (in) labels of the models we want do detect
   */
  void detectObjects(IplImage * frame, const std::vector<std::string> & labels);
  /**
   * detect all objects for which we have models
   * @param frame (in) image in which to detect the object
   */
  void detectAllObjects(IplImage * frame);
  /**
   * used internally by all postObjectToWM methods
   * @param i (in) index of the model we want do detect
   * @param image (in) image in which we detected the object
   */
  void postObjectToWM_Internal(size_t i, const Video::Image &image);
  /**
   * post (add or overwrite) detected object data for the given object
   * @param label (in) label of the detected object
   * @param image (in) image in which we detected the object
   */
  void postObjectToWM(const std::string & label, const Video::Image &image);
  /**
   * post (add or overwrite) detected object data for the given list of objects
   * @param labels (in) label of the detected objects
   * @param image (in) image in which we detected the object
   */
  void postObjectsToWM(const std::vector<std::string> & labels,
       const Video::Image &image);
  /**
   * post (add or overwrite) detected object data for all objects
   * @param image (in) image in which we detected the object
   */
  void postAllObjectsToWM(const Video::Image &image);
  /**
   * Create a VisualObject from internal detector/tracker data.
   * @param i (in) index of the model we want do detect
   * @param image (in) image in which we detected the object
   */
  VisionData::VisualObjectPtr createVisualObject(size_t i,
      const Video::Image &image);
  /**
   * callback function called when a new detecion command is issued
   */
  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();

  virtual void runComponent();

public:
  ObjectDetectorFERNS();
  virtual ~ObjectDetectorFERNS();
};

}

#endif


