/**
 * @file VisionOntology.h
 * @brief header file for VisionOntology
 *  
 * @date February 2008
 */

#ifndef CAST_VISION_ONTOLOGY_H
#define CAST_VISION_ONTOLOGY_H

#include <cast/core/CASTCore.hpp>

class VisionOntologyFactory;

//default useful namespaces, fix to reflect your own code
using namespace std;

/**
 *  @class VisionOntology
 *  @brief Manages mappings between ontological types and framework datatypes.
 */
class VisionOntology: public cast::CASTOntology
{
public:

  /**  Ontological type to map to Vision::Camera  */
//  static const std::string CAMERA_TYPE;

  /**  Ontological type to map to Vision::Head  */
//  static const std::string HEAD_TYPE;

  /**  Ontological type to map to Vision::ImageFrame  */
//  static const std::string IMAGE_FRAME_TYPE;

  /**  Ontological type to map to Vision::SceneChanged  */
//  static const std::string SCENE_CHANGED_TYPE;

  /**  Ontological type to map to Vision::ROI  */
//  static const std::string ROI_TYPE;

  /**  Ontological type to map to Vision::SceneObject  */
//  static const std::string SCENE_OBJECT_TYPE;

  /**  Ontological type to map to Vision::Surface  */
//  static const std::string SURFACE_TYPE;

  /**  Ontological type to map to Vision::AttendedObjectList  */
//  static const std::string ATTENDED_OBJECT_LIST_TYPE;

  /**  Ontological type to map to Vision::HandPointingResults  */
//  static const string HAND_POINTING_RESULTS_TYPE;


 protected:
  VisionOntology();
  virtual ~VisionOntology(){};

  friend class VisionOntologyFactory;
};

#endif

