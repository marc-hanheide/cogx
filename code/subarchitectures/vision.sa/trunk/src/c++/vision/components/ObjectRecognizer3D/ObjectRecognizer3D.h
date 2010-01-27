 /**
 * @file ObjectRecognizer3D.h
 * @author Hannes Prankl, Thomas Mörwald (moerwald@acin.tuwien.ac.at)
 * @date Januray 2010
 * @version 0.1
 * @brief Learning and recognizing pose of Object using SIFT features (cooperates with ObjetTracker)
 * @namespace Tracking
 */
#ifndef OBJECT_RECOGNIZER_3D_H
#define OBJECT_RECOGNIZER_3D_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

#include "ObjectTrackerUtils.hpp"
#include "Tracker.h"
#include "ModelLoader.h"

namespace cast
{

class ObjectRecognizer3D : public ManagedComponent
{
private:

  std::string m_plyfile;
  
  /** @brief list of objects we want to have detected */
  std::vector<std::string> labels;
  
  /** @brief callback function called whenever a new object appears ore an object changes */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

protected:

  /** @brief called by the framework to configure our component */
  virtual void configure(const std::map<std::string,std::string> & _config);
  
  /** @brief called by the framework after configuration, before run loop */
  virtual void start();
  
  /** @brief called by the framework to start compnent run loop */
  virtual void runComponent();
 
public:
  virtual ~ObjectRecognizer3D() {}
};

}

#endif

