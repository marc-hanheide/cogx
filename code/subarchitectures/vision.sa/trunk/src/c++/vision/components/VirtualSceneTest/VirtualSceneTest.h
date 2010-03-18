/* Dummy driver for the object tracker component
*
*  @author: Thomas Mörwald
*  @date: April 2009
*
*  This component is an example on how to control the
*  object tracker by loading a model from ply-file
*  to the working memory and calling several tracking commands.
*
*/

#ifndef VIRTUAL_SCENE_TEST_H
#define VIRTUAL_SCENE_TEST_H

#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>

#include "ObjectTrackerUtils.hpp"
#include "ModelLoader.h"

namespace cast
{

class VirtualSceneTest : public ManagedComponent
{
private:  
  
  std::vector<std::string> m_modelfile;
  
  /** Model loader for geometry data (ply-files) */
  Tracking::ModelLoader m_modelloader;
  
  /** callback function called whenever a new object appears ore an object
   * changes */
  void receiveVisualObject(const cdl::WorkingMemoryChange & _wmc);

protected:
  /** called by the framework to configure our component */
  virtual void configure(const std::map<std::string,std::string> & _config);
  
  /** called by the framework after configuration, before run loop */
  virtual void start();
  
  /** called by the framework to start compnent run loop */
  virtual void runComponent();
 
public:
  virtual ~VirtualSceneTest() {}
};

}

#endif

