#include "ExplorerBindingMotiveGenerator.hpp"
#include <motivation/components/BindingMotiveGenerator/BindingMotiveTemplates.hpp>


ExplorerBindingMotiveGenerator::ExplorerBindingMotiveGenerator(const std::string &_id) :
  WorkingMemoryAttachedComponent(_id),
  BindingMotiveGenerator(_id)
{}


void ExplorerBindingMotiveGenerator::start() {
  
  //always call start in the super class!
  BindingMotiveGenerator::start();

  this->registerMotiveTemplate<BindingData::BindingProxy,GoToDestinationTemplate>();
  this->registerMotiveTemplate<BindingData::BindingProxy,FindTemplate>();
}


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ExplorerBindingMotiveGenerator(_id);
  }
}

