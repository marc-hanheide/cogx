#include "PlaymateBindingMotiveGenerator.hpp"
#include <motivation/components/BindingMotiveGenerator/BindingMotiveTemplates.hpp>


PlaymateBindingMotiveGenerator::PlaymateBindingMotiveGenerator(const std::string &_id) :
  WorkingMemoryAttachedComponent(_id),
  BindingMotiveGenerator(_id)
{}


void PlaymateBindingMotiveGenerator::start() {
  
  //always call start in the super class!
  BindingMotiveGenerator::start();

  //register motive for put actions
  this->registerMotiveTemplate<BindingData::BindingProxy,TargetLandmarkTemplate>();
  this->registerMotiveTemplate<BindingQueries::FeatureRequest,FeatureRequestTemplate>();
  
  //this->registerMotiveTemplate<BindingData::BindingProxy,PolarQuestionTemplate>();

  
  this->registerMotiveTemplate<BindingData::BindingProxy,PlayTheGameTemplate>();

  //what X is the Y
  this->registerMotiveTemplate<BindingData::BindingProxy,FactualWHQuestionTemplate>();

  //what do you see?
  this->registerMotiveTemplate<BindingData::BindingProxy,FactualPerceiveQuestionTemplate>();

}


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new PlaymateBindingMotiveGenerator(_id);
  }
}

