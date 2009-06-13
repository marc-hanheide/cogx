#include "ExampleBindingComponent.hpp"
#include "BindingOntologyFactory.hpp"

#include <boost/array.hpp>

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ExampleBindingComponent(_id);
  }
}



ExampleBindingComponent::ExampleBindingComponent(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id) {

  pOntology = BindingOntologyFactory::getOntology();

  boost::array<int,10> * pTest = new boost::array<int,10>();
}

ExampleBindingComponent::~ExampleBindingComponent() {

}





void ExampleBindingComponent::taskAdopted(const string &_taskID) {
    
}

void ExampleBindingComponent::taskRejected(const string &_taskID) {

 
}

void ExampleBindingComponent::runComponent() {
  
}
