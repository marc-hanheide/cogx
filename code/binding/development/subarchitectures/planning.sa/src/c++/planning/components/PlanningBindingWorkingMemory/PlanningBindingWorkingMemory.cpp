#include <cast/core/CASTCompositeOntology.hpp>
#include <planning/ontology/PlanningOntologyFactory.hpp>
#include <binding/ontology/BindingOntologyFactory.hpp>
#include "PlanningBindingWorkingMemory.hpp"



extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new PlanningBindingWorkingMemory(_id);
  }
}

PlanningBindingWorkingMemory::PlanningBindingWorkingMemory(const string &_id)
  :  
     SubarchitectureWorkingMemory(_id)
{

  CASTCompositeOntology * ont = new CASTCompositeOntology();
   ont->addOntology(BindingOntologyFactory::getOntology());
   ont->addOntology(PlanningOntologyFactory::getOntology());
   setOntology(ont);

    // binding wm should broadcast 
   setSendXarchChangeNotifications(true);


}

PlanningBindingWorkingMemory::~PlanningBindingWorkingMemory() {}
