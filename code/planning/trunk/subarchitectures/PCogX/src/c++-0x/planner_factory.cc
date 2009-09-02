
#include "planner_factory.hh"

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Planner_Factory();
  }
}

// template<>

void Planner_Factory::implement__obtainPlanner(PCogX::obtainPlannerPtr& input){
    input->identityOfCreatedPlannerIsAReturn = "";
}

void Planner_Factory::start() {
    implement<PCogX::obtainPlanner>(&Planner_Factory::implement__obtainPlanner);
}

void Planner_Factory::runComponent()
{
}
