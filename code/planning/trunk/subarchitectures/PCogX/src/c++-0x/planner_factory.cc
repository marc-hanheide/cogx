
#include "planner_factory.hh"

#include<limits>

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Planner_Factory();
  }
}


Planner_Factory::Planner_Factory(Designator&& name)
    :Implement(std::move(name)),
     random_number_generator(
         std::bind(std::uniform_int<number_type>
                   (std::numeric_limits<number_type>::min(),
                    std::numeric_limits<number_type>::max()),
                   std::mt19937()))
{
}

void Planner_Factory::implement__obtainPlanner(PCogX::obtainPlannerPtr& input){
    input->identityOfCreatedPlannerIsAReturn = "";
}


void Planner_Factory::start() {
    
    implement<PCogX::obtainPlanner>(&Planner_Factory::implement__obtainPlanner);
    implement<PCogX::distinctPlanner>(&Planner_Factory::implement__distinctPlanner);
    
}

void Planner_Factory::runComponent()
{
}
