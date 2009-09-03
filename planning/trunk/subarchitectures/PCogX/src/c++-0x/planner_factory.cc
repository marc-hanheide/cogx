
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

    Designator new_designation;

    decltype(random_number_generator()) candidate_designation;
    
    while(designators_already_in_use.end() !=
              designators_already_in_use
          .find(candidate_designation = random_number_generator())){
    }
        
      
    
    
    ostringstream oss;
    oss<<candidate_designation;

    auto designation_of_chosen_implementation
        = {std::move(std::string(CLASSICAL_PLANNER_DESIGNATION))};
    call<distinctPlanner>(candidate_designation.str(),
                          designation_of_chosen_implementation);
}


void Planner_Factory::start() {
    implement<PCogX::obtainPlanner>(&Planner_Factory::implement__obtainPlanner);   
}

void Planner_Factory::runComponent()
{
}
