
#include "planner_factory.hh"

#include<limits>
#include<sstream>

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
    CAST__VERBOSER(200, "");
    
    Designator new_designation;

    decltype(random_number_generator()) __candidate_designation;
    
    while(designators_already_in_use.end() !=
              designators_already_in_use
          .find(__candidate_designation = random_number_generator())){
    }
    
    std::ostringstream _candidate_designation;
    _candidate_designation<<__candidate_designation;
    const Designator& candidate_designation = _candidate_designation.str();
    
    input->identityOfCreatedPlannerIsAReturn = candidate_designation;
    
    
    auto designation_of_chosen_implementation
        = {std::move(std::string(CLASSICAL_PLANNER_DESIGNATION))};
    call<PCogX::distinctPlanner>(candidate_designation,
                                 designation_of_chosen_implementation);
}


void Planner_Factory::start() {
    implement<PCogX::obtainPlanner>(&Planner_Factory::implement__obtainPlanner);   
}

void Planner_Factory::runComponent()
{
}
