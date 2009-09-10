#include "planner_client.hh"


extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Planner_Client();
  }
}

Planner_Client::Planner_Client(Designator&& name)
    :Implement(name)
{
}

void Planner_Client::start()
{ 
    CAST__VERBOSER(200, " ** ** IMPLEMENTED ** ** ");  

    
}

void Planner_Client::runComponent()
{
    /* Don't specify an component to send these planning requests to.*/
    Designators designators = {};

    /* Current implementation of std::forward cannot deal with
     * \type{bool}s, hence we cannot exploit move semantics properly
     * yet due to the type restrictions.. yes of course it is stupid
     * to forward a \type{bool}, but that is not the point...*/
    bool default_bool = true;
    
    /* First up, get a planning system.*/
    auto _obtained_planner = call<PCogX::obtainPlanner>("", designators, designators);
    
    std::string planner_designation = _obtained_planner->identityOfCreatedPlannerIsAReturn;

    Designators planner_designators = {planner_designation};
    
    call<PCogX::postFileNameForDomainDescription>("DOMAINS/blocksworld/propositional/domain.pddl",
                                                  planner_designators);
    
    call<PCogX::postFileNameForProblemDescription>("DOMAINS/blocksworld/propositional/problem.pddl",
                                                   planner_designators);
    
    auto _domain_parse_successful
        = call<PCogX::actionParseDomainDescription>(default_bool, planner_designators);
    auto domain_parse_successful = _domain_parse_successful->parsingWasSuccessfulIsAReturn;
    
    
    auto _problem_parse_successful
        = call<PCogX::actionParseProblemDescription>(default_bool, planner_designators);
    auto problem_parse_successful = _problem_parse_successful->parsingWasSuccessfulIsAReturn;
    
    call<PCogX::actionPreprocessProblemAndDomain>(default_bool, planner_designators);
}
