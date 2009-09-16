#include "planner_client.hh"


// #undef DEBUG_LEVEL
// #define DEBUG_LEVEL 2

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Planner_Client();
  }
}

Planner_Client::Planner_Client(Designator&& name)
    :Implement(std::move(name))
{
}

void Planner_Client::start()
{ 
    CAST__VERBOSER(1,  " ** ** IMPLEMENTED -- BEGIN  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** "); 
    CAST__VERBOSER(1,  " ** ** IMPLEMENTED -- COMPLETED ** ** ");  

    
}

/* This is the closest we can get to C-like argc and argv.*/
void Planner_Client::configure(const std::map<std::string,std::string>& arguments)
{
    CAST__VERBOSER(1,  " ** ** IMPLEMENTED -- BEGIN  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");
    
    planning_subarchitecture_name = "";

    /* Sometimes people do something so utterly unintuitive it boggles
     * the mind. Like leaving the -- on a switch.. WTF!!*/
//     for(auto p = arguments.begin()
//             ; p != arguments.end()
//             ; p++){
//         std::cerr<<p->first<<" -> "<<p->second<<std::endl;
//     }


//     exit(0);
    
    auto _subarchitecture_name = arguments.find("--planning-subarchitecture-name");
    if(arguments.end() != _subarchitecture_name){
        planning_subarchitecture_name
        = std::move(_subarchitecture_name->second);
    } else {
        planning_subarchitecture_name = getSubarchitectureID();
        CAST__WARNING("There was no CAST-architecture-string specified for the planning subsystem. This is problematic, as we are required to know where the requests for plans should be sent. At the moment, we have recovered with :: "<<planning_subarchitecture_name<<". that should be the name of our subarchitecture."<<std::endl);
        
    }
                                                
    CAST__VERBOSER(1,  " ** ** IMPLEMENTED -- COMPLETED ** ** ");  
}


void Planner_Client::runComponent()
{
    CAST__VERBOSER(1,  " ** ** IMPLEMENTED -- BEGIN  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");
    
    /* Don't specify a component to send these planning requests to.*/
    Designators designators = {};

    /* Current implementation of std::forward cannot deal with
     * \type{bool}s, hence we cannot exploit move semantics properly
     * yet due to the type restrictions.. yes of course it is stupid
     * to forward a \type{bool}, but that is not the point...*/
    bool default_bool = true;
    
    CAST__VERBOSER(501, "Obtain a planner from a factory :: "<<planning_subarchitecture_name<<std::endl);
    
    CAST__VERBOSER(501, "Assuming the planner factory is at a subarchitecture named :: "
                   <<planning_subarchitecture_name<<std::endl);
    
    /* First up, get a planning system.*/
    auto _obtained_planner =
    call_to_subarchitecture
    <PCogX::obtainPlanner> /* SLICE name of call.*/
    (planning_subarchitecture_name /* CAST-subarchitecture name -- see
                                    * the .cast file that you are
                                    * using to execute -- "run" --
                                    * this
                                    * component. \argument{planning_subarchitecture_name}
                                    * should occur both as a string
                                    * appearing after
                                    * "SUBARCHITECTURE" and as that
                                    * string repeated occurring after
                                    * CAST-component-arguments-switch
                                    * "--planning-subarchitecture-name".*/
     , std::string("") /* RETURN -- */
     , std::vector<std::string>() /* ARGUMENT -- description of
                                   * requirements on the planner.*/
     , Designators() /* ARGUMENT -- \namespace{CAST_SCAT} component
                      * designator (see
                      * \module{CAST_SCAT/cast_scat.hh}).*/);

//     auto _obtained_planner =
//     call
//     <PCogX::obtainPlanner> /* SLICE name of call.*/
//     (std::string("") /* RETURN -- */
//      , std::vector<std::string>() /* ARGUMENT -- description of
//                                    * requirements on the planner.*/
//      , Designators() /* ARGUMENT -- \namespace{CAST_SCAT} component
//                       * designator (see
//                       * \module{CAST_SCAT/cast_scat.hh}).*/);

    CAST__VERBOSER(1, "Obtained a planner from a factory."<<std::endl);
    
//     std::string planner_designation = _obtained_planner->identityOfCreatedPlannerIsAReturn;

//     Designators planner_designators = {planner_designation};
    
//     CAST__VERBOSER(1, std::string("Let the planner thus obtained know where the") +
//                    std::string(" problem domain definition is.")<<std::endl);
    
//     call_to_subarchitecture
//     <PCogX::postFileNameForDomainDescription>
//     (planning_subarchitecture_name
//      , "./DOMAINS/blocksworld/propositional/domain.pddl"
//      , planner_designators);
    
//     CAST__VERBOSER(1, std::string("Let the planner thus obtained know where the") +
//                    std::string(" problem definition is.")<<std::endl);
    
//     call_to_subarchitecture
//     <PCogX::postFileNameForProblemDescription>
//     (planning_subarchitecture_name
//      , "./DOMAINS/blocksworld/propositional/problem.pddl"
//      , planner_designators);
    
//     auto _domain_parse_successful
//         = call_to_subarchitecture
//     <PCogX::actionParseDomainDescription>
//     (planning_subarchitecture_name
//      , default_bool
//      , planner_designators);
    
//     auto domain_parse_successful = _domain_parse_successful->parsingWasSuccessfulIsAReturn;
    
    
//     auto _problem_parse_successful
//         = call_to_subarchitecture
//     <PCogX::actionParseProblemDescription>
//     (planning_subarchitecture_name
//      , default_bool
//      , planner_designators);
    
//     auto problem_parse_successful = _problem_parse_successful->parsingWasSuccessfulIsAReturn;
    
//     call_to_subarchitecture
//     <PCogX::actionPreprocessProblemAndDomain>
//     (planning_subarchitecture_name
//      , default_bool
//      , planner_designators);
}
