#include "classical_planner.hh"

// #undef DEBUG_LEVEL
// #define DEBUG_LEVEL 2


extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Classical_Planner();
  }
}

Classical_Planner::Classical_Planner(Designator&& name)
    :Implement(std::move(name))
{

    VERBOSER(401, "CONSTRUCTION FROM :: "<<name<<" "<<get_designators()<<" "<<CLASSICAL_PLANNER_DESIGNATION<<std::endl);
}

void Classical_Planner::implement__distinctPlanner(PCogX::distinctPlannerPtr& input){
    CAST__VERBOSER(401, " ** ** IMPLEMENTED -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

//     Implement::add_designator(input->additionalDesignationIsAnArgument);
    
    CAST__VERBOSER(401, " ** ** IMPLEMENTED -- COMPLETED -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");
}



void Classical_Planner::implement__postFileNameForDomainDescription(PCogX::postFileNameForDomainDescriptionPtr& input)
{
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;

    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        domain_file_names[*designator] = input->fileNameForDomainDescriptionIsAnArgument;
    }
}

void Classical_Planner::implement__postFileNameForProblemDescription(PCogX::postFileNameForProblemDescriptionPtr& input)
{
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;

    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        problem_file_names[*designator] = input->fileNameForProblemDescriptionIsAnArgument;
    }
}


extern Planning::Problem problem;
extern int yyparse();
extern void initialiseLexer(istream*, ostream* );
extern void deleteLexer();

void Classical_Planner::implement__actionParseProblemDescription(PCogX::actionParseProblemDescriptionPtr& input)
{
    
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;

    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        if(problem_file_names.find(*designator) == problem_file_names.end()){
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }
        
        assert(problem_file_names.find(*designator) != problem_file_names.end());
        auto problem_file_name = problem_file_names[*designator];
        
        std::ifstream problem_file;
        problem_file.open(problem_file_name.c_str(), ifstream::in);

        
        if(!problem_file.is_open()){
            CAST__WARNING("Couldn't open problem definition :: "<<problem_file_name<<std::endl);
            
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }

        /* The result of the parse is stored in \global{problem}...*/
        if(planning_problems.find(*designator) != planning_problems.end()){
            problem = planning_problems.find(*designator)->second;
        } else {
            problem = Planning::Problem();
        }
        
        
        initialiseLexer(&problem_file, &cout);
        if(yyparse()){
            
            CAST__WARNING("Couldn't parse problem definition :: "<<problem_file_name<<std::endl);
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }
        
        deleteLexer();
        problem_file.close();

        planning_problems[*designator] = problem;
    }
}

void Classical_Planner::implement__actionParseDomainDescription(PCogX::actionParseDomainDescriptionPtr& input)
{
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;

    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        if(problem_file_names.find(*designator) == problem_file_names.end()){
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }
        
        assert(problem_file_names.find(*designator) != problem_file_names.end());
        auto domain_file_name = domain_file_names[*designator];// = fileNameForProblemDescriptionIsAnArgument;
        
        std::ifstream domain_file;
        domain_file.open(domain_file_name.c_str(), ifstream::in);

        
        if(!domain_file.is_open()){
            CAST__WARNING("Couldn't open domain definition :: "<<domain_file_name<<std::endl);
            
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }

        /* The result of the parse is stored in \global{problem}...*/
        if(planning_problems.find(*designator) != planning_problems.end()){
            problem = planning_problems.find(*designator)->second;
        } else {
            problem = Planning::Problem();
        }
        
        initialiseLexer(&domain_file, &cout);
        if(yyparse()){
            
            CAST__WARNING("Couldn't parse domain definition :: "<<domain_file_name<<std::endl);
            input->parsingWasSuccessfulIsAReturn = false;
            return;
        }
        
        deleteLexer();
        domain_file.close();

        planning_problems[*designator] = problem;
    }
}


void Classical_Planner::implement__actionPreprocessProblemAndDomain(PCogX::actionPreprocessProblemAndDomainPtr& input)
{
    
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;

    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        /* Ensure there is a planning problem to preprocess.*/
        if(planning_problems.find(*designator) == planning_problems.end()){
            input->preprocessingWasSuccessfulIsAReturn = false;
            return;
        }

        auto& problem = planning_problems.find(*designator)->second;
        
        /*Types*/
        problem.domain.configureTypes();
        problem.domain.unwindTypes();
        /*Constants*/
        problem.domain.configureConstants();
        problem.domain.unwindConstants();
        /*Constants -- objects*/
        problem.configureObjects();
        problem.unwindObjects();
        problem.domain.initialiseMirrors();
        problem.initialiseMirrors();
        /*Decide what predicate symbols are fluent, and their type of fluency*/
        problem.domain.processActionsForSymbolTypes();
    
    }
}

void Classical_Planner::implement__actionPlan(PCogX::actionPlanPtr& input)
{
    
    CAST__VERBOSER(401, " ** ** IMPLEMENTED  -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");

    auto designators = input->optionalMemberDesignatorIsAnArgument;
    
    input->planIsAReturn = std::string("");
    
    for(auto designator = designators.begin()
            ; designator != designators.end()
            ; designator++){
        
        /* Ensure there is a planning problem to preprocess.*/
        if(planning_problems.find(*designator) == planning_problems.end()){
            input->planIsAReturn = std::string("");
            return;
        }

        auto& problem = planning_problems.find(*designator)->second;
        
        Planning::Planner<> planner(problem);
        
        
        CAST__VERBOSER(401, "Starting the planner."<<std::endl);
        
        planner();

        ostringstream oss;
        oss<<planner.getPlan();
        
        input->planIsAReturn += oss.str();

        plans[*designator] = oss.str();
    }
}

void Classical_Planner::implement__readPropositionIdentifiers(PCogX::readPropositionIdentifiersPtr& input){
    WARNING("UNIMPLEMENTED");
}
void Classical_Planner::implement__postSTRIPSAction(PCogX::postSTRIPSActionPtr& input){
    WARNING("UNIMPLEMENTED");
}
void Classical_Planner::implement__postActionDefinition(PCogX::postActionDefinitionPtr& input){
    WARNING("UNIMPLEMENTED");
}
void Classical_Planner::implement__postTypes(PCogX::postTypesPtr& input){
    WARNING("UNIMPLEMENTED");
}
void Classical_Planner::implement__postXXsubtypeofYY(PCogX::postXXsubtypeofYYPtr& input){
    WARNING("UNIMPLEMENTED");
}

void Classical_Planner::start()
{
    CAST__VERBOSER(401, " ** ** IMPLEMENTED -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");
    
    /* template for implementation of a procedure.*/
    //implement<PCogX::>(&Classical_Planner::implement__);
    
    implement<PCogX::distinctPlanner>(&Classical_Planner::implement__distinctPlanner);
    
//     implement<PCogX::actionPlan>(&Classical_Planner::implement__actionPlan);

//     implement<PCogX::actionPreprocessProblemAndDomain>(&Classical_Planner::implement__actionPreprocessProblemAndDomain);
//     implement<PCogX::actionParseDomainDescription>(&Classical_Planner::implement__actionParseDomainDescription);
//     implement<PCogX::actionParseProblemDescription>(&Classical_Planner::implement__actionParseProblemDescription);
//     implement<PCogX::postFileNameForDomainDescription>(&Classical_Planner::implement__postFileNameForDomainDescription);
//     implement<PCogX::postFileNameForProblemDescription>(&Classical_Planner::implement__postFileNameForProblemDescription);
    
    
//     implement<PCogX::readPropositionIdentifiers>(&Classical_Planner::implement__readPropositionIdentifiers);
    
//     implement<PCogX::postSTRIPSAction>(&Classical_Planner::implement__postSTRIPSAction);
    
//     implement<PCogX::postActionDefinition>(&Classical_Planner::implement__postActionDefinition);
    
//     implement<PCogX::postTypes>(&Classical_Planner::implement__postTypes);
    
//     implement<PCogX::postXXsubtypeofYY>(&Classical_Planner::implement__postXXsubtypeofYY);
    
    CAST__VERBOSER(401, " ** ** IMPLEMENTED -- COMPLETED ** ** ");
}

void Classical_Planner::runComponent()
{   
    CAST__VERBOSER(401, " ** ** IMPLEMENTED -- BEGIN -- "<<getSubarchitectureID()<<"   :: "<<get_designators()<<" :: ** ** ");   
    CAST__VERBOSER(1, " ** ** IMPLEMENTED -- COMPLETED ** ** ");    
}
