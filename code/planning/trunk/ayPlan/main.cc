// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include"Arguments.hh"

#include"global.hh"

#include"Problem.hh"

#include "Planner.hh"
#include "Planner_templates.hh"

// #include "Action_templates.hh"
// #include "PredicatesAndPropositions_templates.hh"
// #include "StateEvaluation_templates.hh"

#include "CostPlanner.hh"

#include "State.hh"
//#include "State_templates.hh"

extern int yyparse();

extern Planning::Problem problem;

extern void initialiseLexer(istream*, ostream* );
extern void deleteLexer();

void satisficingMode()
{
    /*Make a planner for the problem we parsed.*/
    Planning::Planner<> planner(problem);
    
    VERBOSER(9, planner<<endl);

    VERBOSER(14, "Starting the planner."<<endl);

    
    planner();
    
    cout<<"\n\n WE FOUND PLAN :: "<<planner.getPlan()<<endl<<endl<<endl;
    
    VERBOSER(14, "The planner has finished executing.\n");
}


void optimalMode(uint initialLimit)
{
    Planning::CostPlanner planner(problem, initialLimit);
    VERBOSER(9, planner<<endl);

    VERBOSER(14, "Starting the planner."<<endl);
    
    planner();


//     for(Planning::CostPlanner::SetOfStatePointers::iterator state = planner.states.begin()
//             ; state != planner.states.end()
//             ; state++){
//         (*state)->compress();
//     }
    
//     for(Planning::CostPlanner::SetOfStatePointers::iterator state = planner.states.begin()
//             ; state != planner.states.end()
//             ; state++){
//         (*state)->decompress();
//     }
    
    
    cout<<"\n\n WE FOUND PLAN :: cost = "<<planner.getPlanCost()
	<<" :: "<<planner.getPlan()<<endl<<endl<<endl;
    
    VERBOSER(14, "The planner has finished executing.\n");
}



int main(int argc, char **argv)
{
    Arguments arguments(argc, argv);
    
    int seed = 2008;
    srandom(seed);
    srand(seed);
    
    
    string domainFileName = "domain.pddl";
    if(!(arguments.gotGuard("--domain"))){
        WARNING("missing argument ::"<<endl
                <<"--domain filename"<<endl
                <<"Using :: "<<domainFileName<<endl);
    } else {
        domainFileName = arguments.getString(); 
    }
    string problemFileName = "problem.pddl";
    if(!(arguments.gotGuard("--problem"))){
        WARNING("missing argument ::"<<endl
                <<"--problem filename"<<endl
                <<"Using :: "<<problemFileName<<endl);
    } else {
        problemFileName = arguments.getString(); 
    }

    
    ifstream domainFile;
    domainFile.open(domainFileName.c_str(), ifstream::in);

    if(!domainFile.is_open()){
	UNRECOVERABLE_ERROR("Couldn't open domain definition.\n");
    }
    
    initialiseLexer(&domainFile, &cout);
    if(yyparse()){
	UNRECOVERABLE_ERROR("Parsing of domain file :: "<<domainFileName<<" failed!\n");
    }
    deleteLexer();
    domainFile.close();

    
    
    ifstream problemFile;
    problemFile.open(problemFileName.c_str(), ifstream::in);

    if(!problemFile.is_open()){
	UNRECOVERABLE_ERROR("Couldn't open problem definition.\n");
    }
    
    initialiseLexer(&problemFile, &cout);
    if(yyparse()){
	UNRECOVERABLE_ERROR("Parsing of problem file :: "<<problemFileName<<" failed!\n");
    }
    
    deleteLexer();
    problemFile.close();


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
    
    //{char ch; cin>>ch;}
    
    cout<<problem<<endl;

    /*Make some space for the next phase of testing.*/
    for(int i = 0; i < 10; i++)cout<<endl;

    uint maxCost = 0;
    if(!(arguments.gotGuard("--max-cost"))){
        WARNING("missing argument ::"<<endl
                <<"--max-cost natural"<<endl
                <<"Running in satisficing mode."<<endl);
	
	if( arguments.isArgument("hash-tree") ){
	    VERBOSER(20, "We did no distribute hash-tree planning for the competition."<<endl);
	    
	    //hashSatisficingMode();
	} else if ( arguments.isArgument("hash-table") ) {    
	    VERBOSER(20, "Running in hash-table mode."<<endl);
	    
	    satisficingMode();
	} else {
	    WARNING("Hash structure unspecified, using \"hash-table\".");

	    satisficingMode();
	    
	}
	
    } else {
        maxCost = arguments.getInt();
	VERBOSER(20, "Running in optimal mode with limiting cost of :: "<<maxCost<<endl);
	optimalMode(maxCost);
	
    }
    
    
    FREE_MEMORY;
    
    return 0;
}    
