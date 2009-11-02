// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Planner.hh"

using namespace Planning;

namespace Planning
{
    namespace PlannerChooseState
    {
	bool twiceAttemptedExpansionWithEmptySet = false;
    }
}


#include "Planner_templates.hh"



namespace PLANNER_Nonsense
{
    
    void foo()
    {
	Problem* problem;
	State<>* state;
	
	Planner<>* _planner  = new Planner<>(*problem);
	delete _planner;
	
	Planner<> planner(*problem);
	planner.getPlan();
	planner.expand(*state);
	planner.chooseState();
	planner.isGoalState(*state);
	planner();
	planner.getPlan();
	planner.getProblem();
    }   
}

