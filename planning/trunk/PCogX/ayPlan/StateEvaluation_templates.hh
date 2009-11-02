// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef STATEEVALUATION_TEMPLATES_HH
#define STATEEVALUATION_TEMPLATES_HH

#include "State.hh"
#include "ActionCostState.hh"
#include "State_templates.hh"
#include "ActionCostState.hh"
#include "Action_templates.hh"

namespace Planning
{
    template<typename StateType>
    uint StateEvaluator<StateType>::operator()(const StateType* state)
    {
	return state->getTime();
    }

    template<typename StateType>
    uint GoalProximityEvaluator<StateType>::operator()(const StateType* state)
    {
	uint goalsReached = 0;

	assert(goalState.size() > 0);
    
	for(vector<uint>::const_iterator goalProposition = goalState.begin()
		; goalProposition != goalState.end()
		; goalProposition++){
	    VERBOSER(22, "Checking if proposition :: "<<*goalProposition<<" is SATISFIED."<<endl);
	    if(state->isTrue(*goalProposition)){
		VERBOSER(22, "Is SATISFIED."<<endl);
		goalsReached++;
	    } else {
		VERBOSER(22, "Not SATISFIED"<<endl);
	    }
	}
    
	return goalsReached;
    }
    
}


#endif
