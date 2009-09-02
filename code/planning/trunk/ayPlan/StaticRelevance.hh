// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#ifndef STATICRELEVANCE_HH
#define STATICRELEVANCE_HH

#include "State.hh"
#include "GroundAction.hh"

namespace Planning
{
    /*Examines the relevance of propositional and action symbols. Symbols
     * that do not occur in \member{allowedActionIndices} or
     * \member{allowedPropositionIndices} when a call to
     * \member{operator()} terminates are considered irrelevant.*/
    template<typename StateType = State<GroundAction>, typename GroundActionType = GroundAction>
    class StaticRelevance
    {
    public:
	StaticRelevance(const Planner<StateType, GroundActionType>& planner)
	    :planner(planner),
	     actions(planner.actions),
	     state(0)
	{};
    
	~StaticRelevance(){};

	void operator()()
	{
	    uint horizon = 0;
	    while(expand()){
		VERBOSER(27, "Testing reachability to horizon :: "<<++horizon<<endl);
		DEBUG_GET_CHAR(27);
	    }


	    for(uint i = 0; i < planner.fluentPropositions.size(); i++){
		if(state->isTrue(i)){
		    allowedPropositionIndices.insert(i);
		} else {
		    VERBOSER(27, "Fluent at index :: "<<i<<" --> "
			     <<planner.fluentPropositions[i]<<" is not reachable."<<endl);
		}
	    }
	}
    
	bool expand()
	{
	    StateType* toExpand = state;
	
	    if(!toExpand){
		toExpand = new StateType(planner.getStartingState());
	    }


	    uint actionIndex = 0;

	    bool reachabilityChanged = false;
	    for(typename vector<GroundActionType>::const_iterator action = actions.begin()
		    ; action != actions.end()
		    ; action++, actionIndex++){

		if(toExpand->possible(*action)){
		    /*action at index \local{actionIndex} can be executed.*/
		    if(!reachabilityChanged &&
		       allowedActionIndices.find(actionIndex) == allowedActionIndices.end()){
			/*We can execute an action we have never executed before.*/
			reachabilityChanged = true;
		    }
		
		    allowedActionIndices.insert(actionIndex);

		    state = new StateType(toExpand->execute_noDeleteEffects(*action));
		    delete toExpand;
		    toExpand = state;
		} else {
		    VERBOSER(26, "At a certain horizon, action :: "<<*toExpand<<" is not available."<<endl);
		}
	    }

	    return reachabilityChanged;
	}

    
	/*Indices to _possibly_ reachable actions.*/
	set<uint> allowedActionIndices;

	/*Indices to _possibly_ reachable propositions.*/
	set<uint> allowedPropositionIndices;
    public:
	const Planner<StateType, GroundActionType>& planner;
    
	StateType* state;
    
	const vector<GroundActionType>& actions;

    };
}

#endif
