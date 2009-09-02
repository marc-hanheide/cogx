// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "UnexpandedStack.hh"

#include "UnexpandedStack_templates.hh"

using namespace Planning;

namespace Planning
{
    
    typedef StateEvaluator<State<> > SimpleEvaluatorSimpleState;
    typedef StateEvaluator<ActionCostState > SimpleEvaluatorCostState;
    typedef GoalProximityEvaluator<State<> > GoalEvaluatorSimpleState;
    typedef GoalProximityEvaluator<ActionCostState > GoalEvaluatorCostState;
    
    typedef UnexpandedStack<ActionCostState, GoalEvaluatorCostState> Unexpanded;
}




namespace UNEXPANDED_STATE_Nonsense
{
    
    void foo()
    {
	SimpleEvaluatorSimpleState eva1;
	SimpleEvaluatorCostState eva2;
	GoalEvaluatorSimpleState eva3;
	GoalEvaluatorCostState eva4;

	State<>* state = 0;
	ActionCostState* _state = 0;
	
	{UnexpandedStack<State<>, SimpleEvaluatorSimpleState> a;
	    a.remove(state);
	    a.contains(state);
	    a.empty();
	    a.size();
	    a.push_back(state);
	    a.pop();}
	{UnexpandedStack<State<>, GoalEvaluatorSimpleState> a;
	    a.remove(state);
	    a.contains(state);
	    a.empty();
	    a.size();
	    a.push_back(state);
	    a.pop();}
	{UnexpandedStack<ActionCostState, SimpleEvaluatorCostState> a;
	    a.remove(_state);
	    a.contains(_state);
	    a.empty();
	    a.size();
	    a.push_back(_state);
	    a.pop();}
	{UnexpandedStack<ActionCostState, GoalEvaluatorCostState> a;
	    a.remove(_state);
	    a.contains(_state);
	    a.empty();
	    a.size();
	    a.push_back(_state);
	    a.pop();}
	
    }
}
