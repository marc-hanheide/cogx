// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "StateEvaluation.hh"

#include "StateEvaluation_templates.hh"

using namespace Planning;

namespace STATE_EVALUATION_Nonsense
{
    void foo()
    {
	State<>* state;
	ActionCostState* _state;
	{StateEvaluator<State<> > a;
	    a(state);}
	{StateEvaluator<ActionCostState > a;
	    a(_state);}
	{GoalProximityEvaluator<State<> > a;
	    a(state);}
	{GoalProximityEvaluator<ActionCostState > a;
	    a(_state);}
    }
}
