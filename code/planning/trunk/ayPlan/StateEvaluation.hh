// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef STATEEVALUATION_HH
#define STATEEVALUATION_HH

#include "global.hh"

/*(see \module(UnexpandedStack)). These evaluators determine a
 * preference order in which a planning might expand states.*/
namespace Planning
{   
    template<typename StateType>
    class StateEvaluator
    {
    public:
	/*Returns \class{StateType} \function{getTime()}.*/
	uint operator()(const StateType*);
    };

    template<typename StateType>
    class GoalProximityEvaluator
    {
    public:
	/* Counts the number of goal propositions that are satisfied in
	 * the argument state.*/
	uint operator()(const StateType*);

	/*Planning goal*/
	vector<uint> goalState;
    };
}

#endif
