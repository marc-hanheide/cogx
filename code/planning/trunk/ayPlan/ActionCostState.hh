// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.


#ifndef ACTIONCOSTSTATE_HH
#define ACTIONCOSTSTATE_HH

#include "State.hh"

#include "GroundActionWithCost.hh"

#include "UnexpandedStack.hh"
#include "StateEvaluation.hh"

namespace Planning
{
    /* A planning state that has a cost. For IPC-6 we suppose the cost
     * of a state is the best-estimate action cost of achieving the
     * state during a state-space expansion. Hashing and comparison is
     * from \parent{State<GroundActionWithCost>}.*/
    class ActionCostState : public State<GroundActionWithCost>
    {
    public:
	typedef GoalProximityEvaluator<ActionCostState> Evaluator;
	typedef UnexpandedStack<ActionCostState, Evaluator> Unexpanded;

	/*Makes a state with cost zero. \member{markedForExpansion} is
	 * initialised to \false.*/
	ActionCostState(uint = 0);

	/* Makes a state with cost
	 * \argument{ActionCostState.cost}. \member{markedForExpansion}
	 * is initialised to \false. Otherwise copy construction is
	 * based on \parent{State<GroundActionWithCost>} copy
	 * construction.*/
	ActionCostState(const ActionCostState&);
	
	/*(as for copy construction)*/
	ActionCostState& operator=(const ActionCostState& actionCostState);

	ActionCostState execute(const GroundActionWithCost&) const;
	ActionCostState execute_noDeleteEffects(const GroundActionWithCost& groundAction) const;
	
	/* Cost of state has changed. Make sure the states reachable
	 * from this state have costs which reflect this
	 * change. \argument{candidatesForExpansion} is a structure on
	 * states that are presently being considered for
	 * expansion. \argument{costLimit} is the best estimate so far
	 * of the cost to a goal state from the start state. If
	 * \call{propagateDelta} identifies a better cost for
	 * achieving the foal, then \argument{costLimit} is decreased
	 * accordingly.*/
	void propagateDelta(Unexpanded& candidatesForExpansion,
			    uint& costLimit);

        /*(see parent \class{State}) If replacing an entry in
         * \member{s_t_a_pair}, then the replacement only occurs in
         * this case when the cost is less.*/
        void actionYields(uint actionIndex, ActionCostState* successor);
        
	/* Call if \argument{parent} can yield \member{*this} with
	 * cost \argument{newCost}. The best estimated cost of
	 * achieving \member{*this} (i.e., \member{cost}) may have
	 * changed because: from \argument{parent} we can transition
	 * to \member{*this} with cost \argument{newCost}. If
	 * \member{cost} is change for the better via a call, then a
	 * call to \member{propagateDelta()} will be
	 * made. \argument{candidatesForExpansion} is a structure on
	 * states that are presently being considered for
	 * expansion. \argument{costLimit} is the best estimate so far
	 * of the cost to a goal state from the start state.*/
	void updateCostBecause(ActionCostState* parent,
			       uint newCost,
			       Unexpanded& candidatesForExpansion,
			       uint& costLimit);

	
	/*What is currently the best cost for achieving this state.*/
	uint cost;

	/*Has this state been marked for expansion.*/
	bool markedForExpansion;

	/* (initialised to NULL) Keeps track of the best
	 * \class{ActionCostState} found so far by calls to
	 * \member{propagateDelta()}. Such a state also must satisfy
	 * the goal condition .*/
	static ActionCostState* bestGoalTraversed;
    };
}

#endif


/* \begin{quote}

   pho (wide flour noodle soup) dac biet (special)

   \end{quote}

   Duc-Nghia Pham, 2008.

*/
