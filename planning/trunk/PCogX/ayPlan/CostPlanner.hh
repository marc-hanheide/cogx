// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef COSTPLANNER_HH
#define COSTPLANNER_HH

#include "Planner.hh"
//#include "Planner_templates.hh"
#include "ActionCostState.hh"
#include "GroundActionWithCost.hh"

namespace Planning
{
    /* Uses the \parent{Planner} mechanisms to find a plan with
     * minimal action cost. This plan only considered states of type
     * \member{ActionCostState} (i.e., states with costs), and only
     * considers actions for execution of type
     * \class{GroundActionWithCost} (ground actions that when executed
     * have a cost).*/
    class CostPlanner : public Planner<ActionCostState, GroundActionWithCost>
    {
    public:
	/* First argument is the \argument{Problem} that the planner
	 * will be asked to solve. Second argument
	 * (\argument{costLimit}) is an initial bound on the cost of
	 * states-that-the-planner-should-consider.*/
	CostPlanner(Problem&, uint costLimit);
	
	/* What is the cost of the plan? If \parent{Planner}
	 * \member{goalState} is non-null, then we return of the cost
	 * of achieving this state. Calling this member only makes
	 * sense after a \member{goalState} has been identified.*/
	uint getPlanCost() const;
	
	/* Produce a cost-optimal plan using the parents planning
	 * procedure (see \class{Planner}).*/
	void operator()();
	
	/* What is the plan (see \parent{Planner::getPlan()})?
	 *
	 * NOTE: Only makes sense to call this once a call to
	 * \method{operator()} has terminated successfully -- i.e.,
	 * yielding a plan.
	 * 
	 * NOTE: Plan is given in reverse order.*/
	const vector<GroundActionWithCost>& getPlan()
	{return Planner<ActionCostState, GroundActionWithCost>::getPlan();};
	
    private:

	/* Remove all states from hash \parent{Planner.states} whose
	 * cost exceeds \member{costLimit}. States that are removed
	 * from the hash are also removed from
	 * \parent{Planner.unexpanded}. Successor information in the
	 * remaining states that refers to deleted states will be
	 * removed.
	 * 
	 * ASSUMES : \member{goalState} (if non-null) has
	 * \member{goalState->cost} equal to \member{costLimit}. If
	 * this assumption is violated the function immediately
	 * returns without any "cleaning up".
	 */
	void cleanUpStateHash();
	
	/* (see \class{Planner}). If a state was drawn whose cost
	 * (\argument{actionCostState.cost}) exceeds
	 * \member{costLimit}, then it is rejected (result is
	 * false).*/
	bool redraw(ActionCostState* actionCostState);

	/*(see \class{Planner}). As with \method{redraw()}, we only
	 * consider a state for expansion if its cost is less than
	 * \member{costLimit}.*/
	bool considerStateForExpansion(ActionCostState* successor);
	

	/* (see \class{Planner}). When we find that by executing an
	 * action at the state \argument{currentState}, we reach a
	 * state \argument{successor} that we have already seen before
	 * as \argument{oldSuccessor}, it could be that we have a
	 * lower estimate (\argument{successor->cost}) of the cost of
	 * achieving \argument{oldSuccessor}. In this case, where
	 * \argument{oldSuccessor} might not previously have been
	 * considered a candidate for expansion, with a tightening on
	 * its cost this status may have changed. */
	bool reconsiderStateForExpansion(ActionCostState* successor,
					 ActionCostState* currentState,
					 ActionCostState* oldSuccessor);

	/* No \instance{ActionCostState} whose
	 * \member{ActionCostState::cost} is greater than
	 * \member{costLimit} shall be considered by this planner
	 * unless it can be reached by executing a single action from
	 * a state whose cost is less than \member{costLimit} -- This
	 * limiting case is an unimportant detail.*/
	uint costLimit;

	/* If \method{operator()} terminates successfully, then this
	 * member points to a goal state that is least costly to
	 * achieve from the starting state.*/
	ActionCostState* optimalGoalState;
    };
}

#endif

/*

  \begin{quote}
  
  You're introduced to LSD an' less youse.. you've taken some other
  drug, like.. for instance like say marijuana or something ...  Well
  (you know) it's an all together new thing'an-aah to actually.. have a
  religious experience.. an' it can be even more important than..
  reading the Bible 6 times or.. be.. becoming a Pope-or...
  
  \end{quote}

  -- Child at Timothy Leary's (1920..1996) Millbrook estate. NOTE:
     This quote features part way through Track-1 of Hallucinogen's
     1999 album Twisted.
*/
