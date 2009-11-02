// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "CostPlanner.hh"

#include "Planner_templates.hh"
#include "Action_templates.hh"

using namespace Planning;

CostPlanner::CostPlanner(Problem& problem, uint costLimit)
    :Planner<ActionCostState, GroundActionWithCost>(problem),
     costLimit(costLimit),
     optimalGoalState(0)
{
}

void CostPlanner::cleanUpStateHash()
{
    /* Does nothing until a \member{goalState} with cost
     * \member{costLimit} has been found.*/
    if(!goalState){
	return;
    } else {
	/* Expects the goal state to have the limiting cost.*/
	if(goalState->cost > costLimit){
	    return;
	}
    }

    VERBOSER(27, "Pruning for limit :: "<<costLimit<<endl);
    
    /* States that shall be removed from the hash \member{states}.*/
    /* SetOfStatePointers*/vector<ActionCostState*> toKeep(states.size());

    /* Deleted states must be kept in a const-time lookup structure
     * because we have to remove them from the state transitions thus
     * far constructed (see \class{State}.\member{}).*/
    typedef tr1::unordered_set<State<GroundActionWithCost>*,
	deref_hash<State<GroundActionWithCost> >,
	deref_equal_to<State<GroundActionWithCost> > > _SetOfStatePointers;
    _SetOfStatePointers deletedStates;
    
    uint index = 0;
    for(SetOfStatePointers::iterator _state =  states.begin()
	    ; _state != states.end()
	    ; _state++){
	ActionCostState* state = *_state;

	/* If the \local{state} is more costly to obtain than the best
	 * goal (\member{costLimit})*/
	if(state->cost > costLimit){
	    unexpanded.remove(state);

	    if(state == ActionCostState::bestGoalTraversed){
		assert(goalState);
		assert(goalState->cost <= ActionCostState::bestGoalTraversed->cost);
		
		ActionCostState::bestGoalTraversed = goalState;
	    }
	    
	    assert(state != ActionCostState::bestGoalTraversed);
	    assert(state != goalState);
	    assert(state != optimalGoalState);

	    /*Keep the address of the deleted state.*/
	    deletedStates.insert(state);
	    //delete state;//we delay deletion until the end of this function.

	    /*We may indeed empty the \member{unexpanded}, in which case planning must have finished.*/
	    //assert(unexpanded.size() > 0);
	} else {
	    toKeep[index++] = state;
	}
    }

    assert(toKeep.size() > 0);
    states = SetOfStatePointers();
    for(uint _index = 0; _index < index; _index++){
	/* Make sure states that are not removed do not report
	 * transitions to states that are marked for deletion.*/
	toKeep[_index]->removeStateReferences(deletedStates);

	if(deletedStates.size()){
	    toKeep[_index]->markedForExpansion = true;

	    if(!unexpanded.contains(toKeep[_index])){
		unexpanded.push_back(toKeep[_index]);
	    }
	}
	
	states.insert(toKeep[_index]);
    }

    for(_SetOfStatePointers::iterator state = deletedStates.begin()
	    ; state != deletedStates.end()
	    ; state++){
	delete *state;
    }
}

// THE FOLLOWING IMPLEMENTATION IS NOT ADMISSIBLE IN THE COST-OPTIMAL CASE
// void CostPlanner::cleanUpStateHash()
// {
//     /* Does nothing until a \member{goalState} with cost
//      * \member{costLimit} has been found.*/
//     if(!goalState){
// 	return;
//     } else {
// 	/* Expects the goal state to have the limiting cost.*/
// 	if(goalState->cost > costLimit){
// 	    return;
// 	}
//     }

//     VERBOSER(27, "Pruning for limit :: "<<costLimit<<endl);
    
//     /* States that shall be removed from the hash \member{states}.*/
//     /* SetOfStatePointers*/vector<ActionCostState*> toKeep(states.size());

//     /* Deleted states must be kept in a const-time lookup structure
//      * because we have to remove them from the state transitions thus
//      * far constructed (see \class{State}.\member{}).*/
//     typedef tr1::unordered_set<State<GroundActionWithCost>*,
// 	deref_hash<State<GroundActionWithCost> >,
// 	deref_equal_to<State<GroundActionWithCost> > > _SetOfStatePointers;
//     _SetOfStatePointers deletedStates;
    
//     uint index = 0;
//     for(SetOfStatePointers::iterator _state =  states.begin()
// 	    ; _state != states.end()
// 	    ; _state++){
// 	ActionCostState* state = *_state;

// 	/* If the \local{state} is more costly to obtain than the best
// 	 * goal (\member{costLimit})*/
// 	if(state->cost > costLimit){
// 	    unexpanded.remove(state);

// 	    if(state == ActionCostState::bestGoalTraversed){
// 		assert(goalState);
// 		assert(goalState->cost <= ActionCostState::bestGoalTraversed->cost);
		
// 		ActionCostState::bestGoalTraversed = goalState;
// 	    }
	    
// 	    assert(state != ActionCostState::bestGoalTraversed);
// 	    assert(state != goalState);
// 	    assert(state != optimalGoalState);

// 	    /*Keep the address of the deleted state.*/
// 	    deletedStates.insert(state);
// 	    //delete state;//we delay deletion until the end of this function.

// 	    /*We may indeed empty the \member{unexpanded}, in which case planning must have finished.*/
// 	    //assert(unexpanded.size() > 0);
// 	} else {
// 	    toKeep[index++] = state;
// 	}
//     }

//     assert(toKeep.size() > 0);
//     states = SetOfStatePointers();
//     for(uint _index = 0; _index < index; _index++){
// 	/*Make sure states that are not removed do not report
// 	 * transitions to states that are marked for deletion.*/
// 	toKeep[_index]->removeStateReferences(deletedStates);
	
// 	states.insert(toKeep[_index]);
//     }

//     for(_SetOfStatePointers::iterator state = deletedStates.begin()
// 	    ; state != deletedStates.end()
// 	    ; state++){
// 	delete *state;
//     }
// }

bool CostPlanner::considerStateForExpansion(ActionCostState* successor) 
{
    bool result = (successor->cost < costLimit);
    
    if(result){
	successor->markedForExpansion = true;
    }
    
    
    return result;
}

uint CostPlanner::getPlanCost() const
{
    return costLimit;
}

bool CostPlanner::reconsiderStateForExpansion(ActionCostState* successor,
					      ActionCostState* currentState,
					      ActionCostState* oldSuccessor) 
{
    VERBOSER(23, "Reconsidering state for expansion.\n");
    bool result = false;
    
    assert(oldSuccessor->parent != 0 || oldSuccessor == &startingState);

    uint oldCostLimit = costLimit;
    
    /*Has there been a useful change in the cost of achieving \local{oldSuccessor}.*/
    if(successor->cost < oldSuccessor->cost){

	/*The cost of achieving \local{oldSuccessor} has changed.*/
	oldSuccessor->cost = successor->cost;

	/*Update the path, as the \argument{currentState} is clearly
	 * the best parent (cost-wise) for \argument{oldSuccessor}.*/
	oldSuccessor->setParent(currentState);

	/* If there is a chance that this state could lead to a cheaper
	 * goal state than the best found so far.*/
	if(oldSuccessor->cost < costLimit){

            if(oldSuccessor->isGoalState){
                /*Don't bother reconsidering a goal state for expansion.*/
                /* Bugfix :: */
                costLimit = oldSuccessor->cost;
                ActionCostState::bestGoalTraversed = oldSuccessor;
                return false;
            }
            
	    /* The cost of achieving \local{oldSuccessor} has
	     * changed. Successor states should have their costs
	     * updated accordingly.*/
	    oldSuccessor->propagateDelta(unexpanded, costLimit);

	    /*If \argument{oldSuccessor} has not already been marked
	     * for expansion, then it should be marked for expansion
	     * because it could lead to a new goal state that is
	     * cheaper to achieved than the best we have so far.*/
	    if(!oldSuccessor->markedForExpansion){
		oldSuccessor->markedForExpansion = true;
		result = true;
	    }
	}
    }

//     /* If there has been a change to the cheapest way to get a goal,
//      * then we should clean up the hash of \member{states}.*/
//     if(oldCostLimit > costLimit){
// 	//cleanUpStateHash();
//     }//We could not clean up the states hash at this point because the \parent{Planner} may at that point consider a state that no longer exists.
    
    return result;    
}

bool CostPlanner::redraw(ActionCostState* actionCostState)
{
    VERBOSER(23, "Test for redraw.\n");

    /* If the argument state has been taken for expansion, then it must
     * have been from the unexpanded set, and thus marked for expansion.*/
    assert(actionCostState->markedForExpansion);
    
    bool result = (actionCostState->cost > costLimit);
    
    if(result){
	actionCostState->markedForExpansion = false;
    }
    
    if(result){
	
	VERBOSER(24, "CostPlanner wants to redraw :: "
		 <<costLimit<<" < "<<actionCostState->cost<<endl);
    }
    
    return result;
}

void CostPlanner::operator()()
{
    /* The \class{Planner} considers any state it discovers for
     * expansion. For this cost-optimal planner --
     * i.e. \class{CostPlanner} -- we only mark states for expansion
     * when they have a lower cost than the cost of the best plan
     * found so far. The cost of a state is the action cost of
     * reaching it from the \parent{Planner}::\member{startingState}
     * (see \parent{Planner}). This is updated incrementally as
     * planning progresses. NOTE: the only state that we can guarantee
     * we have computed the true cost of is the state that is returned
     * as a cost optimal goal state once the main loop in this
     * function terminates.*/
    startingState.markedForExpansion = true;
    
    
    do{
	/* Store the limit on cost-to-goal at the start of the
	 * loop. This way we can tell if the planner improved the
	 * bound or not.*/
	uint oldCostLimit = costLimit;
	
	/*Run the goal planner until a goal state is found or
	 * otherwise until there are no states that could be usefully
	 * explored.*/
	Planner<ActionCostState, GroundActionWithCost>::operator()();
	
	/*While updating the cost of non-goal and goal states during
	 * state expansion (in the main loop of the default planner), it
	 * sometimes happens that a goal state is traversed and its
	 * cost is updated to be less than "the cost of the best-cost
	 * goal-state during processing at that time". Such a goal
	 * state is stored at
	 * \static{\member{ActionCostState::bestGoalTraversed}}. If
	 * the planner was unable to identify a goal state, or
	 * otherwise the goal state that it did identify has a worse
	 * cost than the updated cost of some other goal state, then
	 * we should update the state pointed to by \member{goalState}
	 * to reflect this.*/
	if(goalState == 0 && ActionCostState::bestGoalTraversed){
	    VERBOSER(25, "Unable to find goal state, using a good state that we found during traversal.\n");
	    goalState = ActionCostState::bestGoalTraversed;
	} else if (ActionCostState::bestGoalTraversed) {
	    if(ActionCostState::bestGoalTraversed->cost < goalState->cost){
		VERBOSER(25, "Found new goal state, however the best goal we found was during traversal.\n");
		goalState = ActionCostState::bestGoalTraversed;
	    }
	}

	if(ActionCostState::bestGoalTraversed)
	    assert(goalState->cost <= ActionCostState::bestGoalTraversed->cost);
	
	/*If the planner was able to identify a goal state, there is
	 * no required processing.*/
	if(goalState != 0){
	    VERBOSER(20, "Got goal state :: "<<*goalState<<endl);
	    
	    VERBOSER(29, "Goal state with cost :: "<<goalState->cost<<endl);
	    DEBUG_GET_CHAR(27);
	    
	    /*If we have found a superior goal state, keep track of
	     * that fact in \member{optimalGoalState}.*/
	    if(optimalGoalState != 0){
		if(goalState->cost < optimalGoalState->cost){
		    optimalGoalState = goalState;
		}
	    } else {
		optimalGoalState = goalState;
	    }

	    /*If the best state found thus far has a better cost limit
	     * than \member{costLimit}, then update \member{costLimit}
	     * to reflect this.*/
	    if(optimalGoalState->cost < costLimit){
		costLimit = optimalGoalState->cost;
	    }
	}


	/*Clear away states we _may_ not need anymore. Essentially
	 * this removes from \member{states} and \member{unexpanded}
	 * those states with a cost greater than
	 * \member{costLimit}. NOTE: Only when we have improved
	 * \member{costLimit} during the last planning step do we
	 * bother with a cleanup.*/
	if(oldCostLimit > costLimit){
	    //cleanUpStateHash();
	}

	/* Make the \member{goalState} NULL for \parent{Planner}. This
	 * is not absolutely necessary because the default planner
	 * (i.e. parent) resets the \member{goalState} to NULL when it
	 * begins a plan search -- However, I am going to set this to
	 * null here anyway...*/
	goalState = 0;
    }while(unexpanded.size() != 0);


    /* ASSUME: No planning problems are posed that do not have a
     * solution.  If there is a goal state, then there is an
     * cost-optimal goal state.*/
    assert(optimalGoalState);
    
    /*If we have found a goal state, then that is great. The
     * \parent{Planner} \member{goalState} should reflect the cost
     * optimal state that was found.*/
    if(optimalGoalState){

	assert(optimalGoalState->isGoalState);
	
	VERBOSER(27, "Finished optimal-cost planning. Got a goal state with cost :: "
		 <<optimalGoalState->cost<<endl);
	
	goalState = optimalGoalState;
    }
}

