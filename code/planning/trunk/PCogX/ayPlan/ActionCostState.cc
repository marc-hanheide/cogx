// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "ActionCostState.hh"

using namespace Planning;

#include "State_templates.hh"

ActionCostState* ActionCostState::bestGoalTraversed = 0;

ActionCostState::ActionCostState(uint i)
    :State<GroundActionWithCost>(i),
     cost(0),
     markedForExpansion(false)
{
}

ActionCostState::ActionCostState(const ActionCostState& actionCostState)
    :State<GroundActionWithCost>(actionCostState),
     cost(actionCostState.cost),
     markedForExpansion(false)//actionCostState.markedForExpansion)
{
}

ActionCostState ActionCostState::execute(const GroundActionWithCost& groundActionWithCost) const
{
    ActionCostState result(*this);
    
    State<GroundActionWithCost>::execute(groundActionWithCost, result);
    
    result.cost += groundActionWithCost.cost;

    return result;
}

ActionCostState ActionCostState::execute_noDeleteEffects(const GroundActionWithCost& groundActionWithCost) const
{
    ActionCostState result(*this);
    
    State<GroundActionWithCost>::execute_noDeleteEffects(groundActionWithCost, result);
    
    return result;
}

void ActionCostState::actionYields(uint actionIndex, ActionCostState* successor)
{

   assert(actionIndex < plannerActions->size());

   MapStatePointerToUint::iterator s_t_a_pair =
successorToAction.find(successor);

   if(s_t_a_pair != successorToAction.end()){

       assert(s_t_a_pair->second < plannerActions->size());

       if((*plannerActions)[actionIndex].cost <
          (*plannerActions)[s_t_a_pair->second].cost){
           s_t_a_pair->second = actionIndex;
       }
   } else {
       successorToAction[successor] = actionIndex;
   }
}

void ActionCostState::updateCostBecause(ActionCostState* parent,
					uint newCost,
					Unexpanded& candidatesForExpansion,
					uint& costLimit)
{
    if(ActionCostState::bestGoalTraversed){
	VERBOSER(24, "Best traversed cost is :: "<<ActionCostState::bestGoalTraversed->cost<<" "
		 <<"Cost limit is  :: "<<costLimit<<" "
		 <<"New cost is :: "<<newCost<<endl);
	
	assert(ActionCostState::bestGoalTraversed->cost >= costLimit);
    }
    

    assert(newCost < cost);
    
    cost = newCost;
    
    if(cost < costLimit){// && !markedForExpansion){
	if(isGoalState){
	    if(ActionCostState::bestGoalTraversed){
		assert(ActionCostState::bestGoalTraversed == this
		       || ActionCostState::bestGoalTraversed->cost > cost);
		
		ActionCostState::bestGoalTraversed = this;
	    } else {
		ActionCostState::bestGoalTraversed = this;
	    }
	    costLimit = cost;
	} else if (!markedForExpansion) {
	    markedForExpansion = true;
	    candidatesForExpansion.push_back(this);
	}
    }
    
    setParent(parent);
    propagateDelta(candidatesForExpansion, costLimit);
}

void ActionCostState::propagateDelta(Unexpanded& candidatesForExpansion,
				     uint& costLimit)
{
    /*For every successor state.*/
    for(MapStatePointerToUint::iterator successorActionPair =  successorToAction.begin()
	    ; successorActionPair != successorToAction.end()
	    ; successorActionPair++){
	assert(successorActionPair->second < plannerActions->size());

	/*Don't bother propagating updated cost to the successor if it
	 * is equal to the predecessor.*/
	if(this == successorActionPair->first) {
	    VERBOSER(23, "Avoiding cost-propagation loop."<<endl);
	    continue;
	}

	/*What is the successor we are currently examining.*/
	ActionCostState* successor = dynamic_cast<ActionCostState*>(successorActionPair->first);

	assert(successor);
	
	/*Compute an updated cost of achieving the successor, it is
	 * was achieved from \local{*this}.*/
	uint newCost = cost + (*plannerActions)[successorActionPair->second].cost;

	/*What is the current action cost of achieving the
	 * \local{successor}.*/
	uint currentSuccessorCost = successor->cost;
	
	/*If we have a better upper-bound on the cost of achieving
	 * \local{successor} make sure that is reflected in that
	 * state.*/
	if(newCost < currentSuccessorCost){
	    successor->updateCostBecause(this, newCost, candidatesForExpansion, costLimit);
	}
    }
}

ActionCostState::ActionCostState& ActionCostState::operator=(const ActionCostState& actionCostState)
{
    State<GroundActionWithCost>::operator=(actionCostState);
    this->cost = actionCostState.cost;
    markedForExpansion = false;//actionCostState.markedForExpansion;
}


namespace ACTIONCOSTSTATE_Nonsense
{
    void foo()
    {
	ActionCostState acs(5);
	hash_value(acs);
	acs.hash_value();
	
	State<Planning::GroundActionWithCost> something;
	something.hash_value();
	hash_value(something);
    }
}

