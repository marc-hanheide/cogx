// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef UNEXPANDEDSTACK_TEMPLATES_HH
#define UNEXPANDEDSTACK_TEMPLATES_HH


#include "State.hh"
#include "ActionCostState.hh"

#include "State_templates.hh"
#include "ActionCostState.hh"

#include "StateEvaluation_templates.hh"

namespace Planning
{
#if 0
#define RunAssertion(stateIndex, stacks)    { \
	for(typename CostToStates::const_iterator p = stacks.begin() \
		; p != stacks.end() \
		; p++){ \
	    for(typename  vector<StateType*>::const_iterator q = p->second.begin() \
		     ; q != p->second.end() \
		     ; q++){ \
		assert(stateIndex.find(*q) != stateIndex.end());	\
	    } \
	} \
    }
#else
#define RunAssertion(stateIndex, stacks) {};
#endif

    

    template<typename StateType, typename StateEvaluatorType>
    UnexpandedStack<StateType, StateEvaluatorType>::UnexpandedStack()
	:count(0)
    {
 

	RunAssertion(stateIndex, stacks);
	VERBOSER(23, "MAKE :: Made stack of unexpanded states :: "<<size()<<endl);
	RunAssertion(stateIndex, stacks);
    }


    template<typename StateType, typename StateEvaluatorType>
    bool UnexpandedStack<StateType, StateEvaluatorType>::contains(StateType* state) const
    {
	if(stateIndex.find(state) != stateIndex.end()) {
	    return true;
	} else {
	    return false;
	} 
    }


    template<typename StateType, typename StateEvaluatorType>
    void UnexpandedStack<StateType, StateEvaluatorType>::remove(StateType* state)
    {
	/*If the state that we remove is not unexpanded anyway.*/
	if(stateIndex.find(state) == stateIndex.end()) {
	    //WARNING("REMOVE :: Ignoring state as it is not a candidate for expansion. "<<*state<<endl);
	    return;
	    // 	exit(0);
	    // 	return;
	}
    
	RunAssertion(stateIndex, stacks);
	uint cost = stateEvaluator(state);
    
	if(stacks.find(cost) == stacks.end()) {
	    UNRECOVERABLE_ERROR("REMOVE :: Failed no stacks with cost "<<cost<<endl);
	    // 	exit(0);
	    // 	return;
	}
    
    
	uint index = stateIndex[state];
	vector<StateType*>& statesList = stacks[cost];

	assert(index < stacks[cost].size());
    
	VERBOSER(23, "REMOVE :: Removing state "<<*statesList[index]<<endl);

	assert(statesList.size() > 0);
    
	if(statesList.size() == 1){
	    stacks.erase(cost);//.clear();
	    stateIndex.erase(state);
	    costs.erase(cost);
	
	    assert(stacks.find(cost) == stacks.end());
	    assert(stateIndex.find(state) == stateIndex.end());
	    assert(costs.find(cost) == costs.end());
	
	    RunAssertion(stateIndex, stacks);
	} else {
	    /*Deleting a state at the end of a list.*/
	    if(index == ( statesList.size() - 1 ) ){//statesList[index] == statesList[statesList.size() - 1]){
		statesList.resize(statesList.size() - 1);
		stateIndex.erase(state);
	    
		assert(stateIndex.find(state) == stateIndex.end());
		RunAssertion(stateIndex, stacks);
	    } else {

		assert(state != statesList[statesList.size() - 1]);
		assert(stateIndex.find(statesList[statesList.size() - 1]) != stateIndex.end());
	    
		statesList[index] = statesList[statesList.size() - 1];
		statesList.resize(statesList.size() - 1);
	    
		assert(stateIndex.find(state) != stateIndex.end());
		stateIndex.erase(state);/*If this state occurs twice, then perhaps we are in trouble.*/
		assert(stateIndex.find(state) == stateIndex.end());
	    
		assert(stateIndex.find(statesList[index]) != stateIndex.end());
		stateIndex[statesList[index]] = index;
		assert(stateIndex.find(statesList[index]) != stateIndex.end());
	    
		RunAssertion(stateIndex, stacks);
	    }
	
	}
    
	assert(count > 0);
	VERBOSER(23, "REMOVE :: Stack of unexpanded states has size "<<(count - 1)<<endl);
	count--;

	if(count == 0){
	    assert(costs.size() == 0);
	}
    }


    template<typename StateType, typename StateEvaluatorType>
    void UnexpandedStack<StateType, StateEvaluatorType>::push_back(StateType* state)
    {
	RunAssertion(stateIndex, stacks);
	uint cost = stateEvaluator(state);

	/*Do not allow a state to be put on this list twice.*/
	assert(stateIndex.find(state) == stateIndex.end());
    
    
	VERBOSER(23, "PUSH_BACK :: Give state with cost :: "<<cost<<endl);
    
	if(stacks.find(cost) == stacks.end()){
	    VERBOSER(23, "Making new stack of states with cost :: "<<cost<<endl);
	    costs.insert(cost);
	    stacks[cost] = vector<StateType*>();
	}
    
	VERBOSER(23, "PUSH_BACK :: Pushing back state at index :: "<<stacks[cost].size()
		 <<" for cost :: "<<cost<<endl);
    
	stateIndex[state] = stacks[cost].size();
	stacks[cost].push_back(state);

	assert(stateIndex.find(state) != stateIndex.end());
    
	VERBOSER(23, "PUSH_BACK :: Stack of unexpanded states has size "<<(count + 1)<<endl);
	count++;
	RunAssertion(stateIndex, stacks);
    }

    template<typename StateType, typename StateEvaluatorType>
    bool UnexpandedStack<StateType, StateEvaluatorType>::empty() const
    {
	RunAssertion(stateIndex, stacks);
	assert(count != 0 || stateIndex.size() == 0);
    
	VERBOSER(23, "EMPTY :: Reporting stack emptiness as :: "<<(stateIndex.size() == 0)<<endl);
	RunAssertion(stateIndex, stacks);
	return (count == 0);//(stateIndex.size() == 0);
    }

    template<typename StateType, typename StateEvaluatorType>
    StateType* UnexpandedStack<StateType, StateEvaluatorType>::pop()
    {
	RunAssertion(stateIndex, stacks);
	if(empty()) return 0;

	VERBOSER(23, "POP :: count is "<<count<<endl);
	assert(count > 0);
	assert(costs.size() > 0);
    
	uint cost = *costs.rbegin();

	VERBOSER(23, "POP :: Choosing cost :: "<<cost<<endl);
    
	assert(stacks.find(cost) != stacks.end());

	assert(stacks[cost].size() > 0);
    
	uint index = random() % stacks[cost].size();

	assert(stacks.find(cost) != stacks.end());
	assert(index < stacks[cost].size());
    
	StateType* result = stacks[cost][index];
    
	assert(result);
	assert(stateEvaluator(result) == cost);
	if(stateIndex.find(result) == stateIndex.end()){
	    VERBOSER(23, "Unexpanded state :: "<<*result<<endl
		     <<"at index :: "<<index<<endl
		     <<"has no index..."<<endl);
	}
    
	assert(stateIndex.find(result) != stateIndex.end());

	VERBOSER(23, "POP :: Giving up state :: "<<*result<<endl);
    
	remove(result);

	VERBOSER(23, "POP :: Giving up state :: "<<*result<<endl);
    
	RunAssertion(stateIndex, stacks);
	return result;
    }


    template<typename StateType, typename StateEvaluatorType>
    uint UnexpandedStack<StateType, StateEvaluatorType>::size() const
    {
	return count;
    }


}

#endif
