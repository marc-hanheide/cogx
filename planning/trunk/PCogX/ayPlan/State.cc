// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.
#include "State.hh"

#include "State_templates.hh"

using namespace Planning;

namespace STATE__Nonsense
{
    
    void foo()
    {
	uint ii;
	State<Planning::GroundAction> _state(ii);
	State<> state(5);
	State<>* tmp = new State<>(ii);
	delete tmp;
	State<> _tmp(state);

	tmp->operator==(_tmp);
	tmp->operator<(_tmp);
	
	hash_value(state);
	state.hash_value();
	
	state = *tmp;

	uint i;
	
	vector<ELEM_TYPE> svec;

	
	state.flipOn(i);
	state.flipOff(i);
	state.flip(i);
	state.isTrue(i) ;
	state.isFalse(i);

	state.getTime();
	state.setTime(i);

	GroundAction* action;

	//state.expand();
	state.execute(*action);
	
	vector<GroundAction> vAct;
	State<>::MapStatePointerToUint maps;

	maps[tmp] = 5;
	
	state.ComputePossibleActions(vAct);
	state.UpdatePossibleActions();
	    
	state.actionYields(i, tmp);
	state.setParent(tmp);
	state.possible(*action);
	
	
	state.isGoal_positive(svec);
	state.isGoal_negative(svec);

	state.randomize();
	state.hash_value();
	state.getNumPropositions();
	state.mimick(svec);
	state.size();
	state.getData();

	hash_value(state);
	
	ostringstream oss;
	oss<<state;
    }
    
}



