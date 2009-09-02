// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef PLANNER_TEMPLATES_HH
#define PLANNER_TEMPLATES_HH

#include "Curses.hh"

#include "State_templates.hh"

#include "Action_templates.hh"
#include "UnexpandedStack_templates.hh"

#include "StaticRelevance.hh"

namespace Planning
{

#ifndef NATHAN
    using namespace tr1;
#endif


}

namespace Planning
{
    template<typename StateType, typename GroundActionType>
    const vector<GroundActionType>& Planner<StateType, GroundActionType>::getPlan()
    {
	VERBOSER(27, "Calling generic \\function{getPlan()} from \\class{Planner}."<<endl);
	
	/*Extract a plan if this hasn't already been done.*/
	if(goalState != 0 && plan.size() == 0){
	    StateType* currentState = goalState;

	    VERBOSER(27, "Trying to extract a plan.\n");
	
	    while(currentState->parent != 0){

		assert(currentState != &startingState || currentState->parent != 0);
	    
		assert( currentState->parent->successorToAction.find(currentState)
			!=  currentState->parent->successorToAction.end());
	    
		uint actionIndex = currentState->parent->successorToAction[currentState];

		assert(actionIndex < actions.size());

		VERBOSER(27, "Pushing back action :: "<<actions[actionIndex]<<endl);
	    
		plan.push_back(actions[actionIndex]);

		assert(dynamic_cast<StateType*>(currentState->parent));
		currentState = dynamic_cast<StateType*>(currentState->parent);//currentState->parent;//->getParent();
	    }

	    vector<GroundActionType> tmp = plan;//(plan.size());

	    reverse_copy(plan.begin(), plan.end(), tmp.begin());
	    plan = tmp;
	    
	    
	    assert(currentState == &startingState);
	} else {
	    if(goalState != 0){
		VERBOSER(27, "Already got a non-nonempty plan. hence not recomputing.");
	    } else {
		VERBOSER(27, "Asked to extract a plan from a NULL goal state."<<endl);
	    }
	    
	}

	return plan;
    }

    template<typename StateType, typename GroundActionType>
    void Planner<StateType, GroundActionType>::operator()()
    {
	goalState = 0;// = false;
	while(!goalState){
	    StateType* chosenState = chooseState();

	    if(!chosenState){
		VERBOSER(27, "Could not choose a state. Planning failed."<<endl);
		return;
	    }

	    VERBOSER(22, "Chosen state :: "<<*chosenState<<endl);

	    VERBOSER(27, "Number of states :: "<<states.size()<<endl);
	    
	    StateType& state = *chosenState;//*unexpanded[chosenStateIndex];
	    
#if  DEBUG_LEVEL == 25
	    cout<<states.size()<<" -- ";
#endif
	    
	    if(!state.actions.size()){
		continue;
	    }
	    
	    assert(state.actions.size() > 0);
	
	    goalState = expand(state);

	    
	    
// 	    /*Make sure we never expand the same state twice.*/
// 	    unexpanded[chosenStateIndex] = unexpanded[unexpanded.size() - 1];//unexpanded.last();
// 	    unexpanded.resize(unexpanded.size() - 1);
	}
    
	VERBOSER(27, "GOAL ACHIEVED!"<<endl);
    }

    template<typename StateType, typename GroundActionType>
    StateType*  Planner<StateType, GroundActionType>::expand(StateType& state)
    {
	if(state.actions.size() == 0){
	    WARNING("Got a state :: "<<state<<endl
		    <<"From which we can not execute any actions."<<endl);
	}

	VERBOSER(13, "Expanding :: "<<state<<endl<<endl);
    
	for(SetOfUnsignedInts::const_iterator action = state.actions.begin()
		; action != state.actions.end()
		; action++){

	    assert(*action < actions.size());

	    if(!state.possible(actions[*action])){
		UNRECOVERABLE_ERROR("Illegal action :: "
				    <<actions[*action]<<endl<<" at index :: "<<*action<<endl);
	    }
	
	    assert(state.possible(actions[*action]));

	    assert(states.find(&state) != states.end());
	
	    /*Generate a new state \local{successor}, the successor of
	     *\argument{State} under action at index \local{action}.*/
	    StateType* successor = new StateType(state.execute(actions[*action]));

	    if(successor->actions.size() == 0){
		WARNING("[from \\class{Planner}] Generated state :: "<<*successor<<endl
			<<"** from which no actions are executable."<<endl);
	    }
	
	    //assert(successor->actions.size() > 0);    
	    if(!successor->actions.size()){
		WARNING("Generated state with no successors!"<<endl);
	    }
	    
	    VERBOSER(17, "Executing :: "<<actions[*action]<<endl<<endl
		     <<"From :: "<<state<<endl<<endl
		     <<"Yields :: "<<*successor<<endl<<endl);
	
	    typename SetOfStatePointers::iterator stateIterator = states.find(successor);

	    VERBOSER(13, "Executed action :: "<<actions[*action]<<endl<<endl);
	
	    VERBOSER(13, "Testing to see if :: "<<*successor<<" is a new state."<<endl<<endl);

	    DEBUG_GET_CHAR(13);
	
	    /*(NEW STATE): If we have a new state.*/
	    if(stateIterator == states.end()){
		VERBOSER(13, "** IS _NEW_!  #"<<states.size()<<endl);

		/*Hash the new state.*/
		states.insert(successor);

		
		/*Keep track of what actions led to a state.*/
		/*successor->*/
		state.actionYields(*action, successor);

	    
		/*The state must always keep track of its parent so that
		  we can extract the plan (sequence of states and actions)
		  later.*/
		successor->setParent(&state);

	    
		assert(successor->parent->successorToAction.find(successor)
		       !=  successor->parent->successorToAction.end());

		/*If we have a goal state then planning is complete.*/
		if(isGoalState(*successor)){
		    assert(successor->isGoal_positive(goalTrue));
		    
		    successor->isGoalState = true;
		    return successor;
		} else {
		    assert(!successor->isGoal_positive(goalTrue));
		}
		
		/*The expansion of the \local{successor} under actions has
		 *not yet been linked to its successors via actions.*/
		if(considerStateForExpansion(successor)){
		    unexpanded.push_back(successor);
		}
	    
	    
	    } /*(REVISIT STATE): If we have seen \local{successor} before.*/ else {

		VERBOSER(13, "** NOT NEW!"<<endl);

		if(reconsiderStateForExpansion(successor, &state, *stateIterator)){
		    unexpanded.push_back(*stateIterator);
		}
		    
		state.actionYields(*action, *stateIterator);

		if((*stateIterator) != &startingState){
		    assert((*stateIterator)->parent->successorToAction.find((*stateIterator))
			   !=  (*stateIterator)->parent->successorToAction.end());
		}
		
		delete successor;
	    }
	}

	return 0;
    }

    template<typename StateType, typename GroundActionType>
    bool Planner<StateType, GroundActionType>::considerStateForExpansion(StateType* successor) 
    {
	return true;
    }

    template<typename StateType, typename GroundActionType>
    bool  Planner<StateType, GroundActionType>::reconsiderStateForExpansion(StateType* successor,
									    StateType* currentState,
									    StateType* oldSuccessor) 
    {
	assert(currentState);
	
	assert(oldSuccessor->parent != 0 || oldSuccessor == &startingState);
	
	if(successor->getTime() < oldSuccessor->getTime()){
	    
	    /*Update the path.*/
	    oldSuccessor->setParent(currentState);
	    
	    /* FIX :: The parents time stamp just changed, hence we
	     * need to change its timestamp, and ensure that all
	     * its successors have their timestamps updated. --
	     * THIS IS GOING TO GREATLY INCREASE THE COAST OF
	     * PLANNING...*/
	    
	    //currentState->actionYields(actionIndex, old);//successor);
	    
// 	    assert(old->parent->successorToAction.find(old)
// 		   !=  old->parent->successorToAction.end());
	    
	}
	
	//assert(oldSuccessor->parent != 0);
	
	return false;    
    }

    namespace PlannerChooseState
    {
	/*The first time the planner looks for a goal, the expansion
	 * stack _may_ be empty. In this case we recover by expanding
	 * the starting state. The second time the stack becomes empty
	 * such a recovery is nonsense. This Boolean recalls if we
	 * have recovered with a starting state yet.*/
	extern bool twiceAttemptedExpansionWithEmptySet;
    }

    template<typename StateType, typename GroundActionType>
    StateType*  Planner<StateType, GroundActionType>::chooseState()
    {
	if(unexpanded.size() == 0){

	    assert(startingState.actions.size() > 0);

	    if(PlannerChooseState::twiceAttemptedExpansionWithEmptySet){
		WARNING("This is the second time we have tried to expand"<<endl
			<<"states from an empty set. We cannot recover by"<<endl
			<<"opting for the start-state more than once. Hence,"<<endl
			<<"she is pumping mud Clancy!..."<<endl);
	    } else {
		unexpanded.push_back(&startingState);
		
		WARNING("Planner was not explicitly given an unexpanded start state."<<endl);
		WARNING("Recovering with :: "<<startingState<<endl);
		
		/*As we push things on to the \local{unexpanded} stack, we
		 *should always make sure that they are in the states hash.*/
		states.insert(&startingState);

		assert(startingState.actions.size() > 0);
		
	    }
	    
	    PlannerChooseState::twiceAttemptedExpansionWithEmptySet = true;
	}
    
	if(unexpanded.size() == 0) return 0;
	assert(unexpanded.size() > 0);

	StateType* candidate = unexpanded.pop();
	
    
	if(!candidate) return 0;

	
	VERBOSER(22, "Chosen candidate state :: "<<*candidate<<endl);
	
	if(redraw(candidate)) {
	    return chooseState();
	} else {
	    return candidate;
	}
    }


    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_setActionPossibilities()
    {

	if(actions.size() == 0){
	    UNRECOVERABLE_ERROR("There are no actions, hence there is no state space."<<endl
				<<"Please ensure that the \\class{Planner} has elements in \\local{actions}.");
	}
    
	/*For each action.*/
	for(uint i = 0; i < actions.size(); i++){
	    const GroundActionType& action = actions[i];
	
	    /*For each (if pruning has occurred then fluent) proposition
	     * mentioned in the actions precondition.*/
	    for(vector<uint>::const_iterator prop = action.prec.begin()
		    ; prop != action.prec.end()
		    ; prop++){
	    
		/*Find the set of actions that are already associated with
		 * \local{prop}*/
		MapIntToInts::iterator intToSetUint
		    = StateType::propositionsAllowsActions.find(*prop);

		/*If no such set exists, then make one.*/
		if(intToSetUint == StateType::propositionsAllowsActions.end()){
		    SetOfUnsignedInts tmp;
		    tmp.insert(i);
		    //State::propositionsAllowsActions[*prop] = SetOfUnsignedInts();
		    //State::propositionsAllowsActions[*prop].insert(i);

		    StateType::propositionsAllowsActions[*prop] = tmp;
		}
		/* Otherwise add the index $i$ to \local{action} to the set
		 * of actions that require \local{prop} as a
		 * precondition.*/
		else {
		    intToSetUint->second.insert(i);
		}
	    }
	}
    }

    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::addActionGroundings(set<pair<uint, uint> >::const_iterator predicateIndex,
								    Constants& argument,
								    VariableToConstant& variableToConstant,
								    const Variables& variablesInOrder,
								    const VariableToUnsignedInt& variableIndex,
								    vector<Constants >& arguments,
								    const Action<>& action,
								    const SignedPredicates& actionPrecondition,
								    const SignedPredicates& actionEffects,
								    const set<pair<uint, uint> >& sortedPredicateIndices,
								    const vector<bool>& choiceVariables,
								    const vector<SetOfConstants>& argumentsToCheck)
    {
	if(predicateIndex == sortedPredicateIndices.end()){
	
	    VERBOSER(15, "Completed one grounding of action :: "<<action.getName()<<endl);
	    VERBOSER(15, "We have an assignment with :: "<<variableToConstant<<endl);
	
	    for(VariableToConstant::const_iterator vToC = variableToConstant.begin()
		    ; vToC != variableToConstant.end()
		    ; vToC++){
		assert(variableIndex.find(vToC->first) != variableIndex.end());
		uint index = variableIndex.find(vToC->first)->second;
	    
		arguments[index] = Constants(1);
		arguments[index][0] = vToC->second;

	    

		VERBOSER(19, "Applied assignment :: "<<vToC->first<<" ("<<index<<") "
			 <<" = "<<vToC->second<<" to action arguments.\n");
	    }

#ifndef NDEBUG
	    for(vector<Constants >::const_iterator arg = arguments.begin()
		    ; arg != arguments.end()
		    ; arg++){
		assert(arg->size() > 0);
	    }
#endif

	    VERBOSER(17, "Abut to make an instance of action :: "<<action.getName()<<endl
		     <<" ** with the following ground arguments : "<<endl);

// #ifndef NDEBUG
// 	    for(uint i = 0; i < arguments.size(); i++){
// 		for(uint j = 0; j < arguments[i].size(); j++){
// 		    cerr<<arguments[i][j]<<", ";
// 		}

// 		cerr<<endl;
// 	    }
// 	    cerr<<endl;
// #endif
	
	
	    addActionGroundings(0,
				argument,
				variableToConstant,
				variablesInOrder,
				arguments,
				action.getName(),
				actionPrecondition,
				actionEffects,
				action);

	    return;
	}
    
	assert(predicateIndex->second < actionPrecondition.size());
	const SignedPredicate& signedPredicate = *actionPrecondition[predicateIndex->second];

	PredicateName predicateName = signedPredicate.getName();

	/*Special case that supports negative preconditions provided
	 * they only relate to the equality symbol (Which when I
	 * originally wrote this planner I had no intention of
	 * supporting).*/
	if("=" == predicateName && signedPredicate.isNegative()){
	    VERBOSER(22, "Appealing to the interpretation of inequality \"!=\"."<<endl);
	    predicateName = PredicateName("!=");
	} else if ("=" == predicateName) {
	    VERBOSER(22, "Appealing to the interpretation of equality \"=\"."<<endl);
	}
	
	/*Make sure we have a model of the \local{signedPredicate} that is
	 * part of the \argument{action} precondition.*/
	assert(propositionIndices.find(predicateName) != propositionIndices.end());
	    
    
	const pair<uint, uint> lowerUpper = propositionIndices[predicateName];
    
	VariableToConstant backup_variableToConstant = variableToConstant;


	VERBOSER(18, "Making partial assignments for action :: "<<action.getName()<<endl);
	while(action.makePartialAssignment(variableToConstant,
					   signedPredicate,
					   predicateIndex->second,
					   propositions,
					   lowerUpper.first,
					   lowerUpper.second,
					   variableIndex,
					   argumentsToCheck)){
	    predicateIndex++;
	
	    VERBOSER(15, "Moving on to next precondition\n");

	    VERBOSER(18, "** PUMPING ** \n");
	
	    VERBOSER(18, "Assignment thus far is :: "<<variableToConstant<<endl);
	
	    addActionGroundings(predicateIndex,
				argument,
				variableToConstant,
				variablesInOrder,
				variableIndex,
				arguments,
				action,
				actionPrecondition,
				actionEffects,
				sortedPredicateIndices,
				choiceVariables,
				argumentsToCheck);
	    VERBOSER(18, "** RETRACTING ** \n");
	    predicateIndex--;
	    variableToConstant = backup_variableToConstant;
	}
    }


    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::addActionGroundings(uint index,
								    Constants& argument,
								    VariableToConstant& variableToConstant,
								    const Variables& variablesInOrder,
								    const vector<Constants >& arguments,
								    const string& actionName,
								    const SignedPredicates& actionPrecondition,
								    const SignedPredicates& actionEffects,
								    const Action<>& action)
    {

    
	if(index >= arguments.size()){

	    
	    /*If the cost of executing this action is determined by the
	     * evaluation of a predicate according to
	     * \argument{variableToConstant}*/
	    if(action.costEvaluator != 0){
		
		Problem::MapPropositionToUint& startingStateFunctionEvaluation = problem.startingStateFunctionEvaluation;
		
		Proposition<> proposition
		    = action.costEvaluator->ground_NoSign(variableToConstant);
		
		if(startingStateFunctionEvaluation.find(proposition) == startingStateFunctionEvaluation.end()){
                    
                    const_cast<Action<>&>(action).cost =
                        0;
                    
                    
		    WARNING("Action "<<*this<<endl
                            <<"Requires we have an integer associated with :: "<<proposition<<endl
                            <<"In the start state. This does not appear to be the case."<<endl);
		} else {
                    /*Alter the cost of the axiom schema so that the
                     * ground action we create is given the correct
                     * cost.*/
                    const_cast<Action<>&>(action).cost = startingStateFunctionEvaluation[proposition];
                }
                
	    }
	    
	    GroundActionType groundAction(actionName,
					  argument,
					  actionPrecondition,
					  actionEffects,
					  variableToConstant,
					  action);

	    actionIndex[groundAction] = actions.size();
	    actions.push_back(groundAction);

	    VERBOSER(9, "Added ground action :: "<< actions[actions.size() - 1]<<endl);

	    CURSES_OSS_WRITE(TOP_RIGHT, "Action: "<<actions.size());
	    
	    // 	Proposition<> tmp(predicateName, argument);
	    // 	propositionIndex[tmp] = propositions.size();
	    // 	propositions.push_back(tmp);
	} else {
	    for(uint i = 0; i< arguments[index].size(); i++){
		argument[index] = arguments[index][i];
		variableToConstant[variablesInOrder[index]] = argument[index];
	    
		addActionGroundings(index + 1,
				    argument,
				    variableToConstant,
				    variablesInOrder,
				    arguments,
				    actionName,
				    actionPrecondition,
				    actionEffects,
				    action);
	    }
	}
    }


    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_actions()
    {
	for(vector<Action<> >::const_iterator _actionSchema = problem.domain.actions.begin()
		; _actionSchema != problem.domain.actions.end()
		; _actionSchema++){
	    const Action<>& actionSchema = *_actionSchema;

	
	    const string& name = actionSchema.getName();
	    const Arguments& _arguments = actionSchema.getArguments();
	    const SignedPredicates& preconditions = actionSchema.getPreconditions();
	    const SignedPredicates& effects = actionSchema.getEffects();
	
	    /*Set of admissible constants for the 1, 2, .., nth
	     *argument. This does not store the values in the case that
	     *the argument participates in a constant predicate symbol in
	     *the action precondition.*/
	    vector<Constants > arguments(getArity(_arguments));

	    /*When the argument participates in a constant predicate
	     * symbol from the action precondition, we store the
	     * admissible assignments to it in the following searchable
	     * structure.*/
	    vector<SetOfConstants> argumentsToCheck(getArity(_arguments));
	
	    const Variables& variablesInOrder = actionSchema.getVariablesInOrder();

	    const VariableToUnsignedInt& variableIndex = actionSchema.getVariableIndex();

	    //VERBOSER(19, "The variable index is \n"<<variableIndex<<endl);
	
	    const vector<bool>& choiceVariables = actionSchema.getVariableChoices(problem.domain);


	    /*If the action takes arguments, then we should have
	     * something to say about those.*/
	    assert(_arguments.size() == 0 || variablesInOrder.size() > 0);
	    assert(_arguments.size() == 0 ||variableIndex.size() > 0);
	    assert(_arguments.size() == 0 ||choiceVariables.size() > 0);
	
	    /*Index...*/
	    uint argCount = arguments.size() - 1;
	
	    /*Remember.. the arguments were parsed backwards.*/
	    for(Arguments::const_iterator argument = _arguments.begin()
		    ; argument != _arguments.end()
		    ; argument++){

		VERBOSER(7, "Generating action :: "<<name<<" with Argument :: "<<*argument<<endl);
	    

		/*We are dealing with a set of arguments which are in the
		 *union of the types \local{argument->first}.*/
		const Types& types = argument->first;

		/*Set of variable symbols that have are objects in the
		 * union of \local{types}.*/
		const Variables& variables  = argument->second;

		/* The order in which we parsed the variables is backwards
		 * remember (CHECKED).*/
	    
		for(int varIndex = 0; varIndex < variables.size(); varIndex ++){
		
		    if(choiceVariables[argCount]){
		
			VERBOSER(7, "Unwinding var :: "<<variables[varIndex]<<" of type :: "<<*types.begin()<<endl);
			problem.getObjects(types, arguments[argCount]);
		    
			VERBOSER(7, "Got objects :: "<<arguments[argCount]<<endl);
		    
			if(arguments[argCount].size() == 0){
			    WARNING("There are no objects of type :: "<<*types.begin()<<endl);
			} else {
			    VERBOSER(3, "There are "<<arguments[argCount].size()
				     <<" object/s of type: : "<<*types.begin()<<endl);
			}
		    }
		    /* Keep track of the objects that are admissible
		     * arguments so that when they are extracted from the
		     * ground preconditions we can verify that they are
		     * admissible arguments.*/
		    else {
			Constants tmp;
			problem.getObjects(types, tmp);
		    
			argumentsToCheck[argCount].insert(tmp.begin(), tmp.end());
		    }
		
		
		    /* Listed the objects associated with one more argument.*/
		    argCount--;
		    assert(argCount>=0);
		}
	    }


	    /* LHS is size of the propositional model associated with RHS
	     * index of predicate as it occurs in the action
	     * preconditions. Ordered predicate indices (<size of model,
	     * index>) (in \local{preconditions}).*/
	    set<pair<uint, uint> > sortedPredicateIndices;
	
	    /*Sort the constant predicates that appear in the action
	     *precondition in terms of the number of models of the
	     *predicate occurring in the model.*/
	    
	    assert(propositions.size() > 0);/*Assert that we actually have some propositions.*/
	    
	    for(uint preconditionIndex = 0
		    ; preconditionIndex < preconditions.size()
		    ; preconditionIndex++){
		const SignedPredicate& predicate = *preconditions[preconditionIndex];
		const PredicateName& predicateName = predicate.getName();
	    
		if(problem.domain.isConstant(predicateName) ||
		   problem.domain.isFluentToNegative(predicateName)){
		
		    assert(propositionIndices.find(predicateName) != propositionIndices.end());
		
		    const pair<uint,uint>& lowerUpper = propositionIndices.find(predicateName)->second;

		    /*<size of model, index>.*/
		    pair<uint, uint> rank(lowerUpper.second - lowerUpper.first, preconditionIndex);
		
		    sortedPredicateIndices.insert(rank);
		}
	    }
	
	    uint beforeActionsSize = actions.size();
	    Constants argument(arguments.size());
	    VariableToConstant variableToConstant;

#ifndef NDEBUG
	    //dynamic_cast<set<pair<uint, uint> >::const_iterator&>(sortedPredicateIndices.begin());/*INDEX FOR (X)*/
	    dynamic_cast<Constants&>(argument);
	    dynamic_cast<VariableToConstant&>(variableToConstant);
	    dynamic_cast<const Variables&>(variablesInOrder);
	    dynamic_cast<const VariableToUnsignedInt&>(variableIndex);/****/
	    dynamic_cast<vector<Constants >&>(arguments);
	    dynamic_cast<const Action<>&>(actionSchema);
	    dynamic_cast<const SignedPredicates&>(preconditions);
	    dynamic_cast<const SignedPredicates&>(effects);
	    dynamic_cast<const set<pair<uint, uint> >&>(sortedPredicateIndices);/*(X)*/
	    dynamic_cast<const vector<bool>&>(choiceVariables);
#endif
	
	    VERBOSER(18, "Call to add groundings for action :: "<<actionSchema.getName()<<endl);
	    addActionGroundings(sortedPredicateIndices.begin(),/*INDEX FOR (X)*/
				argument,
				variableToConstant,
				variablesInOrder,
				variableIndex,/****/
				arguments,
				actionSchema,
				preconditions,
				effects,
				sortedPredicateIndices,/*(X)*/
				choiceVariables,
				argumentsToCheck);
	
	    actionIndices[name] =
		pair<uint, uint>(beforeActionsSize, actions.size());

	    if(beforeActionsSize >= actions.size()){
		WARNING("There are no instances of action :: "<<actionSchema.getName()<<endl);
	    }

	    /*It is allowed to be the case that no actions of a particular
	     * type exist in a problem.*/
	    //assert(beforeActionsSize < actions.size());
	
	    VERBOSER(15, "We added "<<actions.size() - beforeActionsSize<<" more actions."<<endl);
	
	    VERBOSER(15, "---- ADDED ALL GROUNDINGS FOR ACTION :: "<<actionSchema.getName()<<endl);
	}

    
	/*We must have some actions, otherwise we don't have a planning problem.*/
	assert(actions.size() > 0);
    

	VERBOSER(18, "We have :: "<<actions.size()<<" problem actions."<<endl);
    

	VERBOSER(18, "We have :: "<<propositions.size()<<" propositions all up."<<endl);
	//VERBOSER(16, "Actions are :: \n "<<actions<<endl);


#if DEBUG_LEVEL == 18
	for(uint i = 0; i < actions.size(); i++){
	    cerr<<actions[i]<<endl<<endl;
	}
#endif
    
    }

    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_startingState()
    {
	stateSize = fluentPropositions.size();
	const StartingState& _startingState = problem.getStartingState();

	startingState = StateType(stateSize);

	for(StartingState::const_iterator startingProposition = _startingState.begin()
		; startingProposition != _startingState.end()
		; startingProposition++){

	    assert(fluentPropositionIndex.size() == fluentPropositions.size());

	    /*Assume that any proposition describing the starting state is
	     *already known to the planner via \member{propositionIndex}.*/
#ifndef NDEBUG
	    if(propositionIndex.find(*startingProposition) == propositionIndex.end()){
		UNRECOVERABLE_ERROR("Could not find starting-state proposition :: "<<*startingProposition<<endl
				    <<" in groundings of propositions.\n"<<endl
				    <<"Propositions are :: "<<propositions<<endl);
	    }
#endif

	    /*If the start-state proposition is indeed a fluent, then the
	     * state space search will have to keep track of its truth
	     * value.*/
	    if(fluentPropositionIndex.find(*startingProposition) != fluentPropositionIndex.end()){
	    
		assert(!problem.domain.isConstant(startingProposition->getName()));
	    
		uint index = fluentPropositionIndex[*startingProposition];

		assert(index < fluentPropositions.size());
	    
		startingState.flipOn(index);
	    } else {

		assert(problem.domain.isConstant(startingProposition->getName()));
	    }
	}

	VERBOSER(19, "We have the starting state :: "<<startingState<<endl);
    
	assert(actions.size() > 0);
	startingState.ComputePossibleActions(actions);
	assert(startingState.actions.size() > 0);
    }


    template<typename StateType, typename GroundActionType>
    bool  Planner<StateType, GroundActionType>::isGoalState(const StateType& state) const
    {
	const Goal& _goalState = problem.getGoal();

	VERBOSER(26, endl);
	
	for(Goal::const_iterator goalProposition = _goalState.begin()
		; goalProposition != _goalState.end()
		; goalProposition++){
	
	    Proposition<> tmp(*goalProposition);
	
	    /* A GOAL PROPOSITION COULD BE A FALSE NON-FLUENT... Assume
	     * that any proposition describing the goal state is already
	     * known to the planner via \member{propositionIndex}.*/
	    //	assert(propositionIndex.find(tmp) != propositionIndex.end());

	    VERBOSER(26, "Checking for goal :: "<<*goalProposition);//tmp<<" ");

	    /*If the symbol is fluent.*/
	    if(!problem.domain.isConstant(tmp)){

		PropositionToInt::const_iterator _goalPropositionIndex = fluentPropositionIndex.find(tmp);//->second;

		if(_goalPropositionIndex == fluentPropositionIndex.end()){
		    UNRECOVERABLE_ERROR("Fluent proposition :: "
					<<tmp<<" is not characterised by a state proposition."<<endl);
		}
		
		
		assert(_goalPropositionIndex != fluentPropositionIndex.end());

		uint goalPropositionIndex = _goalPropositionIndex->second;
		
		if(goalProposition->isPositive()){
		    if(!state.isTrue(goalPropositionIndex)) {
			VERBOSER(26, " failed"<<endl);
			return false;
		    } else {
		    
			VERBOSER(26, " passed"<<endl);
		    }//mustBeTrue.push_back(propositionIndex[tmp]);
		} else {
		    if(!state.isFalse(goalPropositionIndex)) {
			VERBOSER(26, " failed"<<endl);
			return false;
		    } else {
			VERBOSER(26, " passed"<<endl);
		    }//propositionIndex[tmp];
		}
	    } else {
		/*The case that the symbol is non-fluent, and hence is
		 * only ever true if it was true in the starting state.*/
		if(goalProposition->isPositive()){
		    if(propositionIndex.find(tmp) == propositionIndex.end()){
			return false;
			VERBOSER(26, " failed"<<endl);
		    } else {
			
			VERBOSER(26, " passed"<<endl);
		    }
		} else {
		    if(propositionIndex.find(tmp) != propositionIndex.end()){

			UNRECOVERABLE_ERROR("The goal can not be achieved because :: "<<tmp<<endl
					    <<"is in the goal and not in the starting state."<<endl
					    <<"Because :: "<<tmp<<" is invariant the goal can not"<<endl
					    <<"be achieved."<<endl);
		    
			return false;
		    }
		}
	    }
	}
	
	VERBOSER(26, "Got a goal state!\n"<<endl);
	DEBUG_GET_CHAR(26);
	return true;
    }


    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_goalState()
    {
	stateSize = fluentPropositions.size();
	const Goal& _goalState = problem.getGoal();
    
	for(Goal::const_iterator goalProposition = _goalState.begin()
		; goalProposition != _goalState.end()
		; goalProposition++){
	
	    Proposition<> tmp(*goalProposition);

	    /*Assume that any proposition describing the goal state is
	     * already known to the planner via \member{propositionIndex}.*/
	    assert(propositionIndex.find(tmp) != propositionIndex.end());

	    /*Here we only care about goal propositions that are not
	     *invariant. If there is a non-fluent proposition that is in
	     *the goal and not in the start state, then we know that the
	     *planning problem is not possible to solve.*/
	    if(fluentPropositionIndex.find(tmp) != fluentPropositionIndex.end()){

		assert(!problem.domain.isConstant(tmp));
	    
		if(goalProposition->isPositive()){
		    mustBeTrue.push_back(fluentPropositionIndex[tmp]);
		} else {
		    UNRECOVERABLE_ERROR("We do not support negative goal propositions,"<<endl
					<<"and yet we were given :: "<<*goalProposition<<endl);
	    
		    mustBeFalse.push_back(fluentPropositionIndex[tmp]);
		}
	    } else {
		if(!problem.domain.isConstant(tmp)){
		    UNRECOVERABLE_ERROR("Goal proposition :: "<<tmp<<" can not be achieved."<<endl);
		}	
	    }
	}
    
	{/*Make \member{goalTrue}*/
	    StateType tmp(startingState.getNumPropositions());
	    for(int i = 0; i < mustBeTrue.size(); i++){
		tmp.flipOn(mustBeTrue[i]);
	    }

	    tmp.mimick(goalTrue);
	}

	{/*Make \member{goalFalse}*/
	    StateType tmp(startingState.getNumPropositions());
	
	    if(mustBeFalse.size() > 0){
		UNRECOVERABLE_ERROR("Expecting no goal condition"
				    <<" that requires propositions are false."<<endl);
	    }
	
	    for(int i = 0; i < mustBeFalse.size(); i++){
		tmp.flipOn(mustBeFalse[i]);
	    }

	    /*Make sure the propositions that are extraneous due to the
	     * bit-vector representation are not represented as having to
	     * be true.*/
	    for(uint i = tmp.getNumPropositions() + 1; i < SIZE_ELEM * tmp.size(); i++){
		tmp.flipOff(i);
	    }
	
	    tmp.mimick(goalFalse);


#ifndef NDEBUG
	    VERBOSER(14, "Element size is :: "<<SIZE_ELEM<<endl
		     <<"number of elements is :: "<<tmp.size()<<endl);

	    //exit(0);
	
	    for(uint i = 0; i < SIZE_ELEM * tmp.size(); i++){
		VERBOSER(14, i<<" = "<<tmp.isTrue(i)<<endl);
	    }
	
	    for(int i = 0 ; i < goalFalse.size() ; i++){
		assert(!goalFalse[i]);
	    }
#endif

	}

    }

    
    
    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_goalEvaluator()
    {
	GoalProximityEvaluator<StateType>& goalEvaluator = unexpanded.stateEvaluator;
	
	const Goal& _goalState = problem.getGoal();

	for(Goal::const_iterator goalProposition = _goalState.begin()
		; goalProposition != _goalState.end()
		; goalProposition++){
	
	    Proposition<> tmp(*goalProposition);
	    if(!problem.domain.isConstant(tmp)){
		if(goalProposition->isPositive()){
		    assert(fluentPropositionIndex.find(tmp) != fluentPropositionIndex.end());
		    
		    goalEvaluator.goalState.push_back(fluentPropositionIndex.find(tmp)->second);
		}
	    }
	    
	}
    }
    
    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::addGroundingsFromStartState(const PredicateName& predicateName)
    {
	const StartingState& startingState = problem.getStartingState();

	/*If we are talking about equality, then we shall have to
	 * generate the models ourselves.*/
	if(predicateName == "=" || predicateName == "!="){
	    const Types&  types = problem.domain.types;
	    Constants objects;
	    problem.getObjects(types, objects);
	    
	    VERBOSER(21, "Dealing with equality or inequality symbol.");
	    
	    /*For every constant and object symbol.*/
	    for(Constants::const_iterator object1 = objects.begin()
		    ; object1 != objects.end()
		    ; object1++){
		/*For every constant and object symbol.*/
		for(Constants::const_iterator object2 = objects.begin()
			; object2 != objects.end()
			; object2++){
		    
		    Constants arguments;
		    arguments.push_back(*object1);
		    arguments.push_back(*object2);
		    
		    if(predicateName == "=" && object1 == object2){
			VERBOSER(22, "Adding :: "<<*object1<<"="<<*object2<<endl);
			
			Proposition<> proposition("=", arguments);
			propositionIndex[proposition] = propositions.size();
			propositions.push_back(proposition);
		    } else if (predicateName == "!=" && object1 != object2) {
			VERBOSER(22, "Adding :: "<<*object1<<"!="<<*object2<<endl);
			
			Proposition<> proposition("!=", arguments);
			propositionIndex[proposition] = propositions.size();
			propositions.push_back(proposition);
		    }
		}
	    }
	} else {
	
	    
	    /*FIX :: This is not efficient. It takes us linear time in the
	     * size of the starting state to do this, however if we had
	     * already partitioned the starting state into the different
	     * predicates that occur in it, this would be faster.*/
	    
	    for(StartingState::const_iterator proposition = startingState.begin()
		    ; proposition != startingState.end()
		    ; proposition++){
		if(proposition->getName() == predicateName){
		    propositionIndex[*proposition] = propositions.size();
		    propositions.push_back(*proposition);
		}
	    }
	}
    }

    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::addGroundings(uint index,
							      Constants& argument,
							      const vector<Constants >& arguments,
							      const PredicateName& predicateName)
    {
	assert(propositions.size() == propositionIndex.size());
    
	if(index >= arguments.size()){
	    Proposition<> tmp(predicateName, argument);

	    VERBOSER(6, "Indexing proposition :: "<<tmp<<endl);
	
	    propositionIndex[tmp] = propositions.size();
	    propositions.push_back(tmp);
	} else {
	    for(uint i = 0; i < arguments[index].size(); i++){
		argument[index] = arguments[index][i];
		addGroundings(index + 1, argument, arguments, predicateName);
	    }
	}
    }


    /*For each predicate symbol from the planning domain, ground that
     * symbol according to its arguments.*/
    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::make_propositions()
    {
	const PredicateNameSpecifications& predicates
	    = problem.domain.predicateSpecifications;

	for(PredicateNameSpecifications::const_iterator predicate = predicates.begin()
		; predicate != predicates.end()
		; predicate++){
	
	    const PredicateName& predicateName = predicate->first;
	
	    VERBOSER(3, "Grounding predicate with name :: "<<predicateName<<endl);
	
	    /*If \local{predicate} is constant, or if its truth value can
	     *only transition to false.*/
	    if(problem.domain.isConstant(predicateName) ||
	       problem.domain.isFluentToNegative(predicateName)){
	    
		uint beforePropositionsSize = propositions.size();

		if(  predicateName == "="){
		    
		    addGroundingsFromStartState(PredicateName("="));
	    
		    propositionIndices[PredicateName("=")] =
			pair<uint, uint>(beforePropositionsSize, propositions.size());
		    
		    beforePropositionsSize = propositions.size();
		
		    addGroundingsFromStartState(PredicateName("!="));
	    
		    propositionIndices[PredicateName("!=")] =
			pair<uint, uint>(beforePropositionsSize, propositions.size());
		} else {
		    addGroundingsFromStartState(predicateName);
	    
		    propositionIndices[predicateName] =
			pair<uint, uint>(beforePropositionsSize, propositions.size());
		}   

#ifndef NDEBUG
		for(uint i = beforePropositionsSize; i < propositions.size(); i++){
		    VERBOSER(17, " Proposition :: "<<propositions[i]<<endl);
		}
#endif
	    
	    
		continue;
	    }

	    /*FROM HERE ON WE HAVE THAT THE PREDICATE IS FLUENT, THUS WE
	     *HAVE TO CONSIDER ALL INSTANTIATIONS TO IT.*/
	
	
	    /*Set of admissible constants for the 1, 2, .., nth
	     * argument.*/
	    vector<Constants > arguments(getArity(predicate->second));
	
	    // 	for(uint i = 0; i < getArity(predicate->second); i++ ){
	    // 	    arguments.push_back(Constants());
	    // 	}	
	
	    const Arguments& _arguments = predicate->second;

	    /*Index in to \local{arguments}.*/
	    uint argCount = arguments.size() - 1;
	
	    /*Remember the arguments were parsed backwards.*/
	    for(Arguments::const_iterator argument = _arguments.begin()
		    ; argument != _arguments.end()
		    ; argument++){


		const Types& types = argument->first;
		const Variables& variables  = argument->second;
	    
		/* The order in which we parsed the variables is backwards
		 * remember (CHECKED).*/
	    
		for(int varIndex = 0; varIndex < variables.size(); varIndex ++){//int varIndex = variables.size() - 1; varIndex >= 0; varIndex--){
		
		    VERBOSER(3, "Unwinding var :: "<<*types.begin()<<endl);
		    problem.getObjects(types, arguments[argCount]);

		    if(arguments[argCount].size() == 0){
			WARNING("There are no objects of type :: "<<*types.begin()<<endl);
		    } else {
			VERBOSER(3, "There are "<<arguments[argCount].size()
				 <<" object/s of type: : "<<*types.begin()<<endl);
		    }
		
		    /*Listed the objects associated with one more argument.*/
		    argCount--;
		    assert(argCount>=0);
		}
	    }

	    uint beforePropositionsSize = propositions.size();
	    Constants argument(arguments.size());
	    addGroundings(0, argument, arguments, predicateName);
	
	    propositionIndices[predicateName] =
		pair<uint, uint>(beforePropositionsSize, propositions.size());

	
#ifndef NDEBUG
	    for(uint i = beforePropositionsSize; i < propositions.size(); i++){
		VERBOSER(17, " Proposition :: "<<propositions[i]<<endl);
	    }
#endif
	  
	}

	VERBOSER(19, "Propositions are :: "<<propositions<<endl);
    }

    template<typename StateType, typename GroundActionType>
    void  Planner<StateType, GroundActionType>::prune_propositions()
    {
	for(MapStringToPairInt::const_iterator propositionIndex = propositionIndices.begin()
		; propositionIndex != propositionIndices.end()
		; propositionIndex++){

	    
	    if(propositionIndex->second.second <= propositionIndex->second.first){
		WARNING("Indices to models of :: "<<propositionIndex->first<<endl
			<<" ** are lower :: "<<propositionIndex->second.first<<" upper :: "<<propositionIndex->second.second<<endl
			<<" ** hence we have no models of this predicate, and yet it"<<endl
			<<" ** appears in the domain specification precondition.\n");

		continue;
	    }
	    
// 	    if(propositionIndex->second.first >= propositions.size()){
// 		VERBOSER(25, "Proposition :: "<<propositionIndex->first<<" at index ::"<<propositionIndex->second.first
// 			 <<" to "<<propositionIndex->second.second<<endl
// 			 <<"does not appear as a problem proposition in :: "<<propositions.size()<<endl);
		
// 		for(uint i = 0 ; i < propositions.size() ; i++){
// 		    VERBOSER(25, propositions[i]<<endl);
// 		}
// 		exit(0);
		
// 	    }
	    
	    //assert(propositionIndex->second.first < propositions.size());


	    const PredicateName& predicateName = propositionIndex->first;
	
	    /*If the symbol is non-constant (i.e., fluent)*/
	    if(!problem.domain.isConstant(predicateName)){
	    
		uint oldSize = fluentPropositions.size();
	
		for(uint index = propositionIndex->second.first
			; index < propositionIndex->second.second
			; index ++){
		
		    fluentPropositionIndex[propositions[index]]
			= fluentPropositions.size();
		    fluentPropositions.push_back(propositions[index]);
		}
	    
		fluentPropositionIndices[propositionIndex->first]
		    = pair<uint, uint>(oldSize, fluentPropositions.size());
	    }
	}
	VERBOSER(18, "We have :: "<<propositions.size()<<" FLUENT propositions."<<endl);
    }
}



namespace Planning
{
    template<typename StateType, typename GroundActionType>
    Planner<StateType, GroundActionType>::~Planner()
    {
	//     for(uint i = 0; i < actions.size(); i++){
	// 	FAKE_DELETE(actions[i]);
	//     }
    }
    
}

/*see \module{GroundAction}*/
#ifndef NDEBUG
namespace GoundAction_Planner
{
    extern void* planner;
}
#endif

namespace Planning
{
    template<typename StateType, typename GroundActionType>
    Planner<StateType, GroundActionType>::Planner(Problem& problem)
	:problem(problem),
	 goalState(0)
    {
#ifndef NDEBUG /*see \module{GroundAction}*/
	GoundAction_Planner::planner = this;
#endif

	/*The planner shares some of its members with the
	 * state. Unfortunately, a state does not know the type of planner
	 * that it is being used for.*/
	StateType::fluentPropositions = &fluentPropositions;
	StateType::plannerActions = &actions;
	StateType::propositionIndices = &propositionIndices;
	StateType::problem = &problem;
	StateType::propositions = &propositions;
    
    
	//     /*We assume here that the application has one \class{Planner} per
	//      * execution thread. If that is not the case, then we should have
	//      * each \class{State} refer to a different planner.*/
	//     StateType::planner = this;
    
	VERBOSER(14, "Making the propositions."<<endl);
    
	make_propositions();
	assert(propositions.size() == propositionIndex.size());

	VERBOSER(14, "Making the actions."<<endl);
    
	make_actions();
	assert(propositions.size() == propositionIndex.size());
    
	VERBOSER(14, "(optional) Pruning the set of available propositions."<<endl);

	prune_propositions();
    
	VERBOSER(14, "Generating the set of ground actions with integer representation."<<endl);
    
	/*Make sure that action preconditions are in terms of integers
	 * rather than propositional (i.e. represented in strings).*/
	for(typename vector<GroundActionType>::iterator action =  actions.begin()
		; action != actions.end()
		; action++){
	    //action->generateIntegerRepresentation(propositionIndex);
	    action->generateIntegerRepresentation(fluentPropositionIndex);
	}
	assert(propositions.size() == propositionIndex.size());
    

	VERBOSER(14, "Making a preliminary version of the starting state."<<endl);
    
	make_startingState();
	
	VERBOSER(14, "Further pruning of action and propositional symbols."<<endl);

	StaticRelevance<StateType, GroundActionType> staticRelevance(*this);
	staticRelevance();
	VERBOSER(27, "Now got :: "<<staticRelevance.allowedActionIndices.size()<<" allowed actions."<<endl);
	VERBOSER(27, "Had :: "<<actions.size()<<" allowed actions initially."<<endl);
	
	VERBOSER(27, "Now got :: "<<staticRelevance.allowedPropositionIndices.size()<<" allowed fluents."<<endl);
	VERBOSER(27, "Has :: "<<fluentPropositions.size()<<" allowed fluents initially."<<endl);
	
	removeActions(staticRelevance.allowedActionIndices);	
	removeFluents(staticRelevance.allowedPropositionIndices);

	
	VERBOSER(14, "Configuring the actions integer representation"<<endl
		 <<" given the _relevant_ set of actions and fluents."<<endl);
    
	for(typename vector<GroundActionType>::iterator action =  actions.begin()
		; action != actions.end()
		; action++){
	    action->generateIntegerRepresentation(fluentPropositionIndex);
	}
	
	VERBOSER(27, "Got :: "<<actions.size()<<" allowed actions after pruning."<<endl);
	VERBOSER(27, "Got :: "<<fluentPropositions.size()<<" allowed fluents after pruning."<<endl);
	
	VERBOSER(14, "Re-making the starting state."<<endl);

	/*The starting state has changed because some actions and
	 * fluents are no longer considered relevant.*/
	make_startingState();
	assert(propositions.size() == propositionIndex.size());

	VERBOSER(14, "Making the goal state."<<endl);
    
	make_goalState();
	assert(propositions.size() == propositionIndex.size());
	make_goalEvaluator();

	VERBOSER(14, "Setting action possibilities."<<endl);
    
	/*Make sure every proposition that is mentioned in the
	 * precondition of an action, knows of that action.*/
	make_setActionPossibilities();
	assert(propositions.size() == propositionIndex.size());
    }
    
    template<typename StateType, typename GroundActionType>
    void Planner<StateType, GroundActionType>::removeActions(set<uint> allowedActionIndices)  
    {
	vector<GroundActionType> _actions;

	for(uint _actionIndex = 0; _actionIndex < actions.size(); _actionIndex++){
	    if(allowedActionIndices.find(_actionIndex) != allowedActionIndices.end()){

		assert(_actionIndex < actions.size());
		_actions.push_back(actions[_actionIndex]);
	    } 
	}
	
	actions = _actions;

	assert(actions.size());
	
	actionIndices = MapStringToPairInt();
	actionIndex = GroundActionToInt();
	
	PredicateName predicateName(actions[0].name.getName());
	uint lower = 0;
	for(uint _actionIndex = 0; _actionIndex < actions.size(); _actionIndex++){
	    if(predicateName != actions[_actionIndex].name.getName()){
		actionIndices[predicateName] = pair<uint, uint>(lower, _actionIndex);
		lower = _actionIndex;
		predicateName = actions[_actionIndex].name.getName();
	    }

	    actionIndex[actions[_actionIndex]] = _actionIndex;
	}
    }
    
    template<typename StateType, typename GroundActionType>
    void Planner<StateType, GroundActionType>::removeFluents(set<uint> allowedFluentIndices)  
    {
	vector<Proposition<> > _fluentPropositions;

	for(uint _fluentIndex = 0; _fluentIndex < fluentPropositions.size(); _fluentIndex++){
	    if(allowedFluentIndices.find(_fluentIndex) != allowedFluentIndices.end()){

		assert(_fluentIndex < fluentPropositions.size());
		_fluentPropositions.push_back(fluentPropositions[_fluentIndex]);
	    } 
	}
	
	fluentPropositions = _fluentPropositions;

	assert(fluentPropositions.size());
	
	fluentPropositionIndices = MapStringToPairInt();
	fluentPropositionIndex = PropositionToInt();
	
	PredicateName predicateName(fluentPropositions[0].getName());
	uint lower = 0;
	for(uint _fluentIndex = 0; _fluentIndex < fluentPropositions.size(); _fluentIndex++){
	    if(predicateName != fluentPropositions[_fluentIndex].getName()){
		fluentPropositionIndices[predicateName] = pair<uint, uint>(lower, _fluentIndex);
		lower = _fluentIndex;
		predicateName = fluentPropositions[_fluentIndex].getName();
	    }

	    fluentPropositionIndex[fluentPropositions[_fluentIndex]] = _fluentIndex;
	}

    }
}

namespace Planning
{
    template<typename StateType, typename GroundActionType>
    ostream& operator<<(ostream& o, const Planner<StateType, GroundActionType>& planner)
    {

	VERBOSER(6, "We have :: "<<planner.propositions.size()
		 <<" ( check: "<<planner.propositionIndex.size()<<" propositions)"<<endl);
	
	assert(planner.actions.size() == planner.actionIndex.size());
	assert(planner.propositions.size() == planner.propositionIndex.size());


	
	o<<planner.propositions<<endl;
	
	for_each(planner.actions.begin(),
		 planner.actions.end(),
		 print_elements<GroundActionType>(o, planner.actions.size()));
	
	return o;
    }
}

#endif
