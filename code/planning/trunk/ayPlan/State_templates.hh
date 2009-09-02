// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef STATE_TEMPLATES_HH
#define STATE_TEMPLATES_HH

#include<cmath>

#include"Planner.hh"


// template<typename GroundActionType>
// vector<Proposition<> >* State<GroundActionType>::fluentPropositions = 0;

// template<typename GroundActionType>
// vector<GroundActionType>* State<GroundActionType>::plannerActions = 0;

// template<typename GroundActionType>
// MapStringToPairInt* State<GroundActionType>::propositionIndices = 0;

// template<typename GroundActionType>
// Problem* State<GroundActionType>::problem = 0;

// template<typename GroundActionType>
// vector<Proposition<> >* State<GroundActionType>::propositions = 0;


#ifndef NATHAN
using namespace tr1;
#endif

// template<typename GroundActionType>
// MapIntToInts State<GroundActionType>::propositionsAllowsActions;

//Planner* State<GroundActionType>::planner = 0;


#include <zlib.h>

namespace Planning
{

    class Database
    {
    public:
        void writeToDatabase()
        {
        }
    private:
    };
    
    
    template<typename GroundActionType>
    void State<GroundActionType>::compress() 
    {
        assert(sizeof(ELEM_TYPE) == sizeof(unsigned char));

        if(compressed)
            return;

        //DEBUG // data.resize(1024);
        
        unsigned char* output = new unsigned char [data.size()];
        unsigned char* input = new unsigned char [data.size()];

        for(int i = 0; i < data.size(); i++){
            input[i] = data[i];
        }
        
        int result = Compression::compress(input, data.size(),
                                           output, data.size(), 9);

        if(result >= data.size()){
            UNRECOVERABLE_ERROR("Unable to compress "<<data.size()<<endl
                                <<"compression yielded "<<result<<" elements"<<endl);
        } else {
            UNRECOVERABLE_ERROR("ABLE to compress "<<data.size()<<endl
                                <<"compression yielded "<<result<<" elements"<<endl);
        }
        
        
        
        assert(result < data.size());
        
        cxx_realloc(output, data.size(), data.size() - result);
        compressed_representation = output;
        compressed_representation_size = data.size() - result;
        
        delete [] input;
        data = vector<ELEM_TYPE>(0);
        compressed = true;
    }
    
    template<typename GroundActionType>
    void State<GroundActionType>::decompress() 
    {
        assert(sizeof(ELEM_TYPE) == sizeof(unsigned char));

        if(!compressed)
            return;
        
        data = vector<ELEM_TYPE>(CEIL(numPropositions));
        unsigned char* output = new unsigned char [data.size()];
        
        int result = Compression::decompress(compressed_representation, compressed_representation_size,
                                            output, data.size());

        for(int i = 0; i < data.size(); i++){
            data[i] = output[i];
        }

        delete [] compressed_representation;
        delete [] output;
        compressed = false;
    }
    
    template<typename GroundActionType>
    vector<Proposition<> >* State<GroundActionType>::fluentPropositions = 0;

    template<typename GroundActionType>
    vector<GroundActionType>* State<GroundActionType>::plannerActions = 0;

    template<typename GroundActionType>
    MapStringToPairInt* State<GroundActionType>::propositionIndices = 0;

    template<typename GroundActionType>
    Problem* State<GroundActionType>::problem = 0;

    template<typename GroundActionType>
    vector<Proposition<> >* State<GroundActionType>::propositions = 0;


    using namespace tr1;

    template<typename GroundActionType>
    MapIntToInts State<GroundActionType>::propositionsAllowsActions;


    template<typename GroundActionType>
    uint State<GroundActionType>::getTime()const 
    {
	return time;
    }

    template<typename GroundActionType>
    void State<GroundActionType>::setTime(uint time)
    {
	this->time = time;
    }

    template<typename GroundActionType>
    void State<GroundActionType>::removeStateReferences(const SetOfStatePointers& setOfStatePointers)
    {
	vector<State<GroundActionType>* > toRemove(successorToAction.size());

	uint index = 0;
    
	for(typename MapStatePointerToUint::iterator successorElement = successorToAction.begin()
		; successorElement != successorToAction.end()
		; successorElement++){
	
	    if(setOfStatePointers.find(successorElement->first) != setOfStatePointers.end()){
		toRemove[index++] = successorElement->first;
	    }
	}


	for(uint i = 0; i < index ; i ++){
	    successorToAction.erase(toRemove[i]);
	}
    }



    
    template<typename GroundActionType>
    void State<GroundActionType>::actionYields(uint actionIndex, State<GroundActionType>* successor)
    {
	successorToAction[successor] = actionIndex;
    }

    template<typename GroundActionType>
    void State<GroundActionType>::setParent(State<GroundActionType>* input)
    {
	parent = input;
    }

    template<typename GroundActionType>
    void State<GroundActionType>::ComputePossibleActions(const vector<GroundActionType>& groundActions)
    {

	assert(groundActions.size() > 0);
    
	for(uint action = 0; action < groundActions.size(); action++){

	
	
	    VERBOSER(19, "Testing a :: "<<groundActions[action].name<<" for execution."<<endl);
	
	    if(possible(groundActions[action])){
		VERBOSER(19, "We find that action :: "<<groundActions[action]<<endl
			 <<"at index :: "<<action<<" is possible"<<endl);
		actions.insert(action);
	    } else {
		VERBOSER(19, "We find that action :: "<<groundActions[action]<<endl
			 <<"at index :: "<<action<<" is NOT possible"<<endl);
	    }
	}
    }

    template<typename GroundActionType>
    void State<GroundActionType>::UpdatePossibleActions()
    {
	/*FIX :: This is totally inefficient. It is called from
	 * \member{execute}. Essentially, because actions can not have
	 * negative preconditions, we have that any proposition that is
	 * deleted should remove any actions it permits from
	 * \member{actions}. Moreover, any proposition that is added, we
	 * should check if we can add the action. Removing actions from
	 * the set we allow via deleted propositions is trivial. For
	 * allowed propositions, we should only test the preconditions of
	 * actions they _may_ permit by first checking we have not already
	 * allowed that action in previous processing.*/
    
	/*REMOVE ALL THE ACTIONS THAT ARE NO LONGER POSSIBLE.*/

	/*If we are going to "update the possible actions", then in the
	 *first place we assume that some actions are possible.*/
	assert(actions.size() > 0);

	/*We assume that some propositions have changed, otherwise we
	 *would not have to update the set of actions that are
	 *possible. If is possible of course to execute an action that has
	 *no effect on the state, so we are only going to throw a warning
	 *if this happens.*/
	if(propositionsChanged.size() == 0){
	    //WARNING("Executed action with no effects.");
	    return;
	}
	//assert(propositionsChanged.size() > 0);
    
	/*ADD NEW ACTIONS THAT HAVE BECOME AVAILABLE.*/
    
	for(vector<uint>::const_iterator proposition = propositionsChanged.begin()
		; proposition != propositionsChanged.end()
		; proposition++){
	    MapIntToInts::const_iterator propActionsPair
		= State<GroundActionType>::propositionsAllowsActions.find(*proposition);
	
	
	    //assert(propActionsPair != State<GroundActionType>::propositionsAllowsActions.end());


	    /*If a proposition has been associated with the precondition of an action.*/
	    if(propActionsPair != State<GroundActionType>::propositionsAllowsActions.end()){
	    
		VERBOSER(16, " ## Adding "<<propActionsPair->second.size()
			 <<" actions associated with proposition :: "
			 <<(*State<GroundActionType>::fluentPropositions)[*proposition]<<endl);
	    
		assert(propActionsPair->second.size() > 0);
	    
		for(SetOfUnsignedInts::const_iterator action = propActionsPair->second.begin()
			; action != propActionsPair->second.end()
			; action++){
		    actions.insert(*action);
		
		    VERBOSER(16, " ## Adding potential action :: "<<*action<<endl);
		}
	    } else {
		VERBOSER(16, " ## There are no actions associated with proposition :: "
			 <<(*State<GroundActionType>::fluentPropositions)[*proposition]<<endl);
	    }
	}

    
	/*FIX: This is inefficient, because we only really need to check
	 * actions whose preconditions have been deleted. So check all the
	 * actions for deletion is a waste of time.*/
	vector<uint> toErase;
	for(SetOfUnsignedInts::const_iterator action = actions.begin()
		; action != actions.end()
		; action++){
	    if(!possible((*State<GroundActionType>::plannerActions)[*action])){
		toErase.push_back(*action);
	    }
	}

	for(vector<uint>::const_iterator p = toErase.begin()
		; p != toErase.end()
		; p++){
	    VERBOSER(16, " ## Removing potential action :: "<<*p<<endl);
	    actions.erase(*p);//toErase.begin(), toErase.end());
	}
    
	if(actions.size() == 0){
	    WARNING("[from \\class{State}] Generated state :: "<<*this<<endl
		    <<"** from which no actions can be executed."<<endl);
	}

	if(!actions.size()){
	    WARNING("Generated state with no successors!"<<endl);
	}
    
	//assert(actions.size() > 0);
    }

    template<typename GroundActionType>
    bool State<GroundActionType>::possible(const GroundActionType& groundAction) const
    {
	if(groundAction.prec.size() == 0){
	    WARNING("Evaluating the possibility of executing an action with no precondition."<<endl);
	}
    
	for(uint i = 0; i < groundAction.prec.size(); i++){
	    if(!isTrue(groundAction.prec[i])){
		return false;
	    }

	    //VERBOSER(11, "Action precondition :: "<<groundAction._prec[i]<<" was met..."<<endl);
	    VERBOSER(11, " ** Action precondition :: "<<(*State<GroundActionType>::fluentPropositions)[groundAction.prec[i]]
		     <<" was met..."<<endl);
	}
    
	return true;
    }

    template<typename GroundActionType>
    State<GroundActionType>& State<GroundActionType>::execute(const GroundActionType& groundAction, State<GroundActionType>& result) const
    {
	VERBOSER(21, "Executing action :: "<<groundAction<<endl);
	VERBOSER(21, "At state :: "<<result<<endl);
    
	assert(possible(groundAction));

	/*We do not execute an action in a state that has not actions. We
	 *also expect the result, before we execute an action, to be a
	 *clone of the \class{State} \local{*this}.*/
	assert(result.actions.size() > 0);
    
	/*Apply the delete list.*/
	for(uint i = 0; i < groundAction.del.size(); i++){
	    if(!result.isFalse(groundAction.del[i])){
		VERBOSER(21, "Flipping off :: "<<(*State<GroundActionType>::fluentPropositions)[groundAction.del[i]]<<endl);
		result.flipOff(groundAction.del[i]);
		result.propositionsChanged.push_back(groundAction.del[i]);
	    }
	}

	/*Apply the add list.*/
	for(uint i = 0; i < groundAction.add.size(); i++){
	    if(!result.isTrue(groundAction.add[i])){
		VERBOSER(21, "Flipping on :: "<<(*State<GroundActionType>::fluentPropositions)[groundAction.add[i]]<<endl);
		result.flipOn(groundAction.add[i]);
		result.propositionsChanged.push_back(groundAction.add[i]);
	    }
	}

	assert(result.actions.size() == this->actions.size());
    
	/*Keep track of what actions are executable in the
	 *\local{result}ant state.*/
	result.UpdatePossibleActions();

    
	if(result.actions.size() == 0){
	    WARNING("[from \\class{State}] Generated \\local{result} state ::"<<result<<endl
		    <<"** from which no actions can be executed."<<endl);
	}
    
	result.time = this->time + 1;
	return result;
    }

    template<typename GroundActionType>
    State<GroundActionType> State<GroundActionType>::execute(const GroundActionType& groundAction) const
    {
	assert(possible(groundAction));

	State<GroundActionType> result(*this);
    
	execute(groundAction, result);

	return result;
    }

    template<typename GroundActionType>
    State<GroundActionType> State<GroundActionType>::execute_noDeleteEffects(const GroundActionType& groundAction) const
    {
	assert(possible(groundAction));
    
	State<GroundActionType> result(*this);
    
	/*Apply the add list.*/
	for(uint i = 0; i < groundAction.add.size(); i++){
	    if(!result.isTrue(groundAction.add[i])){
		result.flipOn(groundAction.add[i]);
		//result.propositionsChanged.push_back(groundAction.add[i]);
	    }
	}
    
	return result;
    }
    
    template<typename GroundActionType>
    State<GroundActionType>& State<GroundActionType>::execute_noDeleteEffects(const GroundActionType& groundAction,
									      State<GroundActionType>& result) const
    {
	assert(possible(groundAction));
    
	/*Apply the add list.*/
	for(uint i = 0; i < groundAction.add.size(); i++){
	    if(!result.isTrue(groundAction.add[i])){
		result.flipOn(groundAction.add[i]);
		//result.propositionsChanged.push_back(groundAction.add[i]);
	    }
	}
    
	return result;
    }
    
    
    template<typename GroundActionType>
    void State<GroundActionType>::randomize()
    {
	register ELEM_TYPE tmp;
	register uint i;
	register unsigned short int j;
	
	for( i = 0; i < numPropositions/*data.size()*/; i++){
	    for(j = 0 ; j < sizeof(ELEM_TYPE) * 8; j++){
		tmp = 1;
		tmp = tmp<<j;
		if(random() % 2){
		    tmp = tmp^big;
		    data[i] &= tmp;
		} else {
		    data[i] |= tmp;
		}
	    }
	}
    }

    /*Change individual (at index \argument{uint}) bits of the
     * state representation (\member{data}).*/
    template<typename GroundActionType>
    void State<GroundActionType>::flipOn(uint index)
    {
	uint remainder = REMAINDER(index);
	uint bitVecNum = FLOOR(index);

	ELEM_TYPE tmp = 1;
	tmp = tmp<<remainder;
	data[bitVecNum] |= tmp;
    
	/*Now it's true, some actions may be executable.*/
    }

    template<typename GroundActionType>
    void State<GroundActionType>::flipOff(uint index)
    {
	uint remainder = REMAINDER(index);
	uint bitVecNum = FLOOR(index);

	ELEM_TYPE tmp = 1;
	tmp = tmp<<remainder;
	tmp = tmp^big;
	data[bitVecNum] &= tmp;
    }

    template<typename GroundActionType>
    void State<GroundActionType>::flip(uint index)
    {
	uint remainder = REMAINDER(index);
	uint bitVecNum = FLOOR(index);

	ELEM_TYPE tmp = 1;
	tmp = tmp<<remainder;
    
	if(data[bitVecNum] & tmp){
	    tmp = tmp^big;
	    data[bitVecNum] &= tmp;
	} else {
	    data[bitVecNum] |= tmp;

	    /*Now it's true, some actions may be executable.*/
	}
    }

    template<typename GroundActionType>
    bool State<GroundActionType>::isTrue(uint index) const
    {
	uint remainder = REMAINDER(index);
	uint bitVecNum = FLOOR(index);

	ELEM_TYPE tmp = 1;
	tmp = tmp<<remainder;
    
	return (data[bitVecNum] & tmp)?true:false;
    }



    /*Are we in the goal state.*/
    template<typename GroundActionType>
    bool State<GroundActionType>::isGoal_positive(const vector<ELEM_TYPE>& goalPos) const
    {
	assert(goalPos.size() == data.size());
    
	for(uint i = 0; i < goalPos.size(); i++){
	    /*Is this part of the goal useful?*/
	    if(goalPos[i]){
		VERBOSER(15, "Testing positive goal condition.\n");
		/*Is the right stuff true.*/
		if((data[i] & goalPos[i]) != goalPos[i]){
		    VERBOSER(15, "Failed a positive goal condition.\n");
		    return false;
		} else {
		    VERBOSER(15, "Passed 1 positive goal condition.\n");
		}   
	    }
	}

	VERBOSER(14 , "We meet the positive part of the goal condition."<<endl);
    
	return true;
    }

    template<typename GroundActionType>
    bool State<GroundActionType>::isGoal_negative(const vector<ELEM_TYPE>& goalNeg) const
    {
	assert(goalNeg.size() == data.size());
    
	for(uint i = 0; i < goalNeg.size(); i++){
	    /*Is this part of the goal useful?*/
	    if(goalNeg[i]){

		VERBOSER(14, "There is a negative component to the goal specification.");
	    
		/*Is the right stuff true.*/
		if(data[i] & goalNeg[i]){
		    return false;
		}
	    }
	}

	VERBOSER(14 , "We meet the negative part of the goal condition."<<endl);
    
	return true;
    }



    template<typename GroundActionType>
    State<GroundActionType>::State(const State<GroundActionType>& state)
	:actions(state.actions),
	 numPropositions(state.numPropositions),
	 data(state.data),
	 time(state.time),
	 parent(0),
	 isGoalState(state.isGoalState),
         compressed(false)
    {
    }

    template<typename GroundActionType>
    size_t State<GroundActionType>::hash_value() const
    {
	VERBOSER(12, "State giving a hash value of :: "
		 <<boost::hash_range(data.begin(), data.end())<<endl);
	
	return boost::hash_range(data.begin(), data.end());
    }
    
    template<typename GroundActionType>
    State<GroundActionType>& State<GroundActionType>::operator=(const State& state)
    {
	this->actions = state.actions;
	this->numPropositions = state.numPropositions;
	this->data = state.data;
	this->time = state.time;

	parent = 0;
    }

    template<typename GroundActionType>
    bool State<GroundActionType>::operator<(const State<GroundActionType>& state) const
    {
	return (state.data.size() < this->data.size())?true:
	    ((state.data.size() == this->data.size())?(state.data < this->data):false);
    }
 
    template<typename GroundActionType>
    bool State<GroundActionType>::operator==(const State<GroundActionType>& state) const
    {
	return state.data == this->data;
    }
 
    template<typename GroundActionType>
    State<GroundActionType>::State(uint size)
	:numPropositions(size),
	 time(0),
	 parent(0),
	 isGoalState(false),
         compressed(false)
    {
	data = vector<ELEM_TYPE>(CEIL(size));
    
    
	for(uint i = 0; i < data.size(); i++){
	    data[i] = 0;
	}

    
	for(uint i = numPropositions + 1; i < SIZE_ELEM * data.size(); i++){
	    flipOn(i);
	}
    
	//     for(uint i = SIZE_ELEM * data.size(); i > numPropositions; i--){
	// 	flipOn(i);
	//     }

#ifdef DEBUG
	for(uint i = 0 ; i < numPropositions; i++){
	    assert(isFalse(i));
	}
#endif
    }


    
    /*Function for STL and boost to access \member{hash_value} of
     * \argument{GroundAction}.*/
    template<typename GroundActionType>
    std::size_t hash_value(const State<GroundActionType>& state)
    {
	return state.hash_value();
    }


    template<typename GroundActionType>
    ostream& operator<<(ostream& o, const State<GroundActionType>& state)
    {

	/*Print out the fluent propositions first.*/
	for(int i = 0; i < state.getNumPropositions(); i++){
	    if(state.isTrue(i)){
		
		assert(i < State<GroundActionType>::fluentPropositions->size());
		
		o<<(*State<GroundActionType>::fluentPropositions)[i];
	    }
	}


	/*Print out the invariant propositions next*/
	for(MapStringToPairInt::const_iterator propositionIndex =  State<GroundActionType>::propositionIndices->begin()
		; propositionIndex != State<GroundActionType>::propositionIndices->end()
		; propositionIndex++){
	    // 	    if(State<GroundActionType>::problem->domain
	    // 	       .isConstant((*State<GroundActionType>::propositions)[propositionIndex->second.first])){
	    
	    if(State<GroundActionType>::problem->domain
	       .isConstant(propositionIndex->first)){//(*State<GroundActionType>::propositions)[propositionIndex->second.first])){

		for(uint index = propositionIndex->second.first
			; index < propositionIndex->second.second
			; index++){
		    o<<(*State<GroundActionType>::propositions)[index];
		}
	    }
	}
	

	return o;
    }
}

#endif
