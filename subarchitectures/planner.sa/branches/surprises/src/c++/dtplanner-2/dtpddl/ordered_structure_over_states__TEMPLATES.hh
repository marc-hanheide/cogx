/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * CogX ::
 *
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 *
 */


#ifndef ORDERED_STRUCTURE_OVER_STATES__TEMPLATES_HH
#define ORDERED_STRUCTURE_OVER_STATES__TEMPLATES_HH

#if 0
#define LOCAL_ASSERTION(state__To__Index, value__To__States)    { \
	for(auto p = value__To__States.begin() \
		; p != value__To__States.end() \
		; p++){ \
	    for(auto q = p->second.begin() \
		     ; q != p->second.end() \
		     ; q++){ \
		assert(state__To__Index.find(*q) != state__To__Index.end());	\
	    } \
	} \
    }
#else
#define LOCAL_ASSERTION(state__To__Index, value__To__States) {};
#endif

namespace Planning
{
    
    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::Ordered_Stack_Of_States()
	:count_of_contents(0)
    {
 

	LOCAL_ASSERTION(state__To__Index, value__To__States);
	VERBOSER(23, "MAKE :: Made stack of unexpanded states :: "<<size()<<std::endl);
	LOCAL_ASSERTION(state__To__Index, value__To__States);
    }


    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    bool Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::contains(State_Type* state) const
    {
	if(state__To__Index.find(state) != state__To__Index.end()) {
	    return true;
	} else {
	    return false;
	} 
    }
    
    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    void Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::remove(State_Type* state)
    {
	/*If the state that we remove is not unexpanded anyway.*/
	if(state__To__Index.find(state) == state__To__Index.end()) {
	    //WARNING("REMOVE :: Ignoring state as it is not a candidate for expansion. "<<*state<<std::endl);
	    return;
	    // 	exit(0);
	    // 	return;
	}
    
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	auto state_value = state_evaluator(state);
    
	if(value__To__States.find(state_value) == value__To__States.end()) {
	    UNRECOVERABLE_ERROR("REMOVE :: Failed no value__To__States with state_value "<<state_value<<std::endl);
	}
    
	uint index = state__To__Index[state];
	auto& list_of_states = value__To__States[state_value];

	assert(index < value__To__States[state_value].size());
    
	VERBOSER(23, "REMOVE :: Removing state "<<*list_of_states[index]<<std::endl);

	assert(list_of_states.size() > 0);
    
	if(list_of_states.size() == 1){
	    value__To__States.erase(state_value);//.clear();
	    state__To__Index.erase(state);
	
	    assert(value__To__States.find(state_value) == value__To__States.end());
	    assert(state__To__Index.find(state) == state__To__Index.end());
	
	    LOCAL_ASSERTION(state__To__Index, value__To__States);
	} else {
	    /*Deleting a state at the end of a list.*/
	    if(index == ( list_of_states.size() - 1 ) ){//list_of_states[index] == list_of_states[list_of_states.size() - 1]){
		list_of_states.resize(list_of_states.size() - 1);
		state__To__Index.erase(state);
	    
		assert(state__To__Index.find(state) == state__To__Index.end());
		LOCAL_ASSERTION(state__To__Index, value__To__States);
	    } else {

		assert(state != list_of_states[list_of_states.size() - 1]);
		assert(state__To__Index.find(list_of_states[list_of_states.size() - 1]) != state__To__Index.end());
	    
		list_of_states[index] = list_of_states[list_of_states.size() - 1];
		list_of_states.resize(list_of_states.size() - 1);
	    
		assert(state__To__Index.find(state) != state__To__Index.end());
		state__To__Index.erase(state);/*If this state occurs twice, then perhaps we are in trouble.*/
		assert(state__To__Index.find(state) == state__To__Index.end());
	    
		assert(state__To__Index.find(list_of_states[index]) != state__To__Index.end());
		state__To__Index[list_of_states[index]] = index;
		assert(state__To__Index.find(list_of_states[index]) != state__To__Index.end());
	    
		LOCAL_ASSERTION(state__To__Index, value__To__States);
	    }
	
	}
    
	assert(count_of_contents > 0);
	VERBOSER(23, "REMOVE :: Stack of unexpanded states has size "<<(count_of_contents - 1)<<std::endl);
	count_of_contents--;

    }


    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    void Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::push_back(State_Type* state)
    {
	LOCAL_ASSERTION(state__To__Index, value__To__States);
        
	auto state_value = state_evaluator(state);

	/*Do not allow a state to be put on this list twice.*/
	assert(state__To__Index.find(state) == state__To__Index.end());
    
    
	VERBOSER(23, "PUSH_BACK :: Give state with state_value :: "<<state_value<<std::endl);
    
	if(value__To__States.find(state_value) == value__To__States.end()){
	    VERBOSER(23, "Making new stack of states with state_value :: "<<state_value<<std::endl);
            
	    value__To__States[state_value] = std::vector<State_Type*>();
	}
    
	VERBOSER(23, "PUSH_BACK :: Pushing back state at index :: "<<value__To__States[state_value].size()
		 <<" for state_value :: "<<state_value<<std::endl);
    
	state__To__Index[state] = value__To__States[state_value].size();
	value__To__States[state_value].push_back(state);

	assert(state__To__Index.find(state) != state__To__Index.end());
    
	VERBOSER(23, "PUSH_BACK :: Stack of unexpanded states has size "<<(count_of_contents + 1)<<std::endl);
	count_of_contents++;
	LOCAL_ASSERTION(state__To__Index, value__To__States);
    }

    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    bool Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::empty() const
    {
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	assert(count_of_contents != 0 || state__To__Index.size() == 0);
    
	VERBOSER(23, "EMPTY :: Reporting stack emptiness as :: "<<(state__To__Index.size() == 0)<<std::endl);
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	return (count_of_contents == 0);//(state__To__Index.size() == 0);
    }

    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    State_Type* Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::top()
    {
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	if(empty()) return 0;

        
	VERBOSER(23, "POP :: count_of_contents is "<<count_of_contents<<std::endl);
	assert(count_of_contents > 0);

        auto& most_promising_states_list = value__To__States.rbegin()->second;

        /* Best-value elements should always be contained in
         * \local{most_promising_states_list}, otherwise the pop
         * should have removed it from the map.*/
        assert(most_promising_states_list.size());
        
        /*FIFO*/
        return most_promising_states_list[0];
    }
    
    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    State_Type* Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::pop()
    {
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	if(empty()) return 0;

	VERBOSER(23, "POP :: count_of_contents is "<<count_of_contents<<std::endl);
	assert(count_of_contents > 0);

        auto& most_promising_states_list = value__To__States.rbegin()->second;
        
	auto most_promising_states_value = value__To__States.rbegin()->first;

	INTERACTIVE_VERBOSER(true, 10800, "POP :: Choosing most_promising_state_value :: "
                             <<most_promising_states_value<<std::endl);

	assert(most_promising_states_list.size() > 0);

#ifndef NDEBUG
        for(auto element = most_promising_states_list.begin()
                ; element != most_promising_states_list.end()
                ; element++){
            if(state_evaluator(*element) != most_promising_states_value){
//                 assert(static_cast<float>(state_evaluator(*element)) == static_cast<float>(most_promising_states_value));
                std::cerr<<std::scientific<<state_evaluator(*element)<<" "<<most_promising_states_value<<std::endl;
            }
            
//             assert(state_evaluator(*element) == most_promising_states_value);
        }
        
#endif
        
        /*FIFO*/
	uint index = 0;//random() % most_promising_states_list.size();

	assert(index < most_promising_states_list.size());
    
	State_Type* result = most_promising_states_list[index];
    
	assert(result);

//         std::set<double> things;
//         while(true){
//             things.insert(state_evaluator(result));

//             assert(*things.begin() == state_evaluator(result));
//             if(things.size() != 1) exit(0);
//         }
        
        QUERY_UNRECOVERABLE_ERROR(state_evaluator(result) != most_promising_states_value,
                                  "Most promising value is :: "<<most_promising_states_value<<std::endl
                                  <<"Yet we get a promising state value of :: "<<state_evaluator(result)<<std::endl);
        
// 	assert(state_evaluator(result) == most_promising_states_value);
        
	if(state__To__Index.find(result) == state__To__Index.end()){
	    VERBOSER(23, "State :: "<<*result<<std::endl
		     <<"at index :: "<<index<<std::endl
		     <<"has no index..."<<std::endl);
	}
    
	assert(state__To__Index.find(result) != state__To__Index.end());

	VERBOSER(23, "POP :: Giving up state :: "<<*result<<std::endl);
    
	remove(result);

	VERBOSER(23, "POP :: Giving up state :: "<<*result<<std::endl);
    
	LOCAL_ASSERTION(state__To__Index, value__To__States);
	return result;
    }


    template<typename State_Type, typename Evaluator_Type, typename Value_Type>
    uint Ordered_Stack_Of_States<State_Type, Evaluator_Type, Value_Type>::size() const
    {
	return count_of_contents;
    }
}


#endif
