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


#ifndef ORDERED_STRUCTURE_OVER_STATES_HH
#define ORDERED_STRUCTURE_OVER_STATES_HH

#include"global.hh"

namespace Planning
{
    template<typename State_Type, typename Evaluator_Type, typename Value_Type = double>
    class Ordered_Stack_Of_States
    {
    public:
        Ordered_Stack_Of_States();
        
	/* Remove the \argument{state} from this structure.*/
	void remove(State_Type* state);

	/* TRUE if the \argument{state} in this stack?*/
	bool contains(State_Type* state) const;
	
	/* Are there no states in this structure?*/
	bool empty() const;
	
	/* Add a \argument{state} to this.*/
	void push_back(State_Type* state);

	/* Pick a state to expand.*/
	State_Type* pop();
        
	/* State that will be picked for expansion on the next call to \member{pop}.*/
	State_Type* top();

	/* How many elements are in the unexpanded stack?*/
	uint size() const;// {return count;};
    private:
 	typedef std::tr1::unordered_map<State_Type*, Value_Type, boost::hash<State_Type*> > State__To__Index;

 	typedef std::map<Value_Type, std::vector<State_Type*> > Value__To__States;
        
	/* How many elements are in the unexpanded stack?*/
	uint count_of_contents;
	
	/* Index of a state */
        State__To__Index state__To__Index;

	/* Each stack of states has a cost that corresponds to the
	 * domain element that maps to it.*/
        Value__To__States value__To__States;

        /*Computes the heuristic value of a state.*/
        Evaluator_Type state_evaluator;
    };
}

#include "ordered_structure_over_states__TEMPLATES.hh"

#endif
