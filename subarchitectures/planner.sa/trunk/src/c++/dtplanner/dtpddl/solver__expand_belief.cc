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

#ifndef SOLVER__EXPAND_BELIEF_HH
#define SOLVER__EXPAND_BELIEF_HH


#include "solver.hh"

#include "markov_decision_process_state.hh"

using namespace Planning;


void Solver::expand_belief_state(POMDP_State* pomdp_state)
{
    auto belief_state = pomdp_state->get__belief_state();
    
    /* Expand each individual member of the belief state that is not
     * already expanded. */
    for(auto atom = belief_state.begin(); atom != belief_state.end(); atom++){
        auto state = atom->first;
        auto probability = atom->second;

        POMDP_State::Searchable_Belief_State searchable_Belief_State;
        
        if(!state->get__has_been_expanded()){
            assert(dynamic_cast<const State*>(state));
            expand_optional_transformations(dynamic_cast<State*>(state));
            
            auto successors = state->get__successors();
            auto successor_Probabilities = state->get__successor_Probabilities();
            
            assert(successors.size() == successor_Probabilities.size());
            
//             for(auto index = 0
//                     ; index != successors.size()
//                     ; index++){
//                 auto successor_state = successors.at(index);
//                 auto successor_probability = successor_Probabilities.at(index);

//                 if(are_Doubles_Close(0.0, successor_probability)) continue;
                
//                 if(searchable_Belief_State.find(successor_state) == searchable_Belief_State.end()){
//                     searchable_Belief_State[successor_state] = 0.0;    
//                 }
                
//                 searchable_Belief_State[successor_state] += successor_probability;
//             }
        }
    }
}

bool Solver::expand_belief_state_space()
{
    if(!expansion_queue.size()){
        return false;
    }
    
    auto pomdp_state = expansion_queue.front();
    expansion_queue.pop();

    expand_belief_state(pomdp_state);
}


#endif
