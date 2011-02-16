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

#include "simple_online_solver.hh"

#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning;

Simple_Online_Solver::Simple_Online_Solver(Planning::Parsing::Problem_Data& in)
    :Planning::Solver(in)
{
    
}


Greedy_Heuristic some_Greedy_Heuristic;

void Simple_Online_Solver::report__new_belief_state(POMDP_State* state)
{
#ifndef NDEBUG
    decltype(ordered_Stack_Of_States) _ordered_Stack_Of_States = this->ordered_Stack_Of_States;
    while(_ordered_Stack_Of_States.size()){
        auto thing = _ordered_Stack_Of_States.pop();

        
        double answer = 0.0;
        double answer2 = 0.0;
        auto& belief_State = thing->get__belief_state(); 
        for(auto _mdp_state = belief_State.begin()
                ; _mdp_state != belief_State.end()
                ; _mdp_state++){
            auto mdp_state = _mdp_state->first;
            auto probability = _mdp_state->second;
            assert(dynamic_cast<Planning::State*>(mdp_state));

            answer += probability * static_cast<double>(
                dynamic_cast<Planning::State*>(mdp_state)
                ->get__obtainable_rewards_value());
            
            answer2 += probability * static_cast<double>(
                dynamic_cast<Planning::State*>(mdp_state)
                ->get__obtainable_rewards_count());
        }
    
//         some_Greedy_Heuristic(thing);
//         cerr<<some_Greedy_Heuristic(thing)<<" "<<thing<<" "<<answer<<" "<<answer2<<" "<<thing->get__expected_reward()<<std::endl;
        
    }
//      cerr<<std::endl;
//      {char ch; cin>>ch;}
#endif
    
    ordered_Stack_Of_States.push_back(state);
}

POMDP_State* Simple_Online_Solver::obtain__next_belief_state_for_expansion()
{
    if(!ordered_Stack_Of_States.size()) return 0;
    
    
    return ordered_Stack_Of_States.pop();
}

POMDP_State* Simple_Online_Solver::peek__next_belief_state_for_expansion()
{
    if(!ordered_Stack_Of_States.size()) return 0;
    
    return ordered_Stack_Of_States.top();
}
