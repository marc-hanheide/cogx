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

#include "action__state_transformation.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

using namespace Planning;

void Solver::press__belief_transitions(POMDP_State* pomdp_state,
                                       POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state )
{
    for(auto action__to__Observation_to_Belief = _successor_belief_state.begin()
            ; action__to__Observation_to_Belief != _successor_belief_state.end()
            ; action__to__Observation_to_Belief++){
        auto action_index = action__to__Observation_to_Belief->first;
        for(auto observation_to_Belief = action__to__Observation_to_Belief->second.begin()
                ; observation_to_Belief != action__to__Observation_to_Belief->second.end()
                ; observation_to_Belief++){
            auto observation = observation_to_Belief->first;

            POMDP_State* successor_pomdp_state = new POMDP_State();

            double testing_rational_belief_measure = 0.0;
            for(auto atom = observation_to_Belief->second.begin()
                    ; atom != observation_to_Belief->second.end()
                    ; atom++){
                INTERACTIVE_VERBOSER(true, 9082, "Adding to belief state :: "
                                     <<*(atom->first)<<std::endl
                                     <<atom->second<<std::endl);
                testing_rational_belief_measure += atom->second;
                successor_pomdp_state->add__belief_atom(atom->first, atom->second);
            }

            QUERY_UNRECOVERABLE_ERROR(!are_Doubles_Close(1.0, testing_rational_belief_measure),
                                      "Got a belief state where the sum of atom probabilities sums to :: "
                                      <<testing_rational_belief_measure<<"\n that should be 1.0...");
            
            auto belief_state__index = belief_state__space.find(successor_pomdp_state);
            if(belief_state__index == belief_state__space.end()){
                
                INTERACTIVE_VERBOSER(true, 9083, "New successor POMDP state :: "
                                     <<*successor_pomdp_state<<std::endl);
                
                belief_state__space.insert(successor_pomdp_state);
            } else {
                delete successor_pomdp_state;
                successor_pomdp_state = *belief_state__index;
            }

            pomdp_state->push__successor(action_index, observation, successor_pomdp_state);
        }
    }
}


void Solver::add_entry(POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state
                       , uint action_index
                       , Planning::Observational_State* observation
                       , MDP_State* successor_state
                       , double probability)
{
    auto action__to__Observation_to_Belief__index = _successor_belief_state.find(action_index);
    if(action__to__Observation_to_Belief__index == _successor_belief_state.end()){
        
        INTERACTIVE_VERBOSER(true, 9083, "First time  :: "<<action_index<<std::endl);
        
        _successor_belief_state[action_index] = POMDP_State::Observation_to_Belief();
        action__to__Observation_to_Belief__index = _successor_belief_state.find(action_index);
    }

    auto& observation_to_Belief = action__to__Observation_to_Belief__index->second;
    auto observation_to_Belief__index = observation_to_Belief.find(observation);
    if(observation_to_Belief__index == observation_to_Belief.end()){
        
        INTERACTIVE_VERBOSER(true, 9083, "First time  :: "<<*observation<<std::endl);
        
        
        observation_to_Belief[observation] = POMDP_State::Searchable_Belief_State();
        observation_to_Belief__index = observation_to_Belief.find(observation);
    }

    auto& searchable_Belief_State = observation_to_Belief__index->second; 
    
    auto successor_state__index = searchable_Belief_State.find(successor_state);
    if(successor_state__index == searchable_Belief_State.end()){
        
        INTERACTIVE_VERBOSER(true, 9083, "First time  :: "<<*successor_state<<std::endl);
        
        
        searchable_Belief_State[successor_state] = probability;
    } else {
        searchable_Belief_State[successor_state] = successor_state__index->second + probability; 
    }
}

void Solver::expand_belief_state(POMDP_State* pomdp_state)
{   
    auto belief_state = pomdp_state->get__belief_state();
    
    POMDP_State::Action__to__Observation_to_Belief _successor_belief_state ;
        
    /* For each mdp-state (i.e., atom) in the belief.*/
    for(auto atom = belief_state.begin()
            ; atom != belief_state.end()
            ; atom++){
        
        auto state = atom->first;
        auto state_probability = atom->second;

        
        INTERACTIVE_VERBOSER(true, 9081, "Looking at successors of  :: "
                             <<*state<<std::endl
                             <<"That occurs in a belief with probability :: "<<state_probability<<std::endl);
        
        /* Expand each individual member of the belief state that is not
         * already expanded. */
        if(!state->get__has_been_expanded()){
            assert(dynamic_cast<const State*>(state));
            
            INTERACTIVE_VERBOSER(true, 9081, "START Expanding MDP-state  :: "
                                 <<*state<<std::endl);
            
            expand_optional_transformations(dynamic_cast<State*>(state));
            INTERACTIVE_VERBOSER(true, 9081, "DONE Expanding MDP-state  :: "
                                 <<*state<<std::endl);
        }
        
        auto successors = state->get__successors();
        auto successor_Probabilities = state->get__successor_Probabilities();
        auto successor_Drivers = state->get__successor_Driver();
        
        assert(successors.size() == successor_Probabilities.size());
        assert(successors.size() == successor_Drivers.size());
        
        /* For each action that can be executed at \local{state}.*/
        for(auto driver_index = 0
                ; driver_index < successors.size()
                ; driver_index++){
            /* Action that generates the successor.*/
            auto action_index = successor_Drivers[driver_index];
            /* States that can result from executing that action at \local{state}.*/
            auto& successor_states = successors[driver_index];
            /* Probability that a particular \local{successor_states} results after
             * executing action \local{action_index}  at \local{state}.*/
            auto& outcome_probabilities = successor_Probabilities[driver_index];

            assert(outcome_probabilities.size() == successor_states.size());

            /* For each state that can result from exeucting \local{action_index} at \local{state}.*/
            for(auto state_index =  0
                    ; state_index <  successor_states.size()
                    ; state_index++){
                
                auto successor_state = successor_states[state_index];
                auto successor_state__probability = outcome_probabilities[state_index];


                
                
                
//                 INTERACTIVE_VERBOSER(true, 9081, "Successor from :: "
//                                      <<*state<<std::endl
//                                      <<symbol<<std::endl
//                                      <<*successor_state<<std::endl);
                
                auto& _observations = successor_state->get__observations();
                auto& _observation_probabilities = successor_state->get__observation_Probabilities();
//                 auto& observation_Drivers = successor_state->get__observation_Driver();
                
                assert(successor_state->action_to_observation__includes_index(action_index));
                auto observation_index = successor_state->get__action_to_observation__index(action_index);
                assert(observation_index < _observations.size());
                assert(_observation_probabilities.size() == _observations.size());
                
//                 for(auto observation_Driver = observation_Drivers.begin()
//                         ; observation_Driver != observation_Drivers.end()
//                         ; observation_Driver++){
//                     INTERACTIVE_VERBOSER(true, 9081, "Action for observation :: "<<*observation_Driver<<std::endl
//                                          <<action_index<<std::endl);
//                 }
//                 exit(0);
                
//                 assert(observation_Drivers[driver_index] == action_index);
                
                auto& observation_probabilities = _observation_probabilities[observation_index];
                auto& observations = _observations[observation_index];
                
                assert(observation_probabilities.size() == observations.size());
                
                for(auto observation_index = 0
                        ; observation_index < observations.size()
                        ; observation_index++){
                    
                    auto observation = observations[observation_index];
                    auto observation_probability = observation_probabilities[observation_index];
                    
                    assert(State_Transformation::
                           ith_exists(reinterpret_cast<ID_TYPE>(problem_Grounding.get()), action_index));
                    auto symbol = State_Transformation::
                        make_ith<State_Transformation>
                        (reinterpret_cast<ID_TYPE>(problem_Grounding.get()),
                         action_index);
                    
                    INTERACTIVE_VERBOSER(true, 9085, "Starting at state with probability :: "<<state_probability<<std::endl
                                         <<"Got a successor  :: "
                                         <<*successor_state<<std::endl
                                         <<"Occurring with probability :: "<<successor_state__probability<<std::endl
                                         <<"Under action :: "<<symbol.get__identifier()<<std::endl
                                         <<"Then :: "<<*observation<<std::endl
                                         <<"With probability :: "<<observation_probability<<std::endl
                                         <<"For a total probability of :: "
                                         <<(state_probability * successor_state__probability * observation_probability));
                    
//                     INTERACTIVE_VERBOSER(true, 9083, "Adding entry to POMDP successors :: "
//                                          <<*observation<<std::endl
//                                          << (state_probability * successor_state__probability * observation_probability)
//                                          << " " <<action_index<<std::endl);
                    
                    add_entry(_successor_belief_state
                              , action_index
                              , observation
                              , successor_state
                              , (state_probability * successor_state__probability * observation_probability));
                }
            }
        }
    }
    
    press__belief_transitions(pomdp_state, _successor_belief_state );
}

bool Solver::expand_belief_state_space()
{
    if(!expansion_queue.size()){
        return false;
    }
    
    auto pomdp_state = expansion_queue.front();
    expansion_queue.pop();

    INTERACTIVE_VERBOSER(true, 9083, "Expanding POMDP state :: "<<*pomdp_state<<std::endl);
    
    expand_belief_state(pomdp_state);
    
    INTERACTIVE_VERBOSER(true, 9083, "Expanded POMDP state :: "<<*pomdp_state<<std::endl);
}


#endif
