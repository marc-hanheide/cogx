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

#include "problem_grounding.hh"

#include <cmath>

using namespace Planning;

void Solver::press__belief_transitions(POMDP_State* pomdp_state,
                                       const POMDP_State::Normalisation_Factors& _normalisation_Factors,
                                       const POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state )
{
    INTERACTIVE_VERBOSER(true, 9094, "Adding the following transition information  :: "
                         <<_successor_belief_state<<std::endl);
    
    for(auto action__to__Observation_to_Belief = _successor_belief_state.begin()
            ; action__to__Observation_to_Belief != _successor_belief_state.end()
            ; action__to__Observation_to_Belief++){
        uint action_index = action__to__Observation_to_Belief->first;
        for(auto observation_to_Belief = action__to__Observation_to_Belief->second.begin()
                ; observation_to_Belief != action__to__Observation_to_Belief->second.end()
                ; observation_to_Belief++){
            Observational_State* observation = observation_to_Belief->first;

            POMDP_State* successor_pomdp_state = new POMDP_State();

            INTERACTIVE_VERBOSER(true, 9086, "Observation was :: "
                                 <<*observation<<std::endl);

//             auto t1 =  _normalisation_Factors.find(action_index);
//             auto t2 =  t1->second.find(observation);
//             auto t3 =  t2->second;
            assert(_normalisation_Factors.find(action_index) != _normalisation_Factors.end());
            assert(_normalisation_Factors.find(action_index)->second.find(observation) !=
                   _normalisation_Factors.find(action_index)->second.end());
            
            double mass_of_belief_successor = _normalisation_Factors.find(action_index)->second.find(observation)->second;


            double testing_rational_belief_measure = 0.0;
            for(auto atom = observation_to_Belief->second.begin()
                    ; atom != observation_to_Belief->second.end()
                    ; atom++){
                double atom_mass = atom->second / mass_of_belief_successor;
                
                INTERACTIVE_VERBOSER(true, 9095, *observation<<std::endl
                                     <<"Adding to belief state :: "
                                     <<*(atom->first)<<std::endl
                                     <<atom->second<<std::endl
                                     <<"Normalised :: "<<atom_mass<<std::endl
                                     <<"Via constant :: "<<mass_of_belief_successor<<std::endl);
                testing_rational_belief_measure += atom_mass;
                
//                 if(are_Doubles_Close(1.0, atom_mass)){
//                     atom_mass = 1.0;
//                 } else if (are_Doubles_Close(0.0, atom_mass)) {
//                     atom_mass = 0.0;
//                 }
                
                successor_pomdp_state->add__belief_atom(atom->first, atom_mass);
            }

            QUERY_UNRECOVERABLE_ERROR(!are_Doubles_Close(1.0, testing_rational_belief_measure),
                                      "Got a belief state where the sum of atom probabilities sums to :: "
                                      <<testing_rational_belief_measure<<"\n that should be 1.0...");
            
            
            assert(State_Transformation::
                   ith_exists(reinterpret_cast<ID_TYPE>(problem_Grounding.get()), action_index));
            auto symbol = State_Transformation::
                make_ith<State_Transformation>
                (reinterpret_cast<ID_TYPE>(problem_Grounding.get()),
                 action_index);
//             INTERACTIVE_VERBOSER(true, 9095, "For action :: "<<symbol<<std::endl
//                                  <<"For Observation :: "<<observation<<std::endl
//                                  <<"Got a successor belief state :: "<<*successor_pomdp_state);
            
            auto belief_state__index = belief_state__space.find(successor_pomdp_state);
            if(belief_state__index == belief_state__space.end()){
                
                INTERACTIVE_VERBOSER(true, 9095, "New successor POMDP state :: "
                                     <<*successor_pomdp_state<<std::endl);

                successor_pomdp_state->set__index(belief_state__space.size());
                successor_pomdp_state->initialise__prescribed_action_index();
                belief_state__space.insert(successor_pomdp_state);
                
                expansion_queue.push(successor_pomdp_state);
            } else {
                INTERACTIVE_VERBOSER(true, 9096, "Repeated successor POMDP state :: "
                                     <<*successor_pomdp_state<<std::endl);
                delete successor_pomdp_state;
                successor_pomdp_state = *belief_state__index;
            }
            
            INTERACTIVE_VERBOSER(true, 9090, "For action :: "<<symbol<<std::endl
                                 <<"For Observation :: "<<*observation<<std::endl
                                 <<"Got a successor belief state :: "<<*successor_pomdp_state);
//             INTERACTIVE_VERBOSER(true, 9090, "Adding successor state :: "<<*successor_pomdp_state<<std::endl);
            
            pomdp_state->push__successor(action_index, observation, successor_pomdp_state, mass_of_belief_successor);
        }
    }
}

void Solver::fill_illegals(POMDP_State::Normalisation_Factors& _normalisation_Factors
                           , POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state
                           , uint action_index
                           , MDP_State* successor_state
                           , double probability)
{
    if(!null_observation){
        INTERACTIVE_VERBOSER(true, 9094, " ** Making NULL observation for the first and last time. ** ");
        Observational_State* new_observation
            = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());
        
        auto observation__space_iterator = observation__space.find(new_observation);
        assert(observation__space_iterator != observation__space.end());
        null_observation = *observation__space_iterator;
    }
    
    
    assert(_normalisation_Factors.find(action_index) != _normalisation_Factors.end());

    auto& observation__to__factor = _normalisation_Factors.find(action_index)->second;
    auto observation__to__factor__iterator = observation__to__factor.find(null_observation);
    if(observation__to__factor__iterator == observation__to__factor.end()){
        observation__to__factor[null_observation] = 0.0;
    }
    observation__to__factor[null_observation] += probability;
    
    
    INTERACTIVE_VERBOSER(true, 9094, " ** Adding belief ** "
                         <<"Got a successor  :: "
                         <<*successor_state<<std::endl
                         <<"Under action :: "<<action_index<<std::endl
                         <<"Then :: "<<null_observation<<std::endl
                         <<"With total probability :: "<<probability<<std::endl);
    
    assert(_successor_belief_state.find(action_index) != _successor_belief_state.end());
    auto action__to__Observation_to_Belief__index = _successor_belief_state.find(action_index);
    auto& observation_to_Belief = action__to__Observation_to_Belief__index->second;    
    auto observation_to_Belief__index = observation_to_Belief.find(null_observation);
    if(observation_to_Belief__index == observation_to_Belief.end()){
        
        INTERACTIVE_VERBOSER(true, 9083, "First time  :: "<<*null_observation<<std::endl);
        
        
        observation_to_Belief[null_observation] = POMDP_State::Searchable_Belief_State();
        observation_to_Belief__index = observation_to_Belief.find(null_observation);
    }

    INTERACTIVE_VERBOSER(true, 9087, "Tracking :: "<<observation_to_Belief.size()
                         <<" observations on :: "<<action_index<<std::endl);
    
    auto& searchable_Belief_State = observation_to_Belief__index->second;
    
    auto successor_state__index = searchable_Belief_State.find(successor_state);
    if(successor_state__index == searchable_Belief_State.end()){
        
        INTERACTIVE_VERBOSER(true, 9087, "1st. Reaching  :: "<<*successor_state
                             <<"p := "<<probability<<std::endl
                             <<"new state is :: "<<successor_state<<std::endl
                             <<"new state is :: "<<*successor_state<<std::endl);
        
        
        searchable_Belief_State[successor_state] = probability;
    } else {
        searchable_Belief_State[successor_state] += /*successor_state__index->second +*/ probability;
        
        INTERACTIVE_VERBOSER(true, 9087, "nth. Reaching  :: "<<*successor_state
                             <<"p := "<<searchable_Belief_State[successor_state]<<std::endl);
    }
}

void Solver::add_entry(POMDP_State::Normalisation_Factors& _normalisation_Factors
                       , POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state
                       , uint action_index
                       , Planning::Observational_State* observation
                       , MDP_State* successor_state
                       , double probability)
{
    
    auto _normalisation_Factors__iterator = _normalisation_Factors.find(action_index);
    if(_normalisation_Factors__iterator == _normalisation_Factors.end()){
        _normalisation_Factors[action_index] = std::map<Planning::Observational_State*, double>();
        _normalisation_Factors__iterator = _normalisation_Factors.find(action_index);
    }
    auto& observation__to__factor = _normalisation_Factors__iterator->second;
    auto observation__to__factor__iterator = observation__to__factor.find(observation);
    if(observation__to__factor__iterator == observation__to__factor.end()){
        observation__to__factor[observation] = 0.0;
    }
    observation__to__factor[observation] += probability;
    
    
    INTERACTIVE_VERBOSER(true, 9084, " ** Adding belief ** "
                         <<"Got a successor  :: "
                         <<*successor_state<<std::endl
                         <<"Under action :: "<<action_index<<std::endl
                         <<"Then :: "<<*observation<<std::endl
                         <<"With total probability :: "<<probability<<std::endl);
    INTERACTIVE_VERBOSER(true, 9085, " ** Adding weight ** "
                         <<probability<<std::endl
                         <<"To :: "<<*observation<<std::endl
                         <<"Under action :: "<<action_index<<std::endl);
    
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

    INTERACTIVE_VERBOSER(true, 9087, "Tracking :: "<<observation_to_Belief.size()
                         <<" observations on :: "<<action_index<<std::endl);
    
    auto& searchable_Belief_State = observation_to_Belief__index->second;
    
    auto successor_state__index = searchable_Belief_State.find(successor_state);
    if(successor_state__index == searchable_Belief_State.end()){
        
        INTERACTIVE_VERBOSER(true, 9087, "1st. Reaching  :: "<<*successor_state
                             <<"p := "<<probability<<std::endl
                             <<"new state is :: "<<successor_state<<std::endl
                             <<"new state is :: "<<*successor_state<<std::endl);
        
        
        searchable_Belief_State[successor_state] = probability;
    } else {
        searchable_Belief_State[successor_state] += /*successor_state__index->second +*/ probability;
        
        INTERACTIVE_VERBOSER(true, 9087, "nth. Reaching  :: "<<*successor_state
                             <<"p := "<<searchable_Belief_State[successor_state]<<std::endl);
    }
    
}

void Solver::expand_belief_state(POMDP_State* pomdp_state)
{   
    auto belief_state = pomdp_state->get__belief_state();
    
    POMDP_State::Action__to__Observation_to_Belief _successor_belief_state ;
    POMDP_State::Normalisation_Factors _normalisation_Factors;

    std::set<uint> legal_actions;



    
    
    
    /* For each mdp-state (i.e., atom) in the belief.*/
    for(auto atom = belief_state.begin()
            ; atom != belief_state.end()
            ; atom++){
        
        auto starting_state = atom->first;
        auto starting_state_probability = atom->second;

        
        INTERACTIVE_VERBOSER(true, 9094, "Looking at successors of  :: "
                             <<*starting_state<<std::endl
                             <<"That occurs in a belief with probability :: "<<starting_state_probability<<std::endl);
        
        /* Expand each individual member of the belief state that is not
         * already expanded. */
        if(!starting_state->get__has_been_expanded()){
            assert(dynamic_cast<const State*>(starting_state));
            
            INTERACTIVE_VERBOSER(true, 9094, "START Expanding MDP-state  :: "
                                 <<*starting_state<<std::endl);
            
            expand_optional_transformations(dynamic_cast<State*>(starting_state));
            INTERACTIVE_VERBOSER(true, 9094, "DONE Expanding MDP-state  :: "
                                 <<*starting_state<<std::endl);
        }
        
        auto successors = starting_state->get__successors();
        auto successor_Probabilities = starting_state->get__successor_Probabilities();
        auto successor_Drivers = starting_state->get__successor_Driver();
        
        assert(successors.size() == successor_Probabilities.size());
        assert(successors.size() == successor_Drivers.size());
        
        /* For each action that can be executed at \local{state}.*/
        for(uint driver_index = 0
                ; driver_index < successors.size()
                ; driver_index++){
            /* Action that generates the successor.*/
            auto action_index = successor_Drivers[driver_index];
            legal_actions.insert(action_index);
            
            
            /* States that can result from executing that action at \local{state}.*/
            auto& successor_states = successors[driver_index];
            /* Probability that a particular \local{successor_states} results after
             * executing action \local{action_index}  at \local{state}.*/
            auto& outcome_probabilities = successor_Probabilities[driver_index];

            assert(outcome_probabilities.size() == successor_states.size());

            /* For each state that can result from exeucting \local{action_index} at \local{state}.*/
            for(uint successor_state_index =  0
                    ; successor_state_index <  successor_states.size()
                    ; successor_state_index++){
                
                auto successor_state = successor_states[successor_state_index];
                auto successor_state__probability = outcome_probabilities[successor_state_index];
                

                
                
                
//                 INTERACTIVE_VERBOSER(true, 9081, "Successor from :: "
//                                      <<*state<<std::endl
//                                      <<symbol<<std::endl
//                                      <<*successor_state<<std::endl);
                
                auto& _observations = successor_state->get__observations();
                auto& _observation_probabilities = successor_state->get__observation_Probabilities();
//                 auto& observation_Drivers = successor_state->get__observation_Driver();
                
                assert(successor_state->action_to_observation__includes_index(action_index));
                auto observation__action_index = successor_state->get__action_to_observation__index(action_index);
                assert(observation__action_index < _observations.size());
                assert(_observation_probabilities.size() == _observations.size());
                
//                 for(auto observation_Driver = observation_Drivers.begin()
//                         ; observation_Driver != observation_Drivers.end()
//                         ; observation_Driver++){
//                     INTERACTIVE_VERBOSER(true, 9081, "Action for observation :: "<<*observation_Driver<<std::endl
//                                          <<action_index<<std::endl);
//                 }
//                 exit(0);
                
//                 assert(observation_Drivers[driver_index] == action_index);
                
                auto& observation_probabilities = _observation_probabilities[observation__action_index];
                auto& observations = _observations[observation__action_index];
                
                assert(observation_probabilities.size() == observations.size());
                
                INTERACTIVE_VERBOSER(true, 9094, "Unwinding :: "<<observations.size()<<" observations.");
                
                for(uint observation_index = 0
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
                    
                    INTERACTIVE_VERBOSER(true, 9094, "Starting at state with probability :: "<<starting_state_probability<<std::endl
                                         <<"Got a successor  :: "
                                         <<*successor_state<<std::endl
                                         <<"Occurring with probability :: "<<successor_state__probability<<std::endl
                                         <<"Under action :: "<<symbol.get__identifier()<<std::endl
                                         <<"Then :: "<<*observation<<std::endl
                                         <<"With probability :: "<<observation_probability<<std::endl
                                         <<"For a total transition probability of :: "
                                         <<(starting_state_probability * successor_state__probability * observation_probability)
                                         <<std::endl
                                         <<"Starting prob . "<<starting_state_probability<<std::endl
                                         <<"Successor prob. "<<successor_state__probability<<std::endl
                                         <<"Observation prob. "<<observation_probability<<std::endl);
                    
//                     INTERACTIVE_VERBOSER(true, 9083, "Adding entry to POMDP successors :: "
//                                          <<*observation<<std::endl
//                                          << (state_probability * successor_state__probability * observation_probability)
//                                          << " " <<action_index<<std::endl);
                    
                    INTERACTIVE_VERBOSER(true, 9094, "Pushing observation :: "<<*observation
                                         <<" p:= "<<observation_probability<<std::endl);
                    
                    add_entry(_normalisation_Factors
                              , _successor_belief_state
                              , action_index
                              , observation
                              , successor_state
                              , (starting_state_probability * successor_state__probability * observation_probability));
                }
            }
        }
    }


    auto _legal_actions = std::vector<uint>(legal_actions.begin(), legal_actions.end());
    for(auto atom = belief_state.begin()
            ; atom != belief_state.end()
            ; atom++){
        auto starting_state = atom->first;
        auto starting_state_probability = atom->second;


        auto sorted__successor_Drivers = starting_state->get__sorted__successor_Driver();
        
        std::vector<uint> difference(_legal_actions.size());//std::max(sorted__successor_Drivers.size(), _legal_actions.size()));
        
        auto difference_end = set_difference(_legal_actions.begin(), _legal_actions.end(),
                                             sorted__successor_Drivers.begin(), sorted__successor_Drivers.end(),
                                             difference.begin()
                                             );
        
        for(auto action_index = difference.begin()
            ; action_index != difference_end
            ; action_index++){
            
            fill_illegals(_normalisation_Factors
                          , _successor_belief_state
                          , *action_index
                          , starting_state
                          ,  starting_state_probability);
        }
    }
    
    
    press__belief_transitions(pomdp_state
                              , _normalisation_Factors
                              , _successor_belief_state );
}

bool Solver::expand_belief_state_space()
{
    if(!expansion_queue.size()){
        INTERACTIVE_VERBOSER(true, 10015, "Expty expansion queue :: "<<std::endl);
        return false;
    }
    
    auto pomdp_state = expansion_queue.front();
    expansion_queue.pop();

    INTERACTIVE_VERBOSER(true, 10003, "Expanding POMDP state :: "<<*pomdp_state<<std::endl);
    
    expand_belief_state(pomdp_state);
    
    assert(!pomdp_state->unexpanded());
    
    INTERACTIVE_VERBOSER(true, 10015, "Expanded POMDP state :: "<<*pomdp_state<<std::endl);

    return true;
}


#endif
