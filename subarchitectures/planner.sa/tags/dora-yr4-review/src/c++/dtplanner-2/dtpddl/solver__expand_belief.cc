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

/*VIRTUAL*/ void Solver::report__new_belief_state(POMDP_State* successor_pomdp_state)
{
    if(successor_pomdp_state->useless()){
        INTERACTIVE_VERBOSER(true,  10908, "Got a useless belief-state :: "<<*successor_pomdp_state);
        return;
    }
    
    expansion_queue.push(successor_pomdp_state);  
}

void Solver::press__belief_transitions(POMDP_State* pomdp_state,
                                       const POMDP_State::Normalisation_Factors& _normalisation_Factors,
                                       const POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state )
{
    INTERACTIVE_VERBOSER(true, 10060, "Adding the following transition information  :: "
                         <<_successor_belief_state<<std::endl);

    /* For each action that is (and isn't -- i.e., illegal actions
     * simply spin ---and can thus be considered as no-ops--- for
     * POMDPs) executable in the belief state \argument{pomdp_state}.*/
    for(auto action__to__Observation_to_Belief = _successor_belief_state.begin()
            ; action__to__Observation_to_Belief != _successor_belief_state.end()
            ; action__to__Observation_to_Belief++){
        uint action_index = action__to__Observation_to_Belief->first;

        /* For each observation that you can recieve when you execute
         * that action at the belief state \argument{pomdp_state}.*/
        for(auto observation_to_Belief = action__to__Observation_to_Belief->second.begin()
                ; observation_to_Belief != action__to__Observation_to_Belief->second.end()
                ; observation_to_Belief++){
            Observational_State* observation = observation_to_Belief->first;

            /*Allocate some space for the successor state. */
            POMDP_State* successor_pomdp_state = new POMDP_State();
            
            INTERACTIVE_VERBOSER(true, 9086, "Observation was :: "
                                 <<*observation<<std::endl);
            
            assert(_normalisation_Factors.find(action_index) != _normalisation_Factors.end());
            assert(_normalisation_Factors.find(action_index)->second.find(observation) !=
                   _normalisation_Factors.find(action_index)->second.end());

            /* What is the probability of \local{observation} after an
             * execution of \local{action_index} at
             * \argument{pomdp_state}.*/
            double mass_of_belief_successor = _normalisation_Factors.find(action_index)->second.find(observation)->second;

            /* In the following loop, this variable
             * (\local{testing_rational_belief_measure}) should become
             * 1.0 if we have not made a mistake. We incrementally add
             * to the value the probabilities of being in successive
             * MDP states given the execution sequence. What does that
             * mean exactly. Well, basically, this variable is the sum
             * of the MDP state probabilities of a successor
             * belief-state. If we have a rational belief state, this
             * sum should equal 1.0.*/
            double testing_rational_belief_measure = 0.0;
            
            /* For each successor state that is consistent with the
             * execution sequence just described. */
            for(auto atom = observation_to_Belief->second.begin()
                    ; atom != observation_to_Belief->second.end()
                    ; atom++){
                
                /* What is the probability of \local{observation}
                 * after an execution of \local{action_index} at
                 * \argument{pomdp_state} given the execution puts us
                 * in \local{atom}.*/
                double atom_mass = atom->second / mass_of_belief_successor;

                if(atom_mass < 1e-15){/*BLACK FRIDAY*/
                    continue;
                }
                
                INTERACTIVE_VERBOSER(true, 9095, *observation<<std::endl
                                     <<"Adding to belief state :: "
                                     <<*(atom->first)<<std::endl
                                     <<atom->second<<std::endl
                                     <<"Normalised :: "<<atom_mass<<std::endl
                                     <<"Via constant :: "<<mass_of_belief_successor<<std::endl);
                testing_rational_belief_measure += atom_mass;

                /*Adding an MDP state (i.e., here called as "atom") to the successor POMDP state.*/
                successor_pomdp_state->add__belief_atom(atom->first, atom_mass);
            }

            QUERY_UNRECOVERABLE_ERROR(!are_Doubles_Close(1.0, testing_rational_belief_measure),
                                      "Got a belief state where the sum of atom probabilities sums to :: "
                                      <<testing_rational_belief_measure<<"\n that should be 1.0...");

#ifndef NDEBUG
            /* Obtain the action symbol associated with the action
             * index. This is being obtained for the purpose of
             * debugging.*/
            assert(State_Transformation::
                   ith_exists(reinterpret_cast<ID_TYPE>(problem_Grounding.get()), action_index));
            auto symbol = State_Transformation::
                make_ith<State_Transformation>
                (reinterpret_cast<ID_TYPE>(problem_Grounding.get()),
                 action_index);
#endif


            /*Have we encountered the successor state before?*/
            auto belief_state__index = belief_state__space.find(successor_pomdp_state);
            if(belief_state__index == belief_state__space.end()){
                
                INTERACTIVE_VERBOSER(true, 10060, "New successor POMDP state :: "
                                     <<*successor_pomdp_state<<std::endl);

                successor_pomdp_state->set__index(belief_state__space.size());
                successor_pomdp_state->initialise__prescribed_action_index();
                belief_state__space.insert(successor_pomdp_state);

                /* We have encountered a new belief-state, and
                 * therefore should consider futures from that
                 * state. */
                report__new_belief_state(successor_pomdp_state);
                //expansion_queue.push(successor_pomdp_state);
            } else {
                INTERACTIVE_VERBOSER(true, 10060, "Repeated successor POMDP state :: "
                                     <<*successor_pomdp_state<<std::endl);
                delete successor_pomdp_state;
                successor_pomdp_state = *belief_state__index;
            }

#ifndef NDEBUG
            INTERACTIVE_VERBOSER(true, 9090, "For action :: "<<symbol<<std::endl
                                 <<"For Observation :: "<<*observation<<std::endl
                                 <<"Got a successor belief state :: "<<*successor_pomdp_state);
#endif
            
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
    if(!null_observation){/*If \member{null_observation} has not yet been configured.*/
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

/* In the current solution and interaction procedure, this member
 * function can be called for two reasons: (1) there is an expansion
 * of the state space, and (2) a state is come about during traversal
 * of a belief trajectory, and we must ensure that we know what all
 * the legal successors are, and what their probabilities are.*/
void Solver::expand_belief_state(POMDP_State* pomdp_state)
{
    INTERACTIVE_VERBOSER(true, 10060, "Expansion of POMDP state procedure :: "<<*pomdp_state<<std::endl);
    
    
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

        
        INTERACTIVE_VERBOSER(true, 11000, "Looking at successors of  :: "
                             <<*starting_state<<std::endl
                             <<"That occurs in a belief with probability :: "<<starting_state_probability<<std::endl);
        
        /* Expand each individual member of the belief state that is not
         * already expanded. */
        if(!starting_state->get__has_been_expanded()){
            assert(dynamic_cast<const State*>(starting_state));
            
            INTERACTIVE_VERBOSER(true, 11000, "START Expanding MDP-state  :: "
                                 <<*starting_state<<std::endl);
            
            expand_optional_transformations(dynamic_cast<State*>(starting_state));
            INTERACTIVE_VERBOSER(true, 11000, "DONE Expanding MDP-state  :: "
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
                
                INTERACTIVE_VERBOSER(true, 11000, "Unwinding :: "<<observations.size()<<" observations.");
                
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
                    
                    INTERACTIVE_VERBOSER(true, 11000, "Starting at state with probability :: "<<starting_state_probability<<std::endl
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
                    
                    INTERACTIVE_VERBOSER(true, 11000, "Pushing observation :: "<<*observation
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

#ifndef NDEBUG
    for(auto atom = belief_state.begin()
            ; atom != belief_state.end()
            ; atom++){
        std::cerr<<atom->second<<" "<<*atom->first<<std::endl;
    }
#endif
    INTERACTIVE_VERBOSER(true, 11000, "Got a new belief state with the above configuration.");
    

    /* For each \local{atom} in the belief state, find the actions
     * that are executable at the \argument{pomdp_state} belief-state,
     * not at \local{atom}. The default behaviour here is to cause a
     * transition to a logically equivalent state with a severe
     * reward penalty. */
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

        /* If there are no actions available at \argument{pomdp_state}
         * that are illegal at \local{atom}, continue to a different
         * atom.*/
        if(difference.begin() == difference_end) {

#ifndef NDEBUG
            for(auto p = _legal_actions.begin()
                    ; p != _legal_actions.end()
                    ; p++){
                bool found  = false;
                for(auto q = sorted__successor_Drivers.begin()
                        ; q != sorted__successor_Drivers.end()
                        ; q++){
                    if(*p == *q){
                        found = true;
                    }
                    
                }
                assert(found);
            }
            
#endif
            
            INTERACTIVE_VERBOSER(true, 13000, "No illegal transitions at :: "
                                 <<*dynamic_cast<Planning::State*>(starting_state)<<std::endl);
            
            continue;
        }
        
        INTERACTIVE_VERBOSER(true, 14000, "Got illegal transitions at :: "
                             <<*dynamic_cast<Planning::State*>(starting_state)<<std::endl);

//         /*HERE HERE HERE*/
//         /*PLEASE DELETE*/UNRECOVERABLE_ERROR("*************Got an action that was not executable at a state.");
//         /*DELETE*/exit(0);/*HERE HERE HERE --- Actually, I should not expect this to be called, so I can get rid of these lines. Because, in moritz domain, the action is executable only once. It will be executed in every state, not just some states. So there will never be a situation where it is sometimes executable. TOMORROW I should deal with restarting and memory releasing. */
//         /*HERE HERE HERE*/
        
        assert(dynamic_cast<Planning::State*>(starting_state));
        Planning::State* _cloned_state_with_penalty
            = new Planning::State(*dynamic_cast<Planning::State*>(starting_state));
        problem_Grounding->set__objective_value(*_cloned_state_with_penalty, sink_state_penalty);
        _cloned_state_with_penalty->set__reward(sink_state_penalty);
        
        auto cloned_state_with_penalty__index = state_space.find(_cloned_state_with_penalty);
        if(cloned_state_with_penalty__index == state_space.end()){/*Created a new penalty state*/
            state_space.insert(_cloned_state_with_penalty);
            cloned_state_with_penalty__index = state_space.find(_cloned_state_with_penalty);
            assert(cloned_state_with_penalty__index != state_space.end());
            INTERACTIVE_VERBOSER(true, 11000, "New cloned state with a penalty is :: "
                                 <<**cloned_state_with_penalty__index<<std::endl);
        } else {
            INTERACTIVE_VERBOSER(true, 11000, "Deleting duplicate cloned state with a penalty :: "
                                 <<*_cloned_state_with_penalty<<std::endl);
            delete _cloned_state_with_penalty;/*Repeated construction of a penalty state.*/
        }

        auto cloned_state_with_penalty = *cloned_state_with_penalty__index;
        
        INTERACTIVE_VERBOSER(true, 12000, "Working with a POMDP transition to cloned state :: "
                             <<cloned_state_with_penalty<<std::endl);

        
        INTERACTIVE_VERBOSER(true, 13000, "Adding transiton to illegal state :: "
                             <<*cloned_state_with_penalty<<std::endl
                             <<"form state :: "<<*starting_state<<std::endl
                             <<"with probability :: "<<starting_state_probability<<std::endl
                             <<"under action :: "<<0);
            
        for(auto action_index = difference.begin()
            ; action_index != difference_end
            ; action_index++){
            
            INTERACTIVE_VERBOSER(true, 13000, "Adding transiton to illegal state :: "
                                 <<*cloned_state_with_penalty<<std::endl
                                 <<"form state :: "<<*starting_state<<std::endl
                                 <<"with probability :: "<<starting_state_probability<<std::endl
                                 <<"under action :: "<<*action_index);
            
            
            
            fill_illegals(_normalisation_Factors
                          , _successor_belief_state
                          , *action_index
                          , dynamic_cast<MDP_State*>(cloned_state_with_penalty)
                          , starting_state_probability);
        }

        
        INTERACTIVE_VERBOSER(true, 13000, "Done working with a POMDP transition to cloned state :: "
                             <<cloned_state_with_penalty<<std::endl);
    }
    
    
    
    press__belief_transitions(pomdp_state
                              , _normalisation_Factors
                              , _successor_belief_state );
}

/*VIRTUAL*/ POMDP_State* Solver::obtain__next_belief_state_for_expansion()
{
    INTERACTIVE_VERBOSER(true, 10060, "Expansion queue size is :: "<<expansion_queue.size()<<std::endl);
  
    if(!expansion_queue.size()){
        INTERACTIVE_VERBOSER(true, 10015, "Expty expansion queue :: "<<std::endl);
        return 0;
    }

    
    auto pomdp_state = expansion_queue.front();
    expansion_queue.pop();

    return pomdp_state;
}

/*VIRTUAL*/ POMDP_State* Solver::peek__next_belief_state_for_expansion()
{
    if(!expansion_queue.size()){
        WARNING("Asked to peek, however there are no states to peek at.");
        return 0;
    }
    
    auto pomdp_state = expansion_queue.front();

    return pomdp_state;
}

bool Solver::expand_belief_state_space()
{
    auto pomdp_state = obtain__next_belief_state_for_expansion();

    if(0 == pomdp_state){
        INTERACTIVE_VERBOSER(true, 10015, "No belief states left for expansion :: "<<std::endl);
        return false;
    }

    INTERACTIVE_VERBOSER(true, 14000, "Expanding POMDP state :: "<<*pomdp_state<<std::endl);
    
#ifdef LAO_STAR
    QUERY_UNRECOVERABLE_ERROR(pomdp_state->get__expansion_attempted(),
                              "Expanding a state for the second time...");
    pomdp_state->set__expansion_attempted();
#endif
    
    expand_belief_state(pomdp_state);
    
    QUERY_WARNING(false, //pomdp_state->unexpanded(),
                  "We do not seem to be able to expand POMDP state :: "
                  <<*pomdp_state<<std::endl
                  <<"I suspect this is a sink state, but if not we have a more serious bug on our hands.\n"
                  <<"Because you didn't model an action that spins on this state, I assume you want things\n"
                  <<"To fail horribly at this state.\n");
    
    INTERACTIVE_VERBOSER(true, 14000, "Expanded POMDP state :: "<<*pomdp_state<<std::endl);

    return true;
}

#endif

/* Rickie Lee Jones [|speaking|]
 *
 * They went on forever ... We lived in Arizona, and the skies always
 * had little fluffy clouds in 'em, and, uh (pause) they were long
 * (pause) and clear and (pause) there were lots of stars at night...
 *
 * -- sample from The Orb single "Little Fluffy Clouds", 1990.
 */
