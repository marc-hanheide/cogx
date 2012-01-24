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


#include "solver.hh"

#include "planning_state.hh"
#include "problem_grounding.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "action__literal.hh"
#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"

#include "observation.hh"

#include "state_formula.hh"

#include "action__simple_numeric_change.hh"

using namespace Planning;
// using namespace Planning::Parsing;

#define SATISFY_FALSE_PROPOSITIONAL_ATOMS(ATOMS_ACCESS, STATE)                        \
    {                                                                   \
        auto literals = ATOMS_ACCESS;                                   \
        for(auto literal = literals.begin()                             \
                ; literal != literals.end()                             \
                ; literal++){                                           \
                                                                        \
            /*Is the literal a negative atom?*/                         \
            if((*literal)->get__sign()){                                \
                INTERACTIVE_VERBOSER(true, 8002, "Set literal "         \
                                     <<*literal                         \
                                     <<" to satisfied in starting"      \
                                     << "state."<<std::endl);           \
                                                                        \
                /* Must have been unsatisfied in the starting state.*/  \
                (*literal)->report__newly_satisfied(STATE);             \
                                                                        \
            }                                                           \
                                                                        \
        }                                                               \
                                                                        \
    }                                                                   \


#define SATISFY_FALSE_ACTION_ATOMS(ATOMS_ACCESS, STATE)                 \
    {                                                                   \
        auto literals = ATOMS_ACCESS;                                   \
        for(auto literal = literals.begin()                             \
                ; literal != literals.end()                             \
                ; literal++){                                           \
                                                                        \
            /*Is the literal a negative atom?*/                         \
            if((*literal)->get__sign()){                                \
                INTERACTIVE_VERBOSER(true, 8002, "Set literal "         \
                                     <<*literal                         \
                                     <<" to satisfied in starting"      \
                                     << "state."<<std::endl);           \
                                                                        \
                /* Must have been unsatisfied in the starting state.*/  \
                (*literal)->starting__newly_satisfied(STATE);           \
                                                                        \
            }                                                           \
                                                                        \
        }                                                               \
                                                                        \
    }                                                                   \



void Solver::generate_starting_state()
{
    assert(problem_Grounding->get__state_Propositions().size());

    QUERY_UNRECOVERABLE_ERROR(!problem_Grounding->get__state_Functions().size()
                              , "Failing because there is no reward function. This can happen: (a) Because you have no reward function, or (b) because you have one, but do not modify its value in any action effects.");
    
    assert(problem_Grounding->get__state_Functions().size());

    assert(problem_Grounding->get__conjunctive_Normal_Form_Formulae().size());

    assert(problem_Grounding->get__disjunctive_Clauses().size());

    assert(problem_Grounding->get__deterministic_actions().size());
    
    assert(problem_Grounding->get__state_Propositions().size() ==
           Formula::State_Proposition::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Propositions().begin()->get__runtime_Thread())->second->size());
    
    assert(problem_Grounding->get__state_Functions().size() ==
           Formula::State_Ground_Function::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());
    
    assert(problem_Grounding->get__conjunctive_Normal_Form_Formulae().size() ==
           State_Formula::Conjunctive_Normal_Form_Formula::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());
    
    assert(problem_Grounding->get__disjunctive_Clauses().size() ==
           State_Formula::Disjunctive_Clause::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());
    
    assert(problem_Grounding->get__deterministic_actions().size() ==
           State_Transformation::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());

    assert(!problem_Grounding->get__observations().size() ||
           problem_Grounding->get__observations().size() ==
           Planning::Observation::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());
    
    INTERACTIVE_VERBOSER(true, 17000, "Observation count is :: "
                         <<problem_Grounding->get__observations().size()<<std::endl);

    if(problem_Grounding->get__observations().size()){
        INTERACTIVE_VERBOSER(true, 17000, "Observation count is :: "
                             <<Planning::Observation::indexed__Traversable_Collection
                             .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size()<<std::endl);
    }
    
    
    Planning::State* starting_state
        = new Planning::State(*this
                              , problem_Grounding->get__state_Propositions().size()
                              , problem_Grounding->get__state_Functions().size()
                              , problem_Grounding->get__conjunctive_Normal_Form_Formulae().size()
                              , problem_Grounding->get__disjunctive_Clauses().size()
//                               , problem_Grounding->get__literals().size()
                              , problem_Grounding->get__deterministic_actions().size()
                              , problem_Grounding->get__action_Conjunctive_Normal_Form_Formulae().size()
                              , problem_Grounding->get__action_Disjunctive_Clauses().size()
                              , problem_Grounding->get__observations().size());

    auto& literals = problem_Grounding->get__literals();
    
    INTERACTIVE_VERBOSER(true, 17000, "Number of problem literals is  :: "<<literals.size());
    
#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 17000
    for(auto some = problem_Grounding->get__state_Propositions().begin()
            ; some != problem_Grounding->get__state_Propositions().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }                      
    for(auto literal = literals.begin()                             
            ; literal != literals.end()                             
            ; literal++){
        /*GARDED*/std::cerr<<*literal<<std::endl;
    }

    for(auto some = problem_Grounding->get__state_Functions().begin()
            ; some != problem_Grounding->get__state_Functions().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }
    for(auto some = problem_Grounding->get__conjunctive_Normal_Form_Formulae().begin()
            ; some != problem_Grounding->get__conjunctive_Normal_Form_Formulae().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }
    for(auto some = problem_Grounding->get__disjunctive_Clauses().begin()
            ; some != problem_Grounding->get__disjunctive_Clauses().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }
    for(auto some = problem_Grounding->get__deterministic_actions().begin()
            ; some != problem_Grounding->get__deterministic_actions().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }

    for(auto some = problem_Grounding->get__action_Conjunctive_Normal_Form_Formulae().begin()
            ; some != problem_Grounding->get__action_Conjunctive_Normal_Form_Formulae().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }

    for(auto some = problem_Grounding->get__action_Disjunctive_Clauses().begin()
            ; some != problem_Grounding->get__action_Disjunctive_Clauses().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }

    for(auto some = problem_Grounding->get__observations().begin()
            ; some != problem_Grounding->get__observations().end()
            ; some++){
        /*GARDED*/std::cerr<<*some;
    }
#endif 
#endif 
#endif 
    
    INTERACTIVE_VERBOSER(true, 17000, "Number of propositions is :: "
                         <<problem_Grounding->get__state_Propositions().size()<<std::endl);
    
    
//     exit(0);
    
//     exit(0);
//     auto literals = problem_Grounding->get__literals(); pro
    
    for(auto literal = literals.begin()                             
            ; literal != literals.end()                             
            ; literal++){                                           

        auto listeners = (*literal)->get__traversable__listeners();
        if(!listeners.size()){
            INTERACTIVE_VERBOSER(true, 17000, "Literal has no listeners :: "<<*literal<<std::endl);
        }
        
        
        for(auto listener = listeners.begin()
                ; listener != listeners.end()
                ; listener ++){
            auto listeners2 = listener->cxx_get<State_Formula::Satisfaction_Listener>()->get__traversable__listeners();
            
            INTERACTIVE_VERBOSER(true, 17000, *literal<<" has listener :: "<<*listener<<std::endl);

            
            for(auto listener2 = listeners2.begin()
                    ; listener2 != listeners2.end()
                    ; listener2 ++){
                auto listeners3 = listener2->cxx_get<State_Formula::Satisfaction_Listener>()->get__traversable__listeners();
                INTERACTIVE_VERBOSER(true, 16000, *listener<<" has listener2 :: "<<*listener2<<std::endl);
            
                for(auto listener3 = listeners3.begin()
                        ; listener3 != listeners3.end()
                        ; listener3 ++){
                    INTERACTIVE_VERBOSER(true, 16000, *listener3<<" has listened by :: "<<*listener2<<std::endl);
                }
                
            }
            
        }                                                             
    }                                                               
                   
    INTERACTIVE_VERBOSER(true, 10909, "Initialising the count of rewarding action elements."<<std::endl);
    {
        
        auto& the_actions =  problem_Grounding->get__action_symbol__to__state_transformation();
        for(auto p = the_actions.begin()
                ; p != the_actions.end()
                ; p++){
            auto transformation = p->second;

            INTERACTIVE_VERBOSER(true, 10908, "Trying for :: "<<p->first);
            
            count_reward_assignments_at_state(State_Formula::Satisfaction_Listener__Pointer(transformation)
                                              , *starting_state
                                              , problem_Grounding->get__objective_index());
        }
    }
    INTERACTIVE_VERBOSER(true, 10909, "DONE :: Initialising the count of rewarding action elements :: "
                         <<starting_state->get__obtainable_rewards_count()<<std::endl);


    
    INTERACTIVE_VERBOSER(true, 17000, "Adding false literals"<<std::endl);
    SATISFY_FALSE_PROPOSITIONAL_ATOMS(problem_Grounding->get__literals(), *starting_state);
    INTERACTIVE_VERBOSER(true, 17000, "DONE :: Adding false literals"<<std::endl);
    
    INTERACTIVE_VERBOSER(true, 17000, "Getting the action that generates the starting state."<<std::endl);
    starting_state->add__optional_transformation(
        problem_Grounding->get__executable_starting_states_generator().get());
    INTERACTIVE_VERBOSER(true, 17000, "DONE :: Getting the action that generates the starting state."<<std::endl);

//     auto END_object = problem_Grounding->get__executable_actions_without_preconditions().end();
//     auto THING_object = problem_Grounding->get__executable_starting_states_generator();
//     auto INDEX_object = problem_Grounding->get__executable_actions_without_preconditions().find(THING_object);
    
    assert(problem_Grounding->get__executable_actions_without_preconditions().end()
           == problem_Grounding->get__executable_actions_without_preconditions()
           .find(problem_Grounding->get__executable_starting_states_generator()));
    
    INTERACTIVE_VERBOSER(true, 17000, "Executing action that generates the starting state."<<std::endl);
    expand_optional_transformation
        (starting_state,
         problem_Grounding->get__executable_starting_states_generator().get());
    INTERACTIVE_VERBOSER(true, 17000, "DONE :: Executing action that generates the starting state."<<std::endl);
    
    //expand_optional_transformations(starting_state);

    INTERACTIVE_VERBOSER(true, 17000, "Starting to create initial POMDP belief-state."<<std::endl);
    starting_belief_state = new POMDP_State();

    
    INTERACTIVE_VERBOSER(true, 17000, "Having expanded to the starting belief-states, we have an MDP state count of :: "
                         << state_space.size()<<std::endl);

    
    for(auto state = state_space.begin()
            ; state != state_space.end()
            ; state++){
        assert((*state)->get__optional_transformations()
               .find(problem_Grounding->get__executable_starting_states_generator().get())
               != (*state)->get__optional_transformations().end());
        (*state)->remove__optional_transformation(
            problem_Grounding->get__executable_starting_states_generator().get());

//         Observational_State* new_observation
//             = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());
        (*state)->set__observational_state_during_expansion(0);//new_observation);
        
        /*Initialise action literals for starting states.*/
        SATISFY_FALSE_ACTION_ATOMS(problem_Grounding->get__action_Literals(), const_cast<State&>(**state));
//         (*state)->reset__observations();
//         (*state)->reset__probabilistic_observations();

//         delete (*state)->get__observational_state_during_expansion(new_observation);
//         (*state)->set__observational_state_during_expansion(0);
        
        
//         assert( 0 == (*state)->count__observations() ) ;
        INTERACTIVE_VERBOSER(true, 17000, "A starting state is :: "
                             <<**state<<std::endl);
    }
    
    assert(starting_state->get__successor_Driver().size() == 1);
    auto successor_Probabilities = *starting_state->get__successor_Probabilities().begin();
    auto successors = *starting_state->get__successors().begin();
    assert(successor_Probabilities.size() == successors.size());
    for(uint i = 0
            ; i < successor_Probabilities.size()
            ; i++){
        auto state = successors[i];
        auto probability = successor_Probabilities[i];
        
        INTERACTIVE_VERBOSER(true, 10004, "A starting state is :: "
                             <<*state<<std::endl);

        if(probability < 1e-15){/*BLACK FRIDAY*/
                    continue;
        }
        
        starting_belief_state
            ->add__belief_atom(state, probability);
        
    }

    
    starting_belief_state->set__index(belief_state__space.size());
    starting_belief_state->initialise__prescribed_action_index();
    belief_state__space.insert(starting_belief_state);
    
    //expansion_queue.push(starting_belief_state);
    report__new_belief_state(starting_belief_state);
    assert(starting_belief_state == peek__next_belief_state_for_expansion());
    
    INTERACTIVE_VERBOSER(true, 10006, "Starting belief state is :: "
                         <<(*starting_belief_state)<<std::endl);
}
