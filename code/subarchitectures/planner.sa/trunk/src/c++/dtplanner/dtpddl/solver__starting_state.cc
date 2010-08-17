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

using namespace Planning;
// using namespace Planning::Parsing;

#define SATISFY_FALSE_ATOMS(ATOMS_ACCESS, STATE)                        \
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
        


void Solver::generate_starting_state()
{
    assert(problem_Grounding->get__state_Propositions().size());

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
    
    assert(problem_Grounding->get__observations().size() ==
           Planning::Observation::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size());
    
    INTERACTIVE_VERBOSER(true, 8060, "Observation count is :: "
                         <<problem_Grounding->get__observations().size()<<std::endl);
    
    INTERACTIVE_VERBOSER(true, 8060, "Observation count is :: "
                         <<Planning::Observation::indexed__Traversable_Collection
           .find(problem_Grounding->get__state_Functions().begin()->get__runtime_Thread())->second->size()<<std::endl);
    
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
    
    
    SATISFY_FALSE_ATOMS(problem_Grounding->get__literals(), *starting_state);
    SATISFY_FALSE_ATOMS(problem_Grounding->get__action_Literals(), *starting_state);
    
    starting_state->add__optional_transformation(
        problem_Grounding->get__executable_starting_states_generator().get());

//     auto END_object = problem_Grounding->get__executable_actions_without_preconditions().end();
//     auto THING_object = problem_Grounding->get__executable_starting_states_generator();
//     auto INDEX_object = problem_Grounding->get__executable_actions_without_preconditions().find(THING_object);
    
    assert(problem_Grounding->get__executable_actions_without_preconditions().end()
           == problem_Grounding->get__executable_actions_without_preconditions()
           .find(problem_Grounding->get__executable_starting_states_generator()));
    
    expand_optional_transformation
        (starting_state,
         problem_Grounding->get__executable_starting_states_generator().get());
    //expand_optional_transformations(starting_state);

    starting_belief_state = new POMDP_State();

    
    for(auto state = state_space.begin()
            ; state != state_space.end()
            ; state++){
        assert((*state)->get__optional_transformations()
               .find(problem_Grounding->get__executable_starting_states_generator().get())
               != (*state)->get__optional_transformations().end());
        (*state)->remove__optional_transformation(
            problem_Grounding->get__executable_starting_states_generator().get());
        assert( 0 == (*state)->count__observations() ) ;
    }
    
    
    assert(starting_state->get__successor_Driver().size() == 1);
    auto successor_Probabilities = *starting_state->get__successor_Probabilities().begin();
    auto successors = *starting_state->get__successors().begin();
    assert(successor_Probabilities.size() == successors.size());
    for(auto i = 0
            ; i < successor_Probabilities.size()
            ; i++){
        auto state = successors[i];
        auto probability = successor_Probabilities[i];
        
        INTERACTIVE_VERBOSER(true, 8059, "A starting state is :: "
                             <<*state<<std::endl);
        
        starting_belief_state
            ->add__belief_atom(state, probability);
        
    }
    
    belief_state__space.insert(starting_belief_state);
    expansion_queue.push(starting_belief_state);
    INTERACTIVE_VERBOSER(true, 8061, "Starting belief state is :: "
                         <<(*starting_belief_state)<<std::endl);
}

