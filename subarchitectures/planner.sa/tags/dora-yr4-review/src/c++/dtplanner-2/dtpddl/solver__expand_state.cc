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


#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"
#include "problem_grounding.hh"

#include "planning_state.hh"
#include "state_formula__literal.hh"

#include "action__literal.hh"
#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"

#include "state_formula__conjunctive_normal_form_formula.hh"

using namespace Planning;
using namespace Planning::Parsing;


std::vector<Planning::State*> Solver::expand(Planning::State* state)
{
    
    std::vector<Planning::State*> result(0);
    INTERACTIVE_VERBOSER(true, 10506, "Testing and applying probabilistic transformations :: "
                         <<state->count__probabilistic_transformations());
    while(state->count__probabilistic_transformations()){
        auto probabilistic_transformation = state->pop__probabilistic_transformation();

        
        INTERACTIVE_VERBOSER(true, 10506, "Expanding according to probabilistic transformation :: "
                             <<probabilistic_transformation<<std::endl);
        
        std::vector<Planning::State*> successor_states = (*probabilistic_transformation)(state);

        
        assert(successor_states.end() != /*DON'T LOSE MEMORY.*/
               find(successor_states.begin(), successor_states.end(), state));


        for(auto successor_state = successor_states.begin()
                ; successor_state != successor_states.end()
                ; successor_state++){
            std::vector<Planning::State*> some_results = expand(*successor_state);

            INTERACTIVE_VERBOSER(true, 10507, "Got some results :: "
                                 <<some_results.size());
    
            for(auto a_result = some_results.begin()
                    ; a_result != some_results.end()
                    ; a_result++){
                result.push_back(*a_result);
            }
            
        }

        
        INTERACTIVE_VERBOSER(true, 10507, "Result so far includes :: "
                             <<result.size()<<" states.");
    }
    
    INTERACTIVE_VERBOSER(true, 10506, "DONE :: Applying probabilistic transformations.");

    if(result.size() == 0){
        INTERACTIVE_VERBOSER(true, 10506, "There were no applicable transformation, so we are back where we started.");
        result.push_back(state);
    }

    for(auto resulting_state = result.begin()
            ; resulting_state != result.end()
            ; resulting_state++){
        
        INTERACTIVE_VERBOSER(true, 10506, "Applying the compulsary transformations :: "
                             <<(*resulting_state)->count__compulsory_transformations());
        while((*resulting_state)->count__compulsory_transformations()){
            auto compulsory_transformation = (*resulting_state)->pop__compulsory_transformation();
            INTERACTIVE_VERBOSER(true, 10506, "Expanding :: "
                                 <<(*(*resulting_state))<<std::endl
                                 <<"With compulsory transformation :: "<<*compulsory_transformation<<std::endl);
            
            compulsory_transformation->operator()((*resulting_state));
        }
        
        INTERACTIVE_VERBOSER(true, 10506, "Applying the compulsary generative transformations :: "
                             <<(*resulting_state)->count__compulsory_generative_transformations());
        
        while((*resulting_state)->count__compulsory_generative_transformations()){
            auto compulsory_generative_transformation = (*resulting_state)->pop__compulsory_generative_transformation();
            INTERACTIVE_VERBOSER(true, 10506, "Expanding :: "
                                 <<(*(*resulting_state))<<std::endl
                                 <<"With compulsory generative transformation :: "<<*compulsory_generative_transformation<<std::endl);
            (*compulsory_generative_transformation)((*resulting_state));
        }
    }

    return std::move<>(result);
}

std::vector<Planning::State*> Solver::expand(Planning::State* state,
                                             const State_Transformation* optional_transformation)
{

    INTERACTIVE_VERBOSER(true, 10506, "Applying optional transformation "<<std::endl);
    assert(!optional_transformation->get__compulsory());
    INTERACTIVE_VERBOSER(true, 10506, "Waking sleepers on transformation "<<std::endl);
    optional_transformation->wake_sleepers(*state);
    INTERACTIVE_VERBOSER(true, 10506, "Make the optional transformation compulsary at the state "<<std::endl);
    state->push__compulsory_transformation(optional_transformation);
    INTERACTIVE_VERBOSER(true, 10506, "Expand the state "<<std::endl);
    auto result = expand(state);
    return std::move<>(result);

    
    

        

//     while(state->count__compulsory_transformations()){
//        auto compulsory_transformation = state->pop__compulsory_transformation();
//        (*compulsory_transformation)(state);
//     }

// //     WARNING(!state->count__compulsory_generative_transformations()
// //             , "");
    
// //     while(state->count__compulsory_generative_transformations()){
// //        auto compulsory_transformation = state->pop__compulsory_generative_transformation();
// //        (*compulsory_generative_transformation)(state);
// //     }
    
//     std::vector<Planning::State*> result(0);
//     while(state->count__probabilistic_transformations()){
//         auto probabilistic_transformation = state->pop__probabilistic_transformation();
//         std::vector<Planning::State*> successor_states = (*probabilistic_transformation)(state);

        
//         assert(successor_states.end() != /*DON'T LOSE MEMORY.*/
//                find(successor_states.begin(), successor_states.end(), state));

        
//         for(auto successor_state = successor_states.begin()
//                 ; successor_state != successor_states.end()
//                 ; successor_state++){
//             std::vector<Planning::State*> some_results = expand(*successor_state);
            
//             for(auto a_result = some_results.begin()
//                     ; a_result != some_results.end()
//                     ; a_result++){
//                 result.push_back(*a_result);
//             }
//         }
//     }

//     if(result.size() == 0){
//         result.push_back(state);
//     }

//     return std::move<>(result);
}



void Solver::expand_optional_transformation(Planning::State* state
                                            , const State_Transformation* optional_transformation)
{
    Planning::State* new_state = new State(*state);

    INTERACTIVE_VERBOSER(true, 10506, "Expanding :: "
                         <<(*new_state)<<std::endl
                         <<"With action-transformation :: "<<*optional_transformation<<std::endl);
    
    
    auto successors = expand(new_state, optional_transformation);
    
    assert(successors.end() != /*DON'T LOSE MEMORY.*/
           find(successors.begin(), successors.end(), new_state));
    
    assert(successors.end() == /*DON'T MISUSE MEMORY.*/
           find(successors.begin(), successors.end(), state));
    
    Non_Hashed_State_Pointers successors_without_duplicates;
    
    for(auto _successor = successors.begin()
            ; _successor != successors.end()
            ; _successor++){
        
        auto state_space_iterator = state_space.find(*_successor);
        
        if(state_space_iterator == state_space.end()){
            double state_value =  problem_Grounding->get__objective_value(**_successor);
            (*_successor)->set__reward(state_value);
        
            INTERACTIVE_VERBOSER(true, 10506, "Setting value of state :: "<<(**_successor)<<std::endl
                                 <<state_value<<std::endl);
        
                
            state_space.insert(*_successor);
            state_space_iterator = state_space.find(*_successor);
            assert(state_space_iterator != state_space.end());
        } else {
            
            if((*state_space_iterator)->get__probability_during_expansion() < 1.0){
                
                (*state_space_iterator)
                    ->set__probability_during_expansion(
                        (*state_space_iterator)->get__probability_during_expansion() +
                        (*_successor)->get__probability_during_expansion());
                
                QUERY_UNRECOVERABLE_ERROR(!((*state_space_iterator)->get__probability_during_expansion() <= 1.0
                                            || are_Doubles_Close
                                            (1.0,
                                             (*state_space_iterator)->get__probability_during_expansion())),
                                          "Got a successor state :: "<<(**state_space_iterator)<<std::endl
                                          <<"that is invalid :: "
                                          <<(*state_space_iterator)->get__probability_during_expansion()<<std::endl
                                          <<"Because it occurs with probability :: "
                                          <<(*state_space_iterator)->get__probability_during_expansion()<<std::endl
                                          <<!are_Doubles_Close
                                          (1.0,
                                           (*state_space_iterator)->get__probability_during_expansion())<<std::endl
                                          <<((*state_space_iterator)->get__probability_during_expansion() <= 1.0)<<std::endl);
                
            } else {
                (*state_space_iterator)
                    ->set__probability_during_expansion(
                        (*_successor)->get__probability_during_expansion());
            }

//             const_cast<State*>(*state_space_iterator)
//                 ->take__observations__from(*_successor);
//             const_cast<State*>(*state_space_iterator)
//                 ->take__probabilistic_observations__from(*_successor);
            
            delete *_successor;
        }
        
        auto successor = *state_space_iterator;
        
        successors_without_duplicates.insert(successor);
    }
    
    for(auto successor = successors_without_duplicates.begin()
            ; successor != successors_without_duplicates.end()
            ; successor++){
        state->push__successor(optional_transformation->get__id()
                               , *successor
                               , (*successor)->get__probability_during_expansion());

        
        INTERACTIVE_VERBOSER(true, 10506,
                             "Expanded state using optional transformation :: "
                             <<optional_transformation->get__id()<<" "
                             <<*optional_transformation<<std::endl);
        
        expand_observations(optional_transformation/*->get__id()*/, *successor);
        
        
        (*successor)->reset__probability_during_expansion();
        (*successor)->set__observational_state_during_expansion(0);
    }
    
    state->set__has_been_expanded();
}

void Solver::expand_optional_transformations(Planning::State* state)
{
    assert(1.0 == state->get__probability_during_expansion());
    state->reset__probability_during_expansion();
    assert(0 == state->get__observational_state_during_expansion());
    
    
    auto _optional_transformations = problem_Grounding->get__executable_actions_without_preconditions();

    for(auto optional_transformation = _optional_transformations.begin()
            ; optional_transformation != _optional_transformations.end()
            ; optional_transformation++){

        INTERACTIVE_VERBOSER(true, 10000,
                             "Expanding transformation at state :: "
                             <<**optional_transformation<<std::endl);
    
        expand_optional_transformation( state
                                        , (*optional_transformation).get());
    }
    
    auto optional_transformations = state->get__optional_transformations();
    
    for(auto optional_transformation = optional_transformations.begin()
            ; optional_transformation != optional_transformations.end()
            ; optional_transformation++){

        INTERACTIVE_VERBOSER(true, 10000,
                             "Expanding transformation at state :: "
                             <<**optional_transformation<<std::endl);
    
        
        
        expand_optional_transformation( state
                                        , *optional_transformation);
    }
}

