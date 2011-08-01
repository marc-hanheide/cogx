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

#include "markov_decision_process_state.hh"

#include "observation.hh"
#include "observation__probabilistic.hh"

#include "action__state_transformation.hh"


#include "problem_grounding.hh"


using namespace Planning;

std::vector<Planning::Observational_State*>
Solver::expand_observations(Observational_State* observation, ID_TYPE action_id, State* state)
{
//     assert(state->get__observational_state_during_expansion());
    
    state->set__observational_state_during_expansion(observation);
    
    INTERACTIVE_VERBOSER(true, 9074, "Got active observation count :: "
                         <<observation->count__observations()<<std::endl);
    while(observation->count__observations()){
       auto perceives = observation->pop__observation();
       (*perceives).operator()(observation, state);
    }
    
    INTERACTIVE_VERBOSER(true, 9074, "Got probabilistic observation count :: "
                         <<observation->count__probabilistic_observations()<<std::endl);
    std::vector<Planning::Observational_State*> result(0);
    while(observation->count__probabilistic_observations()){
        auto probabilistic_observation = observation->pop__probabilistic_observation();
        std::vector<Planning::Observational_State*> successor_observations
            = (*probabilistic_observation).operator()(observation, state);

        
        assert(successor_observations.end() != /*DON'T LOSE MEMORY.*/
               find(successor_observations.begin(), successor_observations.end(), observation));

        
        for(auto successor_observation = successor_observations.begin()
                ; successor_observation != successor_observations.end()
                ; successor_observation++){
            std::vector<Planning::Observational_State*> some_results
                = expand_observations(*successor_observation, action_id, state);
            
            for(auto a_result = some_results.begin()
                    ; a_result != some_results.end()
                    ; a_result++){
                result.push_back(*a_result);
            }
        }
    }

    if(result.size() == 0){

        INTERACTIVE_VERBOSER(true, 9050, "Generating from empty observation.");
        
        result.push_back(observation);
    }

    return std::move<>(result);
}

void Solver::expand_observations(const State_Transformation* optional_transformation,
                                 State* state)
{
    assert(!state->count__compulsory_generative_transformations());
    assert(!state->count__compulsory_transformations());
    
    ID_TYPE action_id = optional_transformation->get__id();
    if(state->has__considered_observations_under_action(action_id)) {
        return;
    }
    
    Observational_State* new_observation
        = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());
    state->set__observational_state_during_expansion(new_observation);
    
    INTERACTIVE_VERBOSER(true, 9075, "************** For action :: "<<std::endl
                         <<*optional_transformation<<std::endl);

    
    INTERACTIVE_VERBOSER(true, 9075, "Waking sleepers at :: "<<*state<<std::endl);

//     if(!optional_transformation->get__traversable__Sleepers().size()){
        
//     }
    
    
    QUERY_WARNING(!optional_transformation->get__traversable__sleepers().size()
                  , "Warning, there is an action :: "<<optional_transformation->get__identifier()<<std::endl
                  <<"That generates no interesting observations."<<std::endl);
    
    optional_transformation->wake_sleepers_that_require_forcing(*state);

    
    state->report__considered_observations_under_action(action_id);
    
//     Observational_State* new_observation
//         = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());

    auto successor_observations =
        expand_observations(new_observation
                            , action_id
                            , state);    
    
    assert(successor_observations.size());
    assert(successor_observations.end() != /*DON'T LOSE MEMORY.*/
           find(successor_observations.begin(), successor_observations.end(), new_observation));

    Non_Hashed_Observational_State_Pointers successors_without_duplicates;
    
    for(auto observation = successor_observations.begin()
        ; observation != successor_observations.end()
        ; observation++){
        
        auto observation__space_iterator = observation__space.find(*observation);
        
        if(observation__space_iterator == observation__space.end()){
//             (*observation)->set__id(observation__space.size());
            observation__space.insert(*observation);
            observation__space_iterator = observation__space.find(*observation);
            assert(observation__space_iterator != observation__space.end());
        } else {
            
            if((*observation__space_iterator)->get__probability_during_expansion() < 1.0){

                assert((*observation__space_iterator)->get__probability_during_expansion() > 0.0);
                
                (*observation__space_iterator)
                    ->set__probability_during_expansion(
                        (*observation__space_iterator)->get__probability_during_expansion() +
                        (*observation)->get__probability_during_expansion());
                
                QUERY_UNRECOVERABLE_ERROR((*observation__space_iterator)->get__probability_during_expansion() > 1.0
                                          && !are_Doubles_Close
                                          (1.0,
                                           (*observation__space_iterator)->get__probability_during_expansion()),
                                          "Got an observation :: "<<(**observation__space_iterator)<<std::endl
                                          <<"that is invalid :: "
                                          <<(*observation__space_iterator)->get__probability_during_expansion());
                
            } else {
                (*observation__space_iterator)
                    ->set__probability_during_expansion(
                        (*observation)->get__probability_during_expansion());
            }
            
            delete *observation;
        }
        
        auto generate_observation = *observation__space_iterator;
        
        successors_without_duplicates.insert(generate_observation);
    }

//     if(!successors_without_duplicates.size()){
        
//         INTERACTIVE_VERBOSER(true, 9075, "Adding NULL observation..."<<std::endl);
        
//         Observational_State* new_observation
//             = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());
        
//         auto observation__space_iterator = observation__space.find(new_observation);
//         if(observation__space_iterator == observation__space.end()){
//             observation__space.insert(new_observation);
//             observation__space_iterator = observation__space.find(new_observation);
//             assert(observation__space_iterator != observation__space.end());
//         }

//         successors_without_duplicates.insert(*observation__space_iterator);
//     }
    

    assert(successors_without_duplicates.size());
    
    for(auto observation = successors_without_duplicates.begin()
            ; observation != successors_without_duplicates.end()
            ; observation++){

        
        INTERACTIVE_VERBOSER(true, 9090, "Pushing observation :: "
                             <<*(state)<<std::endl
                             <<optional_transformation->get__identifier()<<std::endl
                             <<**observation<<std::endl
                             <<"prob. "<<(*observation)->get__probability_during_expansion()<<std::endl);
        
        state->Markov_Decision_Process_State::
            push__observation
            (action_id,
             *observation,
             (*observation)->get__probability_during_expansion());
        
        (*observation)->reset__probability_during_expansion();
    }   
}

