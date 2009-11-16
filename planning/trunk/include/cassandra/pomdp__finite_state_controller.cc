/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
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



#include "pomdp__finite_state_controller.hh"

using namespace POMDP;



FSC::Finite_State_Controller(shared_ptr<Problem_Data> problem_Data,
                             uint number_of_nodes)
    :number_of_nodes(number_of_nodes),
     problem_Data(problem_Data),
     fsc__Index_Management(*this)
{
}

int FSC::get__nodes_count() const
{
    assert(number_of_nodes > 0);
    return number_of_nodes;
}

void FSC::set__action_execution_probability(int node_index,
                                            int action_index,
                                            double action_choice_probability)
{
    assert(node_index < action_execution_probabilities.size());
    assert(action_index < action_execution_probabilities[node_index].size());
    action_execution_probabilities
        [node_index]
        [action_index]
         = action_choice_probability;
}

void FSC::set__node_transition_probability(int action_index,
                                           int observation_index,
                                           int starting_node_index,
                                           int successor_node_index,
                                           double transition_probability)
{
    
    assert(starting_node_index < node_transition_probabilities.size());
    
    assert(action_index <
           node_transition_probabilities
           [starting_node_index].size());
    
    assert(observation_index <
           node_transition_probabilities
           [starting_node_index]
           [action_index].size());
    
    assert(successor_node_index < node_transition_probabilities
           [starting_node_index]
           [action_index]
           [observation_index].size());
    
    node_transition_probabilities
        [starting_node_index]
        [action_index]
        [observation_index]
        [successor_node_index]
        = transition_probability;   
}



void FSC::zero_initialise()
{
    zero_initialise__node_transition_probabilities();
    zero_initialise__action_execution_probabilities();
}

void FSC::zero_initialise__node_transition_probabilities()
{
    node_transition_probabilities =  decltype(node_transition_probabilities)(number_of_nodes);
    
    for(auto starting_node_index = 0
            ; starting_node_index < number_of_nodes
            ; starting_node_index ++){

        assert(starting_node_index < node_transition_probabilities.size());
        node_transition_probabilities[starting_node_index]
            = std::vector<std::vector<std::vector< double > > >(problem_Data->get__actions_count());
        
        for(auto action_index = 0
                ; action_index < problem_Data->get__actions_count()
                ; action_index++){
            
            assert(action_index < node_transition_probabilities[starting_node_index].size());
            node_transition_probabilities
                [starting_node_index]
                [action_index]
                = std::vector<std::vector< double > >(problem_Data->get__observations_count());
            
            for(auto observation_index = 0
                    ; observation_index < problem_Data->get__observations_count()
                    ; observation_index++){
                
                assert(observation_index < node_transition_probabilities[starting_node_index][action_index].size());
                
                node_transition_probabilities
                    [starting_node_index]
                    [action_index]
                    [observation_index]
                    = std::vector< double >(number_of_nodes);
            
                for(auto successor_node_index = 0
                        ; successor_node_index < number_of_nodes
                        ; successor_node_index ++){

                    assert(successor_node_index
                           < node_transition_probabilities[starting_node_index][action_index][observation_index].size());
                    
                    node_transition_probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index]
                         = 0.0;
                }
            }
        }
    }
}

void FSC::zero_initialise__action_execution_probabilities()
{
    action_execution_probabilities = decltype(action_execution_probabilities)(number_of_nodes);
    
    for(auto starting_node_index = 0
            ; starting_node_index < number_of_nodes
            ; starting_node_index ++){

        assert(starting_node_index < action_execution_probabilities.size());
        action_execution_probabilities[starting_node_index] = std::vector< double >(problem_Data->get__actions_count());
        
        
        for(auto action_index = 0
                ; action_index < problem_Data->get__actions_count()
                ; action_index++){

        assert(action_index < action_execution_probabilities[starting_node_index].size());
            
            action_execution_probabilities[starting_node_index]
                [action_index]
                = 0.0;
        }
    }
}

void FSC__Randomizer::randomize__node_transition_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){

        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            
            for(auto observation_index = 0
                    ; observation_index < fsc.problem_Data->get__observations_count()
                    ; observation_index++){
                
            
                for(auto successor_node_index = 0
                        ; successor_node_index < fsc.number_of_nodes
                        ; successor_node_index ++){
                    
                    assert(starting_node_index < fsc.node_transition_probabilities.size());
                    assert(action_index < fsc.node_transition_probabilities[starting_node_index].size());
                    assert(observation_index < fsc.node_transition_probabilities[starting_node_index][action_index].size());
                    assert(successor_node_index
                           < fsc.node_transition_probabilities[starting_node_index][action_index][observation_index].size());
                    
                    
                    assert(starting_node_index < fsc.action_execution_probabilities.size());
                    assert(action_index < fsc.action_execution_probabilities[starting_node_index].size());
                    auto tmp_limit = fsc.action_execution_probabilities[starting_node_index][action_index];
                    assert(tmp_limit <= 1.0);
                    
                    fsc.node_transition_probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index] = (tmp_limit / static_cast<double>(fsc.number_of_nodes));
                }
            }
        }
    }
}

bool FSC__Randomizer::sanity__node_transition_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){

        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            
            for(auto observation_index = 0
                    ; observation_index < fsc.problem_Data->get__observations_count()
                    ; observation_index++){
                
                double  accumulate = 0.0;
                
                for(auto successor_node_index = 0
                        ; successor_node_index < fsc.number_of_nodes
                        ; successor_node_index ++){
                    
                    assert(starting_node_index < fsc.node_transition_probabilities.size());
                    assert(action_index < fsc.node_transition_probabilities[starting_node_index].size());
                    assert(observation_index < fsc.node_transition_probabilities[starting_node_index][action_index].size());
                    assert(successor_node_index
                           < fsc.node_transition_probabilities[starting_node_index][action_index][observation_index].size());
                    
                    
                    accumulate += fsc.node_transition_probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index];// = 1.0 / static_cast<double>(fsc.number_of_nodes);
                }


                assert(starting_node_index < fsc.action_execution_probabilities.size());
                assert(action_index < fsc.action_execution_probabilities[starting_node_index].size());
                auto tmp_limit = fsc.action_execution_probabilities[starting_node_index][action_index];
                assert(tmp_limit <= 1.0);

                if(!are_Doubles_Close(accumulate, tmp_limit)){
                    WARNING("Invalid node transition probabilities generated for random controller.\n"
                            <<"Limit was :: "<<std::scientific<<tmp_limit<<std::endl
                            <<"And we got :: "<<std::scientific<<accumulate<<std::endl);

                    return false;
                }
            }
        }
    }

    return true;
}



FSC__Randomizer::Finite_State_Controller__Randomizer()
    :are_Doubles_Close(1e-9)
{
}

bool FSC__Randomizer::sanity__action_execution_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){
        double  accumulate = 0.0;
        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            assert(starting_node_index < fsc.action_execution_probabilities.size());
            assert(action_index < fsc.action_execution_probabilities[starting_node_index].size());
            
            accumulate += fsc.action_execution_probabilities[starting_node_index][action_index];
        }


        if(!are_Doubles_Close(accumulate, 1.0)){
            WARNING("Invalid action choice probabilities ::"<<accumulate<<" generated for random controller.");
            return false;
        }
        
//         if(comparator__double_closeness(accumulate, 1.0)){//abs(accumulate - 1.0) < epsilon ){
//             WARNING("Invalid action choice probabilities ::"<<accumulate<<" generated for random controller.");
            
//             return false;
//         }
    }

    return true;
}




void FSC__Randomizer::randomize__action_execution_probabilities(FSC& fsc)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.number_of_nodes
            ; starting_node_index ++){
        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){
            
            assert(starting_node_index < fsc.action_execution_probabilities.size());
            assert(action_index < fsc.action_execution_probabilities[starting_node_index].size());
            
            fsc.action_execution_probabilities[starting_node_index][action_index]
                = 1.0 / static_cast<double>(fsc.problem_Data->get__actions_count());
        }
    }
}

void FSC__Randomizer::operator()(FSC& fsc)
{
    fsc.zero_initialise();
    
    randomize__action_execution_probabilities(fsc);
    assert(sanity__action_execution_probabilities(fsc));
    
    randomize__node_transition_probabilities(fsc);
    assert(sanity__node_transition_probabilities(fsc));
    
}

int FSC__Index_Management::compute_index__state_node(int state_index, int node_index) const
{
    int state_count = fsc.problem_Data->get__states_count();
    int node_count = fsc.number_of_nodes;

    return state_index + (node_index * state_count);
}
