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


bool FSC::sanity__node_transition_probabilities() const
{
    for(auto starting_node_index = 0
            ; starting_node_index < (*this).number_of_nodes
            ; starting_node_index ++){

        for(auto action_index = 0
                ; action_index < (*this).problem_Data->get__actions_count()
                ; action_index++){
            
            
            for(auto observation_index = 0
                    ; observation_index < (*this).problem_Data->get__observations_count()
                    ; observation_index++){
                
                double  accumulate = 0.0;
                
                for(auto successor_node_index = 0
                        ; successor_node_index < (*this).number_of_nodes
                        ; successor_node_index ++){
                    
                    assert(starting_node_index < (*this).node_Transition_Probabilities.size());
                    assert(action_index < (*this).node_Transition_Probabilities[starting_node_index].size());
                    assert(observation_index < (*this).node_Transition_Probabilities[starting_node_index][action_index].size());
                    assert(successor_node_index
                           < (*this).node_Transition_Probabilities[starting_node_index][action_index][observation_index].size());
                    
                    
                    accumulate += (*this).node_Transition_Probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        [successor_node_index];// = 1.0 / static_cast<double>((*this).number_of_nodes);
                }


                assert(starting_node_index < (*this).action_Execution_Probabilities.size());
                assert(action_index < (*this).action_Execution_Probabilities[starting_node_index].size());
                auto tmp_limit = (*this).action_Execution_Probabilities[starting_node_index][action_index];
                assert(tmp_limit <= 1.0);

                
                Are_Doubles_Close are_Doubles_Close(1e-9);
                
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


bool FSC::sanity__action_execution_probabilities() const
{
    for(auto starting_node_index = 0
            ; starting_node_index < (*this).number_of_nodes
            ; starting_node_index ++){
        double  accumulate = 0.0;
        for(auto action_index = 0
                ; action_index < (*this).problem_Data->get__actions_count()
                ; action_index++){
            
            assert(starting_node_index < (*this).action_Execution_Probabilities.size());
            assert(action_index < (*this).action_Execution_Probabilities[starting_node_index].size());
            
            accumulate += (*this).action_Execution_Probabilities[starting_node_index][action_index];
        }


        Are_Doubles_Close are_Doubles_Close(1e-9);
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

