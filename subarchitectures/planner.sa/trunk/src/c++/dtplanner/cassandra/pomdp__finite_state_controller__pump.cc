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



#include "pomdp__finite_state_controller__pump.hh"

using namespace POMDP;
using namespace Solving;


FSC__Pump::Finite_State_Controller__Pump(FSC& fsc)
    :fsc(fsc)
{
}

void FSC__Pump::add_controller_node_for_action__action_choice_probability
(int action_index)
{
    auto made_a_prob_one_assignment = false;
    
    auto actions = fsc.problem_Data->get__actions();

    assert(action_index < actions.size());
    
    fsc.action_Execution_Probabilities
        .push_back(std::vector<double>(actions.size()));
    FSC::AEP__Starting_Nodes_Layer& additional_node = fsc.action_Execution_Probabilities.back();
    
    for(auto _action_index = 0; _action_index < actions.size(); _action_index++){
        
        if(_action_index == action_index){
            made_a_prob_one_assignment = true;
            additional_node[_action_index] = 1.0;
        } else {
            additional_node[_action_index] = 0.0;
        }
    }

    assert(made_a_prob_one_assignment);
}


/* ntp -- node transition probability */
void FSC__Pump::add_controller_node_for_action__node_transition_probability
(int input_action_index)
{

    assert(fsc.problem_Data->get__actions().valid(input_action_index));
    
    fsc.node_Transition_Probabilities.push_back(FSC::NTP__Starting_Nodes_Layer());

    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++){
        fsc.node_Transition_Probabilities.back().push_back(FSC::NTP__Actions_Layer());

        for(auto observation_index = 0
                ; observation_index < fsc.problem_Data->get__observations_count()
                ; observation_index++){
            (fsc.node_Transition_Probabilities.back())[action_index].push_back(FSC::NTP__Observations_Layer());

            for(auto successor_node_index = 0
                    ; successor_node_index  < fsc.get__nodes_count()
                    ; successor_node_index++){
                
                if (action_index == input_action_index) {
                    auto transition_probability =
                        1.0 / static_cast<double>(fsc.get__nodes_count());

                    
                    (fsc.node_Transition_Probabilities.back())[action_index][observation_index].push_back(transition_probability);
                } else {
                    (fsc.node_Transition_Probabilities.back())[action_index][observation_index].push_back(0.0);
                }
            }
        }   
    }
    
    
//     VERBOSER(200, "Adding controller node number :: "<<fsc.node_Transition_Probabilities.size() + 1
//              <<" for action :: "<<fsc.problem_Data->get__action(input_action_index)<<std::endl);

    
//     FSC::NTP& node_Transition_Probabilities = fsc.node_Transition_Probabilities;

//     VERBOSER(200, " ## ## Adding :: "<<fsc.problem_Data->get__actions_count()<<std::endl
//              <<" ## ## Actions for node :: "<<fsc.node_Transition_Probabilities.size()<<std::endl);
    
//     node_Transition_Probabilities.push_back(FSC::NTP__Starting_Nodes_Layer(fsc.problem_Data->get__actions_count()));

//     assert(node_Transition_Probabilities.size() == fsc.node_Transition_Probabilities.size());
    
//     for(auto action_index = 0
//             ; action_index < fsc.problem_Data->get__actions_count()
//             ; action_index++){

//         assert(fsc.problem_Data->get__actions().valid(action_index));
        
//         FSC::NTP__Starting_Nodes_Layer& ntp__starting_nodes_layer = fsc.node_Transition_Probabilities.back();

//         VERBOSER(200, " ## ## Adding :: "<<fsc.problem_Data->get__observations_count()<<std::endl
//                  <<" ## * ## Observations for node :: "<<fsc.node_Transition_Probabilities.size()<<std::endl
//                  <<" ## * ## At action :: "<<fsc.problem_Data->get__action(action_index)<<std::endl);
    
//         ntp__starting_nodes_layer.push_back(FSC::NTP__Actions_Layer(fsc.problem_Data->get__observations_count()));

//         assert(ntp__starting_nodes_layer.size() == fsc.node_Transition_Probabilities.back().size());
        
//         for(auto observation_index = 0
//                 ; observation_index < fsc.problem_Data->get__observations_count()
//                 ; observation_index++){
            
//             assert(fsc.problem_Data->get__observations().valid(observation_index));
        
//             FSC::NTP__Actions_Layer& ntp__actions_layer = ntp__starting_nodes_layer.back();

//             VERBOSER(200, " ## ## Adding :: "<<fsc.get__nodes_count()<<std::endl
//                      <<" ## ** ## Nodes for node :: "<<fsc.node_Transition_Probabilities.size()<<std::endl
//                      <<" ## ** ## At action :: "<<fsc.problem_Data->get__action(action_index)<<std::endl
//                      <<" ## ** ## At observation :: "<<fsc.problem_Data->get__observation(observation_index)<<std::endl);

//             /*HERE HERE HERE HERE HERE*/
            
//             ntp__actions_layer.push_back(FSC::NTP__Observations_Layer(fsc.get__nodes_count()));
            
//             for(auto successor_node_index = 0
//                     ; successor_node_index  < fsc.get__nodes_count()
//                     ; successor_node_index++){

//                 FSC::NTP__Observations_Layer& ntp__observations_layer = ntp__actions_layer.back();
                
//                 if (action_index == input_action_index) {
//                     auto transition_probability =
//                         1.0 / static_cast<double>(fsc.get__nodes_count());

//                     assert(successor_node_index < ntp__observations_layer.size());
                    
//                     ntp__observations_layer[successor_node_index] = transition_probability;
// //                     additional_successor_node.push_back(transition_probability);
//                 } else {
//                     ntp__observations_layer[successor_node_index] = 0.0;
// //                     additional_successor_node.push_back(0.0);
//                 }
//             }   
//         }
//     }
}

void FSC__Pump::add_controller_nodes__update_node_count
(int number_of_new_nodes_added)
{
    assert(number_of_new_nodes_added + fsc.get__nodes_count() == fsc.node_Transition_Probabilities.size());
    fsc.increment__nodes_count(number_of_new_nodes_added);
}

void FSC__Pump::add_controller_nodes__add_zero_probability_transitions_to_new_nodes
(int number_of_new_nodes_added)
{
    for(auto starting_node_index = 0
            ; starting_node_index < fsc.get__nodes_count() + number_of_new_nodes_added
            ; starting_node_index++){
        
        assert(starting_node_index < fsc.node_Transition_Probabilities.size());
        
        
        for(auto action_index = 0
                ; action_index < fsc.problem_Data->get__actions_count()
                ; action_index++){

            assert(action_index <
                   fsc.node_Transition_Probabilities
                   [starting_node_index].size());
            
            for(auto observation_index =0
                    ; observation_index < fsc.problem_Data->get__observations_count()
                    ; observation_index++){

                assert(observation_index <
                       fsc.node_Transition_Probabilities
                       [starting_node_index]
                       [action_index].size());
                
                for(auto successor_node_index = fsc.get__nodes_count()
                        ; successor_node_index < fsc.get__nodes_count() + number_of_new_nodes_added
                        ; successor_node_index++){

                    assert(successor_node_index == fsc.node_Transition_Probabilities
                           [starting_node_index]
                           [action_index]
                           [observation_index].size());
                    
                    fsc.node_Transition_Probabilities
                        [starting_node_index]
                        [action_index]
                        [observation_index]
                        .push_back(0.0);
                }
                
                assert(number_of_new_nodes_added + fsc.get__nodes_count() == fsc.node_Transition_Probabilities
                       [starting_node_index]
                       [action_index]
                       [observation_index].size());
            }
        }   
    }
}

void FSC__Pump::add_controller_node_for_action(int action_index)
{
    add_controller_node_for_action__action_choice_probability(action_index);
    add_controller_node_for_action__node_transition_probability(action_index);
}

void FSC__Pump::operator()()
{
    auto actions = fsc.problem_Data->get__actions();

    /* We cannot pump an empty controller.*/
    assert(fsc.get__nodes_count() > 0);
    
    /* For each action add an additional node to the controller.*/
    for(auto action_index = 0 ; action_index < actions.size(); action_index++){
        VERBOSER(200, "Adding a node for action :: "
                 <<fsc.problem_Data->get__action(action_index)<<" to the controller."<<std::endl);
        add_controller_node_for_action(action_index);

        VERBOSER(200, "Having added a node for action :: "<<fsc.problem_Data->get__action(action_index)<<std::endl
                 <<"The new controller is :: "<<fsc<<std::endl);
    }

    {char ch; std::cin>>ch;};
    
    VERBOSER(200, "Where is the segmentation fault."<<std::endl);
    
    auto number_of_new_nodes_added = (fsc.node_Transition_Probabilities.size() - fsc.get__nodes_count());

    QUERY_UNRECOVERABLE_ERROR(actions.size() != number_of_new_nodes_added,
                              " Were expecting the number of actions :: "<<actions.size()
                              <<" to equal the number of nodes added to the controller :: "<<number_of_new_nodes_added
                              <<std::endl);
    
    assert(actions.size() == number_of_new_nodes_added);

    
    VERBOSER(200, "Altering the node transition probabilities between new-and old-nodes and the new nodes."
             <<std::endl);
    add_controller_nodes__add_zero_probability_transitions_to_new_nodes(number_of_new_nodes_added);
    
    VERBOSER(200, "Altering the node count to reflect the number number of nodes."<<std::endl);
    add_controller_nodes__update_node_count(number_of_new_nodes_added);
}
