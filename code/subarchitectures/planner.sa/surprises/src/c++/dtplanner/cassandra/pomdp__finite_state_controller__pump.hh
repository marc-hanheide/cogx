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


#ifndef POMDP__FINITE_STATE_CONTROLLER__PUMP_HH
#define POMDP__FINITE_STATE_CONTROLLER__PUMP_HH


#include "utilities.hh"
// #include "cassandra_POMDP__parser.hh"

#include "pomdp__finite_state_controller.hh"


namespace POMDP
{
    namespace Solving
    {
        class Finite_State_Controller__Pump
        {
        public:            

            Finite_State_Controller__Pump(FSC&);

            void operator()();
        private:

//             typedef std::set<int> Set_Of_Node_Indices;
            
//             Set_Of_Node_Indices indices_of_existing_nodes;
            
            void add_controller_node_for_action(int action_index);
            void add_controller_node_for_action__action_choice_probability(int action_index);
            void add_controller_node_for_action__node_transition_probability(int action_index);
//             void add_controller_node_for_action__update_node_count(int action_index);

            
            void add_controller_nodes__update_node_count(int number_of_new_nodes_added);
            void add_controller_nodes__add_zero_probability_transitions_to_new_nodes(int number_of_new_nodes_added);

            
            /*Controller*/
            FSC& fsc;
        };
        
        
        typedef Finite_State_Controller__Pump FSC__Pump;
        
    }
}



#endif
