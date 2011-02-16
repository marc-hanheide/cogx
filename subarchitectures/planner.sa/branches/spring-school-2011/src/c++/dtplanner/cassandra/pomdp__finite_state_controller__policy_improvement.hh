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


#ifndef POMDP__FINITE_STATE_CONTROLLER__POLICY_IMPROVEMENT_HH
#define POMDP__FINITE_STATE_CONTROLLER__POLICY_IMPROVEMENT_HH

#include "utilities.hh"
#include "pomdp__finite_state_controller__evaluation.hh"

#include <glpk.h>

namespace POMDP
{
    namespace Solving
    {
        class Finite_State_Controller__Node_Improvement
        {
        public:
            friend std::ostream& operator<<(std::ostream&, const Finite_State_Controller__Node_Improvement&);
            
            Finite_State_Controller__Node_Improvement(int node_index,
                                                      FSC& fsc,
                                                      FSC__Evaluator& fsc__Evaluator);

            /* True if the policy at the node \member{node_index}
             * could be improved. If the policy could be improved,
             * this is reflected in the \member{fsc}
             * node-transition-probability and action-choice
             * parameters -- i.e., new values are written into the
             * \member{fsc}.*/
            bool operator()();
        private:
            /* Number of linear constraints associated with
             * \member{linear_programming_problem}.*/
            int number_of_linear_constraints;

            /* Number of LP variables associated with
             * \member{linear_programming_problem}.*/
            int number_of_variables;

            
            void configure__linear_program();
            void compute__number_of_linear_constraints();
            void compute__number_of_variables();
            
            /* Index of controller node that we are going to improve */
            int node_index;

            glp_prob *linear_programming_problem;
            
            /* String name of the linear programming problem
             * \member{linear_programming_problem}.*/
            std::string linear_program_name;

#define MAX_ENTRIES (1+10000)
            
            int row_indices[MAX_ENTRIES], column_indices[1+1000];
            double matrix_entries[MAX_ENTRIES];
            int entries_index;
            
            void initialise__action_choice_probabilities();
            void initialise__node_transition_probabilities();
            void initialise__bellman_constraints();
            
            void initialise__bellman_constraints__instantaneous_reward(int starting_state_index);
            void initialise__bellman_constraints__expected_future_rewards(int starting_state_index);
            void initialise__bellman_constraints__epsilon(int starting_state_index);

            /* The linear programming problem associated with node
             * improvement would have to be solved, and the solution
             * would have to be available before these member
             * functions are called.*/
            void update_fsc_psi_entries();
            void update_fsc_eta_entries();
            
            
            typedef std::tr1::tuple<int, int> Range;
            typedef std::tr1::tuple<int, int, double> Row_Column_Coefficient;

            
            
            /* (see \member{number_of_linear_constraints}*/
            Range action_choice_probability__range;
            /* (see \member{number_of_linear_constraints})*/
            Range node_transition_probability__range;
            /* (see \member{number_of_linear_constraints})*/
            Range bellman_constraints__range;
//             /* (see \member{number_of_linear_constraints})*/
//             Range  action_choice_probability_variables__range;
//             /* (see \member{number_of_linear_constraints})*/
//             Range node_transition_probabilities__range;
            
            /* (see \member{number_of_variables})*/
            Range action_choice_variables__range;
            /* (see \member{number_of_variables})*/
            Range node_transition_variables__range;
            /* (see \member{number_of_variables})*/
            Range epsilon_variable__range;


            /* Assumed to be an evaluation object for \member{fsc} --
             * i.e., Cannot improve a node of \member{fsc} unless we
             * know the value of the controller.*/
            FSC__Evaluator& fsc__Evaluator;
            
            /*Controller*/
            FSC& fsc;
        };
        
        typedef Finite_State_Controller__Node_Improvement FSC__Node_Improvement;
        
        class Finite_State_Controller__Improvement
        {   
        public:
            Finite_State_Controller__Improvement(FSC& fsc);
            
            /* For each node of \argument{finite_State_Controller},
             * improve the controller at that node according to
             * \class{Finite_State_Controller__Node_Improvement}*/
            bool operator()();
        private:
            
            /*Controller*/
            FSC& fsc;
        };
        
        typedef Finite_State_Controller__Improvement FSC__Improvement;

        class glpk__glp_simplex__print_message;
        
        std::ostream& operator<<(std::ostream&, const glpk__glp_simplex__print_message&);

        class glpk__glp_simplex__print_message
        {
        public:
            friend std::ostream& operator<<(std::ostream&, const glpk__glp_simplex__print_message&);
            glpk__glp_simplex__print_message(int glp_simplex__return);
            int glp_simplex__return;
        };

    
        
    }
}

        

#endif
