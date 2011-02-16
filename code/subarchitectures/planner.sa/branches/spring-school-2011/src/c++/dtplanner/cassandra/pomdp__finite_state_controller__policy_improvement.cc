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


#include "pomdp__finite_state_controller__policy_improvement.hh"

using namespace POMDP;
using namespace POMDP::Solving;

#include <glpk.h>

FSC__Node_Improvement::Finite_State_Controller__Node_Improvement(int node_index,
                                                                 FSC& fsc,
                                                                 FSC__Evaluator& fsc__Evaluator)
    :node_index(node_index),
     fsc(fsc),
     fsc__Evaluator(fsc__Evaluator)
{
}


void FSC__Node_Improvement::configure__linear_program()
{
    using std::tr1::get;

    std::ostringstream oss;
    oss<<"FSC Node Optimisation -- "<<node_index;
    linear_program_name = oss.str();
    
    /* Make a linear programming object with a particular name.*/
    linear_programming_problem = glp_create_prob();
    glp_set_prob_name(linear_programming_problem,
                      linear_program_name.c_str());

    /* Set the objective of the linear program.*/
    glp_set_obj_dir(linear_programming_problem,
                    GLP_MAX);
    
    /*Find out how many constraints are to be involved in the program.*/
    compute__number_of_linear_constraints();

    VERBOSER(200, "We require :: "<<number_of_linear_constraints
             <<" linear constraints."<<std::endl);
    
    glp_add_rows(linear_programming_problem,
                 number_of_linear_constraints);


    
    assert(get<0>(action_choice_probability__range) + 1 == get<1>(action_choice_probability__range));
    
    /*Setup equality constraints on */
    for(auto i = get<0>(action_choice_probability__range) + 1
            ; i < get<1>(action_choice_probability__range) + 1
            ; i++){
        glp_set_row_bnds(linear_programming_problem,
                         i,
                         GLP_FX,
                         1.0, 1.0);
    }
    
    for(auto i = get<0>(node_transition_probability__range) + 1
            ; i < get<1>(node_transition_probability__range) + 1
            ; i++){
        glp_set_row_bnds(linear_programming_problem,
                         i,
                         GLP_FX,
                         0.0, 0.0);
    }

    /* Constraints given the current value function.*/
    decltype(fsc__Evaluator.get__value_vector()) value_vector = fsc__Evaluator.get__value_vector();

    auto state_index = 0;
    for(auto i = get<0>(bellman_constraints__range) + 1
            ; i < get<1>(bellman_constraints__range) + 1
            ; i++, state_index++){
        
        auto state_node__row_index
            = fsc.fsc__Index_Management.compute_index__state_node(
                state_index, node_index
                )
            ;

        assert(state_node__row_index >= 0);
        QUERY_UNRECOVERABLE_ERROR(state_node__row_index >= value_vector.size(),
                                  "Value function does not have an entry for state :: "
                                  <<state_index<<" at node "
                                  <<node_index<<" . Index management expects to find this at index :: "
                                  <<state_node__row_index<<std::endl);
        
        auto constraint_value = value_vector[state_node__row_index];
        
        glp_set_row_bnds(linear_programming_problem,
                         i,
                         GLP_LO,
                         constraint_value, 0.0);
    }

    
//     for(auto i = get<0>(action_choice_variables__range) + 1
//             ; i != get<1>(action_choice_variables__range) + 1
//             ; i++){
        
//         glp_set_row_bnds(linear_programming_problem,
//                          i,
//                          GLP_LO,
//                          0.0,
//                          0.0);
//     }


//     for(auto i = get<0>(node_transition_variables__range)
//             ; i != get<1>(node_transition_variables__range)
//             ; i++){
        
//         glp_set_row_bnds(linear_programming_problem,
//                          i,
//                          GLP_LO,
//                          0.0,
//                          0.0);
//     }    

    /* ----------- CONFIGURE OBJECTIVE FUNCTION ----------- */
    /* ----------- CONFIGURE OBJECTIVE FUNCTION ----------- */
    
    /*Add something for epsilon.*/
    compute__number_of_variables();

    VERBOSER(200, "We require :: "<<number_of_variables
             <<" problem variables."<<std::endl);
    
    glp_add_cols(linear_programming_problem, number_of_variables);


    /* \psi_n(a)*/
    for(auto i = get<0>(action_choice_variables__range) + 1
            ; i < get<1>(action_choice_variables__range) + 1
            ; i++){
        glp_set_col_bnds(linear_programming_problem,
                         i,
                         GLP_LO,
                         0.0,
                         0.0);
        glp_set_obj_coef(linear_programming_problem,
                         i,
                         0.0);
    }

    /* \eta_n(a, o, n') */
    for(auto i = get<0>(node_transition_variables__range) + 1
            ; i < get<1>(node_transition_variables__range) + 1
            ; i++){
        glp_set_col_bnds(linear_programming_problem,
                         i,
                         GLP_LO,
                         0.0,
                         0.0);
        glp_set_obj_coef(linear_programming_problem,
                         i,
                         0.0);
    }
    
    /* ----------- Special column variable -- EPSILON Hansen 2008 ----------- */
    /* \epsilon -- Configure details of the formula we want to
     * maximise (epsilon -- Hansen 2008)*/
    glp_set_col_bnds(linear_programming_problem,
                     get<0>(epsilon_variable__range) + 1,
                     GLP_LO,
                     0.0,
                     0.0);
//     glp_set_col_bnds(linear_programming_problem,
//                      get<0>(epsilon_variable__range) + 1,
//                      GLP_FR,/*free unbounded variable*/
//                      0.0,
//                      0.0);
    glp_set_obj_coef(linear_programming_problem,
                     get<0>(epsilon_variable__range) + 1,
                     1.0);


    /* ----------- CONFIGURE BELLMAN CONSTRAINTS ----------- */
    /* ----------- CONFIGURE BELLMAN CONSTRAINTS ----------- */

    /* Row 1 -- Action-choice probabilities at the node must sum-to-one.*/
    entries_index = 1;
    
    initialise__action_choice_probabilities();
    initialise__node_transition_probabilities();
    initialise__bellman_constraints();
    
    auto constraints_matrix_dimension = number_of_variables * number_of_linear_constraints;
    
    VERBOSER(200, "We require a :: "<<constraints_matrix_dimension
             <<" dimension constraints matrix."<<std::endl);
    
//     VERBOSER(200, "Got the following linear program for node :: "<<node_index<<std::endl
//              <<*this);

//     exit(0);
    
    glp_load_matrix(linear_programming_problem,
                    entries_index - 1,//constraints_matrix_dimension,/**/
                    row_indices,
                    column_indices,
                    matrix_entries);
    
}

void FSC__Node_Improvement::initialise__bellman_constraints__epsilon(int starting_state_index)
{
    using std::tr1::get;
    auto row_index = get<0>(bellman_constraints__range) + starting_state_index + 1;

    
    
    row_indices[entries_index] = row_index;{std::cerr<<entries_index<<" -> "<<row_index<<"  "<<__PRETTY_FUNCTION__<<std::endl;}
    column_indices[entries_index] =  get<0>(epsilon_variable__range) + 1;
    matrix_entries[entries_index] = -1.0;
    assert(row_indices[entries_index] > 0);
    assert(column_indices[entries_index] > 0);
    
    entries_index++;
}


void FSC__Node_Improvement::initialise__bellman_constraints__expected_future_rewards(int starting_state_index)
{
    using std::tr1::get;
    /* Starting state \argument{starting_state_index} determines the
     * row of the bellman equation we are dealing with.*/
    auto row_index = get<0>(bellman_constraints__range) + starting_state_index + 1;

    
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++/*, entries_index++*/){

        for(auto observation_index = 0
                ; observation_index < fsc.problem_Data->get__observations_count()
                ; observation_index++){
                
            for(auto successor_node_index = 0
                    ; successor_node_index < fsc.get__nodes_count()
                    ; successor_node_index++, entries_index++){

                double coefficient = 0.0; 
                
                for(auto successor_state_index = 0
                        ; successor_state_index < fsc.problem_Data->get__states_count()
                        ; successor_state_index++){
                    

                    auto expected_future_reward =
                        fsc__Evaluator.get__expected_reward(successor_state_index, successor_node_index);

                    if(0.0 == expected_future_reward) continue;
                    
                    auto observation_given_state_and_action =
                        fsc.problem_Data
                        ->get__observation_probability(action_index,
                                                       successor_state_index,
                                                       observation_index);

                    if(0.0 == observation_given_state_and_action) continue;
                    
                    auto probability_of_transition =
                        fsc.problem_Data
                        ->get__transition_probability(action_index,
                                                      starting_state_index,
                                                      successor_state_index);

                    if(0.0 == probability_of_transition) continue;

                    /* The coefficient includes the value vector
                     * entry, the transition probabiltiy, and the
                     * observation probability. Hang on, that does not
                     * even make sense.
                     *
                     *
                     * I think that something interesting is comming
                     * now. And I cannot concentrate. For each \eta\
                     * entry we require something interesting. Each
                     * \eta\ entry is associated with: (1) an action,
                     * (2) an observation, and (3) a successor node.
                     */

                    coefficient += expected_future_reward *
                        observation_given_state_and_action *
                        probability_of_transition;
                }

                assert(row_index < get<1>(bellman_constraints__range) + 1);
                assert(get<0>(node_transition_variables__range)
                       + action_index * (fsc.problem_Data->get__observations_count() * fsc.get__nodes_count())
                       + observation_index * (fsc.get__nodes_count())
                       + successor_node_index < get<1>(node_transition_variables__range));
                
                row_indices[entries_index] = row_index;{std::cerr<<entries_index<<" -> "<<row_index<<"  "<<__PRETTY_FUNCTION__<<std::endl;}
                column_indices[entries_index] =  get<0>(node_transition_variables__range)
                    + action_index * (fsc.problem_Data->get__observations_count() * fsc.get__nodes_count())
                    + observation_index * (fsc.get__nodes_count())
                    + successor_node_index
                    + 1;
                
                auto discount_factor = fsc.problem_Data->get__discount_factor();
                
                matrix_entries[entries_index] = discount_factor * coefficient;
                assert(row_indices[entries_index] > 0);
                assert(column_indices[entries_index] > 0);
            }
        }
    }
}


void FSC__Node_Improvement::initialise__bellman_constraints__instantaneous_reward(int starting_state_index)
{
    using std::tr1::get;
    auto row_index = get<0>(bellman_constraints__range) + starting_state_index + 1;

    
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++, entries_index++){

        double reward_coefficient = 0.0;
        

        for(auto observation_index = 0
                ; observation_index < fsc.problem_Data->get__observations_count()
                ; observation_index++){
                
            for(auto successor_state_index = 0
                    ; successor_state_index < fsc.problem_Data->get__states_count()
                    ; successor_state_index++){
                    
                
                auto reward =
                    fsc.problem_Data
                    ->get__reward(action_index,
                                  starting_state_index,
                                  successor_state_index,
                                  observation_index);

                if(0.0 == reward) continue;
                    
                auto observation_given_state_and_action =
                    fsc.problem_Data
                    ->get__observation_probability(action_index,
                                                   successor_state_index,
                                                   observation_index);

                if(0.0 == observation_given_state_and_action) continue;
                    
                auto probability_of_transition =
                    fsc.problem_Data
                    ->get__transition_probability(action_index,
                                                  starting_state_index,
                                                  successor_state_index);

                if(0.0 == probability_of_transition) continue;
                    
                reward_coefficient += probability_of_transition *
                    observation_given_state_and_action *
                    reward;
            }
        }
        
        if(0.0 == reward_coefficient) {
            entries_index--;
            continue;
        }
        
        
        row_indices[entries_index] = row_index;{std::cerr<<entries_index<<" -> "<<row_index<<"  "<<__PRETTY_FUNCTION__<<std::endl;}
        column_indices[entries_index] =  get<0>(action_choice_variables__range)
            + action_index
            + 1;
        assert(row_indices[entries_index] > 0);
        assert(column_indices[entries_index] > 0);
        matrix_entries[entries_index] = reward_coefficient;
    }
}


void FSC__Node_Improvement::initialise__bellman_constraints()
{
    /* For each POMDP state we have a constraint. */
    for(auto starting_state = 0
            ; starting_state < fsc.problem_Data->get__states_count()
            ; starting_state++){
        initialise__bellman_constraints__instantaneous_reward(starting_state);
        initialise__bellman_constraints__expected_future_rewards(starting_state);
        initialise__bellman_constraints__epsilon(starting_state);
    }
}


/* CONSTRAINTS INITIALISATION -- OBVIOUSLY WE AREN'T CONCERNED WITH
 * INITIALISING THE FSC FOR THE PURPOSES OF FSC IMPROVEMENT.*/
void FSC__Node_Improvement::initialise__node_transition_probabilities()
{
    using std::tr1::get;
    auto row_index = get<0>(node_transition_probability__range) + 1;
    assert(row_index == 2);
    
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++){
        for(auto observation_index = 0
                ; observation_index < fsc.problem_Data->get__observations_count()
                ; observation_index++){
            for(auto successor_node_index = 0
                    ; successor_node_index < fsc.get__nodes_count()
                    ; successor_node_index++, entries_index++){

                
                assert(row_index < get<1>(node_transition_probability__range) + 1);
                row_indices[entries_index] = row_index;{std::cerr<<entries_index<<" -> "<<row_index<<"  "<<__PRETTY_FUNCTION__<<std::endl;}
                column_indices[entries_index] =  get<0>(node_transition_variables__range)
                    + action_index * (fsc.problem_Data->get__observations_count() * fsc.get__nodes_count())
                    + observation_index * (fsc.get__nodes_count())
                    + successor_node_index
                    + 1;
                assert(row_indices[entries_index] > 0);
                assert(column_indices[entries_index] > 0);
                matrix_entries[entries_index] =  1.0;
            }
            
            assert(row_index < get<1>(node_transition_probability__range) + 1);
            row_indices[entries_index] = row_index;{std::cerr<<entries_index<<" -> "<<row_index<<"  "<<__PRETTY_FUNCTION__<<std::endl;}
            column_indices[entries_index] =  get<0>(action_choice_variables__range)
                + action_index 
                + 1;
            assert(row_indices[entries_index] > 0);
            assert(column_indices[entries_index] > 0);
            matrix_entries[entries_index] =  -1.0;
            entries_index++;
                
            row_index++;
            assert(row_index <= get<1>(node_transition_probability__range) + 1);
        }
    }
}

void FSC__Node_Improvement::initialise__action_choice_probabilities()
{
    using std::tr1::get;
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++, entries_index++){
        row_indices[entries_index] = 1;{std::cerr<<entries_index<<" -> "<<1<<std::endl;}
        column_indices[entries_index] = get<0>(action_choice_variables__range)
            + action_index
            + 1;
        assert(row_indices[entries_index] > 0);
        assert(column_indices[entries_index] > 0);
        matrix_entries[entries_index] = 1.0;
    }
}

void FSC__Node_Improvement::compute__number_of_variables()
{
    using std::tr1::get;
    number_of_variables = 0;

    /* Probability of an action at a node */
    number_of_variables += fsc.problem_Data->get__actions_count();

    action_choice_variables__range
        = decltype(action_choice_variables__range)
        (0, number_of_variables);
    
    /* Probability of a successor node given an action and observation */
    number_of_variables +=
        fsc.problem_Data->get__actions_count() * /* action-count */
        fsc.problem_Data->get__observations_count() * /* observation-count */
        fsc.get__nodes_count() /* node-count */ ;
    
    node_transition_variables__range
        = decltype(node_transition_variables__range)
        (get<1>(action_choice_variables__range), number_of_variables);

    /* Maximisation variable */
    number_of_variables++;
    
    epsilon_variable__range
        = decltype(epsilon_variable__range)
        (get<1>(node_transition_variables__range), number_of_variables);
}


void FSC__Node_Improvement::compute__number_of_linear_constraints()
{
    using std::tr1::get;
    
    number_of_linear_constraints = 0;

    /* Sum of probabilities of executing an action at
     * \member{node_index} must be equal-to 1.*/
    number_of_linear_constraints += 1;

    action_choice_probability__range
        = decltype(action_choice_probability__range)
        (0, number_of_linear_constraints);

    assert(get<0>(action_choice_probability__range) + 1 == get<1>(action_choice_probability__range));
    
    /* For each action and observation pair we constrain the node
     * transition function.*/
    number_of_linear_constraints += fsc.problem_Data->get__actions_count() *
        fsc.problem_Data->get__observations_count();

    node_transition_probability__range
        = decltype(node_transition_probability__range)
        (get<1>(action_choice_probability__range), number_of_linear_constraints);
    
    /* And then there is one constraint for each inequality associated
     * with the Bellman equation. */
    number_of_linear_constraints +=
        /* fsc.number_of_nodes * */ fsc.problem_Data->get__states_count();
    
    bellman_constraints__range
        = decltype(bellman_constraints__range)
        (get<1>(node_transition_probability__range), number_of_linear_constraints);    
}

void FSC__Node_Improvement::update_fsc_psi_entries()
{
    using std::tr1::get;
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++){

        auto psi_index = get<0>(action_choice_variables__range)
            + action_index
            + 1;

        
        assert(psi_index >= 0);
        assert(psi_index < number_of_variables);
        
        auto psi_entry = glp_get_col_prim(linear_programming_problem, psi_index);
        
        fsc.set__action_execution_probability(node_index, action_index, psi_entry);        
    }
}

void FSC__Node_Improvement::update_fsc_eta_entries()
{

    using std::tr1::get;
    /* Read of the values of the action choice entries -- i.e., psi*/
    for(auto action_index = 0
            ; action_index < fsc.problem_Data->get__actions_count()
            ; action_index++){
        
        for(auto observation_index = 0
                ; observation_index < fsc.problem_Data->get__observations_count()
                ; observation_index++){
            
            for(auto successor_node_index = 0
                    ; successor_node_index < fsc.get__nodes_count()
                    ; successor_node_index++, entries_index++){

                auto eta_index = get<0>(node_transition_variables__range)
                    + action_index * (fsc.problem_Data->get__observations_count() * fsc.get__nodes_count())
                    + observation_index * (fsc.get__nodes_count())
                    + successor_node_index
                    + 1;

                assert(eta_index >= 0);
                assert(eta_index < number_of_variables);
                
                auto eta_entry = glp_get_col_prim(linear_programming_problem, eta_index);

                fsc.set__node_transition_probability(action_index,
                                                    observation_index,
                                                    node_index,
                                                    successor_node_index,
                                                    eta_entry);
            }
        }
    }
}




/*This printing function is completely FUCKED -- HERE HERE HERE*/
std::ostream& POMDP::Solving::
operator<<(std::ostream& o,
           const Finite_State_Controller__Node_Improvement& fsc___Node_Improvement)
{

    auto entries_index = fsc___Node_Improvement.entries_index;
    auto row_indices = fsc___Node_Improvement.row_indices;
    auto column_indices = fsc___Node_Improvement.column_indices;
    auto matrix_entries = fsc___Node_Improvement.matrix_entries;
    
//     /* Row and column indexing start at 1*/
//     auto previous_row_index = 1;
//     auto previous_column_index = 1;
    
    /* Number of variables participating in the linear constraints.*/
    auto row_length = fsc___Node_Improvement.number_of_variables + 1;
    VERBOSER(200, "Each matrix row has :: "<<row_length<<" variables."<<std::endl);


    std::map<std::pair<int, int>, double> constraints_matrix;
    for(auto entry_index = 1; entry_index < entries_index; entry_index++){
        auto row_index = row_indices[entry_index];
        auto column_index = column_indices[entry_index];
        auto entry = matrix_entries[entry_index];

        constraints_matrix[std::pair<int, int>(row_index, column_index)] = entry;
    }

    for(auto row_index = 1
            ; row_index < fsc___Node_Improvement.number_of_linear_constraints + 1
            ; row_index++){
        
        for(auto column_index = 1; column_index < row_length; column_index++){
            auto constraint_matrix_entry =
                constraints_matrix.find(std::pair<int, int>(row_index,column_index));
                
            if(constraint_matrix_entry
               == constraints_matrix.end()){
                o<<std::setw(5)<<std::right<<0.0<<" ";
            } else {
                o<<std::setw(5)<<std::right<<constraint_matrix_entry->second<<" ";
            }
        }
        o<<std::endl;
    }
    
    return o;
}

glpk__glp_simplex__print_message::
glpk__glp_simplex__print_message(int glp_simplex__return)
    :glp_simplex__return(glp_simplex__return)
{}

std::ostream& POMDP::Solving::
operator<<(std::ostream& o, const glpk__glp_simplex__print_message& error)
{
    auto error_number = error.glp_simplex__return;
    switch(error_number){
        case 0 : o<<"The LP problem instance has been successfully solved."<<std::endl
                  <<"(This code does not necessarily mean that the solver has"<<std::endl
                  <<"found optimal solution. It only means that the solution"<<std::endl
                  <<"process was successful.)"<<std::endl
                  <<std::endl;
             break;
        case GLP_EBADB :
            o<<"Unable to start the search, because the initial basis speci-"<<std::endl
                
             <<"fied in the problem object is invalid -- the number of basic"<<std::endl
             <<"(auxiliary and structural) variables is not the same as the"<<std::endl
             <<"number of rows in the problem object."<<std::endl
             <<std::endl;
            
             break;
        case GLP_ESING :
            o<<"Unable to start the search, because the basis matrix corre-"<<std::endl
             <<"sponding to the initial basis is singular within the working precision."<<std::endl
             <<"Unable to start the search, because the basis matrix cor-"<<std::endl
             <<"responding to the initial basis is ill-conditioned, i.e. its"<<std::endl
             <<"condition number is too large."<<std::endl
             <<std::endl;
             break;
        case GLP_ECOND :
            o<<"Unable to start the search, because the basis matrix cor-"<<std::endl
             <<"responding to the initial basis is ill-conditioned, i.e. its"<<std::endl
             <<"condition number is too large."<<std::endl
             <<std::endl;
             break;
        case GLP_EBOUND :
            o<<"Unable to start the search, because some double-bounded"<<std::endl
             <<"(auxiliary or structural) variables have incorrect bounds."<<std::endl
             <<std::endl;
            break;
        case GLP_EFAIL :
            o<<"The search was prematurely terminated due to the solver"<<std::endl
             <<"failure"<<std::endl
             <<std::endl;
            break;
        case GLP_EOBJLL :
            o<<"The search was prematurely terminated, because the ob-"<<std::endl
             <<"jective function being maximized has reached its LOWER"<<std::endl
             <<"limit and continues decreasing (the dual simplex only)."<<std::endl
             <<std::endl;
             break;
        case GLP_EOBJUL :
            o<<"The search was prematurely terminated, because the ob-"<<std::endl
             <<"jective function being minimized has reached its UPPER"<<std::endl
             <<"limit and continues increasing (the dual simplex only)."<<std::endl
             <<std::endl;
            break;
        case GLP_EITLIM :
            o<<"The search was prematurely terminated, because the sim-"<<std::endl
             <<"plex iteration limit has been exceeded."<<std::endl
             <<std::endl;
             break;
        case GLP_ETMLIM :
            o<<"The search was prematurely terminated, because the time"<<std::endl
             <<"limit has been exceeded."<<std::endl
             <<std::endl;
             break;
        case GLP_ENOPFS :
            o<<"The LP problem instance has no primal feasible solution"<<std::endl
             <<"(only if the LP presolver is used)."<<std::endl
             <<std::endl;
             break;
        case GLP_ENODFS :
            o<<"The LP problem instance has no dual feasible solution"<<std::endl
             <<"(only if the LP presolver is used)."<<std::endl
             <<std::endl;
             break;
        default :
            o<<"Undocumented failure mode in call to \function{glp_simplex}."<<std::endl;
             break;
    }
    
    return o;
}


    
bool FSC__Node_Improvement::operator()()
{
    configure__linear_program();

    VERBOSER(200, "Got the following linear program for node :: "<<node_index<<std::endl
             <<*this);

//     exit(0);
    
    /* Solve the \member{linear_programming_problem}.*/
    int glp_simplex__return = glp_simplex(linear_programming_problem, NULL);

    auto message = glpk__glp_simplex__print_message(glp_simplex__return);
    VERBOSER(200, message);

    if(!glp_simplex__return){

        Are_Doubles_Close are_Doubles_Close(1e-9);
        
        if(are_Doubles_Close(glp_get_obj_val(linear_programming_problem), 0.0)){
            
            glp_delete_prob(linear_programming_problem);
            VERBOSER(200, "Was not really able to improve the controller."<<std::endl);
            
            return false;
        } else {
            VERBOSER(200, "Improvement of controller node (epsilon) is measured as :: "
                     <<glp_get_obj_val(linear_programming_problem)<<std::endl);
            
        }
        
        
        
        update_fsc_psi_entries();
        update_fsc_eta_entries();
        
        glp_delete_prob(linear_programming_problem);
        return true;
    } else {
        glp_delete_prob(linear_programming_problem);
        return false;
    }
    
    
    
//     /* If we are able to improve the policy at this node.*/
//     if(0.0 != glp_get_obj_val(linear_programming_problem)){
        
//         update_fsc_psi_entries();
//         update_fsc_eta_entries();
        
//         glp_delete_prob(linear_programming_problem);
//         return true;
//     } else {
//         glp_delete_prob(linear_programming_problem);
//         return false;
//     }
    
    assert(0);
}


FSC__Improvement::
Finite_State_Controller__Improvement(FSC& fsc)
    :fsc(fsc)
{
}

            
bool FSC__Improvement::operator()()
{
    bool nodes_could_be_improved = false;
    
    for(auto node_index = 0; node_index < fsc.get__nodes_count(); node_index++){
        
        POMDP::Solving::FSC__Evaluator fsc__Evaluator(fsc);
        
        /* Evaluate the randomised controller -- i.e., compute the value
         * of being in a state at a given controller node */
        fsc__Evaluator();
        
        VERBOSER(200, "Evaluation yields :: \n"<<fsc__Evaluator<<std::endl);
        
        POMDP::Solving::FSC__Node_Improvement fsc__Node_Improvement(node_index, fsc, fsc__Evaluator);
        auto improvement_was_possible = fsc__Node_Improvement();
        
        if(improvement_was_possible){
            nodes_could_be_improved = true;
            
            VERBOSER(200, "Node :: "<<node_index<<" was improved."<<std::endl);
            
            VERBOSER(200, "The new controller looks like :: "<<std::endl);
            VERBOSER(200, fsc<<std::endl);
        } else {
//             exit(0);
            VERBOSER(200, "Node :: "<<node_index<<" could not be improved."<<std::endl);
            VERBOSER(200, fsc<<std::endl);
        }
    }

    return nodes_could_be_improved;
}
