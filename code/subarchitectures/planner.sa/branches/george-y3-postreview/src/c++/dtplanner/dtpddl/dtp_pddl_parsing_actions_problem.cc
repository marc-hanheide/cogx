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
#include "dtp_pddl_parsing_actions_problem.hh"

#include "dtp_pddl_parsing_data_problem.hh"


/******************************************************************************************************************
 * Parsing actions that are specific to problems.
 *
 *
 ******************************************************************************************************************/
namespace Planning
{
    namespace Parsing
    {
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Problem_Name__Action, add__problem_Name);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Minimise__Action, report__minimisation_objective);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Maximise__Action, report__maximisation_objective);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Starting_State__Action, report__starting_state);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Objective_Formula__Action, report__objective_function);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Goal_Formula__Action, report__goal_formula);
        
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Start_Initial_State_Parsing__Action, report__enter_parsing_initial_state);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Stop_Initial_State_Parsing__Action, report__exit_parsing_initial_state);
    }
}

/******************************************************************************************************************
 * Parsing actions that are _not_ specific to problems, but the
 * implementation here is only for the problem case.
 *
 *
 ******************************************************************************************************************/
namespace Planning
{
    namespace Parsing
    {

        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Start_Effect_Parsing__Action,
                                                     report__enter_parsing_effect_context);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Stop_Effect_Parsing__Action,
                                                     report__exit_parsing_effect_context);
        
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Domain_Name__Action, reset__domain_Data);

        
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Constant__Action, add__constant);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Type__TO__Type_Of_Constant__Action
                                                    , convert__type_of_type__TO__type_of_constant);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Constant__Action, add__type_of_constant);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Add_Constants__Action, add__constants);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Commit_Constants__Action, commit__constants);


        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(State_Function_Name__Action, report__state_function_name);

        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Skip_Next____Formula__Action____Action, report__skip_next____report__formula);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_State_Function__Action , report__formula_state_function);
//         problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Variable_Cluster__Action, stack__typed_Arguments);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Formula__Action, report__formula);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Constant_Argument__Action, add__constant_argument);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Number__Action, add__number);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Predicate_Name__Action, report__predicate_name);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_Predicate__Action, report__formula_predicate);
        

        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Dive__Action, report__dive);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Emerge__Action, report__emerge);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(And__Action, report__and_formula);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Forall__Action, report__forall_formula);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Variable_Argument__Action, add__variable_argument);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type__Action, add__type);
        problem__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Type__Action, add__type_of_type);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Argument__Action, commit__argument_types);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Probabilistic__Action, report__probabilistic_formula);
        
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Number_In_Formula__Action, report__number_in_formula);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Object_In_Formula__Action, report__object_in_formula);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Constant_In_Formula__Action, report__constant_in_formula);
        
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(GOT_REAL_NUMBER__Action, report__parsing_real_number);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(GOT_INTEGER_NUMBER__Action, report__parsing_integer_number);


        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Increase__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Decrease__Action);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Assign__Action, report__assign_formula);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Equality__Action, report__equality_formula);


        
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Action_Name__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Percept_Name__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Perceptual_Function_Name__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Empty_Formula__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Formula_Action__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Formula_Percept__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Formula_Perceptual_Function__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Not__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Or__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(If__Action);
        problem__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Variable_Cluster__Action, stack__typed_Arguments);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Exists__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Conditional_Effect__Action);
        problem__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Forall_Effect__Action);
        
        
        
    }
}
