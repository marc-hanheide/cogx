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
#include "dtp_pddl_parsing_actions_domain.hh"

#include "dtp_pddl_parsing_data_domain.hh"


/******************************************************************************************************************
 * Parsing actions that are specific to domains.
 *
 *
 ******************************************************************************************************************/
namespace Planning
{
    namespace Parsing
    {
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Observation_Execution__Action, add__observation_execution_precondition);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Observation_Name__Action, report__observation_name);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Observation_Header__Action, add__observation_header);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Observation_Precondition__Action, add__observation_precondition);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Observation_Effect__Action, add__observation_effect);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Completed_Observation__Action, add__observation);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Percept_Description__Action, add__percept);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Predicate_Description__Action, add__predicate);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(State_Function_Domainx_Description__Action
                                                    , report__state_function_domain);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Perceptual_Function_Domainx_Description__Action
                                                    , report__perceptual_function_domain);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(State_Function_Description__Action, add__state_function);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Perceptual_Function_Description__Action, add__perceptual_function);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Derived_Predicate_Header__Action, add__derived_predicate_header);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Complete_Derived_Predicate__Action, commit__derived_predicate);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Action_Header__Action, add__action_header);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Action_Precondition__Action, add__action_precondition);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Action_Effect__Action, add__action_effect);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Completed_Action__Action, add__action);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Function_Type_Number__Action, set_function_range__number);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Function_Type_Int__Action, set_function_range__int);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Function_Type_Double__Action, set_function_range__double);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Commit_Type_Hierarchy__Action, commit__types);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Add_Types__Action, add__types);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Function_Type_Type__Action, set_function_range__type);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Requirement__Action, add__requirement);
    }
}


/******************************************************************************************************************
 * Parsing actions that are _not_ specific to domains, but the
 * implementation here is only for the domain case.
 *
 *
 ******************************************************************************************************************/
namespace Planning
{
    namespace Parsing
    {
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Start_Effect_Parsing__Action,
                                                     report__enter_parsing_effect_context);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Stop_Effect_Parsing__Action,
                                                     report__exit_parsing_effect_context);

        
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Domain_Name__Action, add__domain_Name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Predicate_Name__Action, report__predicate_name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Action_Name__Action, report__action_name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Percept_Name__Action, report__percept_name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(State_Function_Name__Action, report__state_function_name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Perceptual_Function_Name__Action, report__perceptual_function_name);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Variable_Argument__Action, add__variable_argument);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Argument__Action, commit__argument_types);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Dive__Action, report__dive);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Emerge__Action, report__emerge);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Empty_Formula__Action, report__empty_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Skip_Next____Formula__Action____Action, report__skip_next____report__formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_Predicate__Action, report__formula_predicate);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_Action__Action, report__formula_action);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_Percept__Action , report__formula_percept);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_State_Function__Action , report__formula_state_function);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Formula_Perceptual_Function__Action , report__formula_perceptual_function);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Not__Action, report__not_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(And__Action, report__and_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Or__Action, report__or_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(If__Action, report__if_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Variable_Cluster__Action, stack__typed_Arguments);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Exists__Action, report__exists_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Forall__Action, report__forall_formula);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Formula__Action, report__formula);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Constant_Argument__Action, add__constant_argument);

        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Conditional_Effect__Action, report__conditional_effect_formula);
//         domain__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Conditional_Effect__Action);/* TODO */
        domain__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Forall_Effect__Action);/* TODO */

        
        
//         domain__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Decrease__Action);/* TODO */
//         domain__UNIMPLEMENTED_PEGTL_ACTION__IMPLEMENTATION(Assign__Action);/* TODO */
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Number__Action, add__number);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type__Action, add__type);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Type__Action, add__type_of_type);

        
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Constant__Action, add__constant);
        domain__FORWARDING_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Constant__Action, add__type_of_constant);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Add_Constants__Action, add__constants);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Type_Of_Type__TO__Type_Of_Constant__Action, convert__type_of_type__TO__type_of_constant);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Commit_Constants__Action, commit__constants);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Probabilistic__Action, report__probabilistic_formula);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Number_In_Formula__Action, report__number_in_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Object_In_Formula__Action, report__object_in_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Constant_In_Formula__Action, report__constant_in_formula);
        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(GOT_REAL_NUMBER__Action, report__parsing_real_number);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(GOT_INTEGER_NUMBER__Action, report__parsing_integer_number);

        
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Increase__Action, report__increase_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Decrease__Action, report__decrease_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Assign__Action, report__assign_formula);
        domain__SIMPLE_PEGTL_ACTION__IMPLEMENTATION(Equality__Action, report__equality_formula);
        
    }
}
