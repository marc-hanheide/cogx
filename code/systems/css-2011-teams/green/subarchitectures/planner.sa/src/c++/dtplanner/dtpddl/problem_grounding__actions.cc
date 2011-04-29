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




#include "problem_grounding.hh"

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"

#include "planning_formula_to_variable_ordering.hh"
#include "domain_action_to_problem_action.hh"
#include "domain_observation_to_problem_observation.hh"

#include "planning_cnf_to_state_cnf.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"

#include "planning_state.hh"

using namespace Planning;


/* usual suspects C++*/
using std::map;
using std::vector;
using std::string;
using std::list;

/* \module{planning_formula}*/
using Planning::Formula::Conjunction;
using Planning::Formula::Disjunction;
using Planning::Formula::Negation;
using Planning::Formula::Subformula;
using Planning::Formula::Subformulae;
using Planning::Formula::Vacuous;
using Planning::Derived_Predicate;



void Problem_Grounding::ground_actions()
{

    Planning::Action_Schemas& schemas = domain_Data->get__action_Schemas();
    for(Planning::Action_Schemas::iterator action = schemas.begin()
            ; action != schemas.end()
            ; action ++){
        Planning::Action_Schema schema = *action;
        ground_action_schema(schema);//const_cast<Planning::Action_Schema&>(*action));/*FIX*/
    }
}

void Problem_Grounding::simplify_action_schema_precondition(Planning::Action_Schema& action_Schema)
{
    /* If the action has no precondition.*/
    if(action_Schema.get__precondition().test_cast<Vacuous>()){
        return;
    }
    
    auto new_conjunction = simplify_formula(action_Schema.get__precondition());
    
    action_Schema.alter__precondition(new_conjunction);
}

/* --4-- */
void Problem_Grounding::
press_ground_action(const Action_Name& action_Name,
                    Subformula _precondition,  
                    Subformula __effect_formula,/*This should be completely ground at this stage -- i.e., no variable symbols.. */
                    Planning::Assignment& assignment_detail,
                    const Argument_List& action_variables
                    )
{
    /*HERE -- TURN CNF into formula with problem grounding references.*/

    INTERACTIVE_VERBOSER(true, 10017, "Pressing action :: "<<action_Name<<std::endl
                         <<"with precondition :: "<<_precondition<<std::endl);

    bool statically_executable_action = false;
    
    
    State_Formula::Conjunctive_Normal_Form_Formula__Pointer
        precondition;
    
    if(enum_types::formula_false == _precondition->get__type_name()){
        return ;
    } else if (enum_types::formula_true == _precondition->get__type_name()) {
        statically_executable_action = true;
        State_Formula::List__Disjunctive_Clauses list__Disjunctive_Clauses;
        NEW_referenced_WRAPPED_deref_POINTER
            (&problem_Data, //this,
             State_Formula::Conjunctive_Normal_Form_Formula,
             _conjunct,
             list__Disjunctive_Clauses);
        
        precondition = _conjunct.cxx_deref_get<State_Formula::Conjunctive_Normal_Form_Formula>();

        INTERACTIVE_VERBOSER(true, 10017, "Empty precondition :: "<<precondition<<std::endl);
    } else {
        Planning_CNF__to__State_CNF
            planning_CNF__to__State_CNF
            (reinterpret_cast<basic_type::Runtime_Thread>(this)
             , state_Propositions
             , literals
             , disjunctive_Clauses
             , conjunctive_Normal_Form_Formulae
             , problem_Data);
    

        planning_CNF__to__State_CNF(_precondition);

        precondition = planning_CNF__to__State_CNF.get__answer();
        INTERACTIVE_VERBOSER(true, 10017, "Interesting precondition :: "<<precondition<<std::endl);
    }

    Constant_Arguments constant_Arguments(action_variables.size());
    auto index = 0;
    for(auto argument_symbol = action_variables.begin()
            ; argument_symbol != action_variables.end()
            ; argument_symbol++){
        assert(index < constant_Arguments.size());
        if(argument_symbol->test_cast<Planning::Variable>()){
            auto variable = argument_symbol->cxx_get<Planning::Variable>();
            assert(assignment_detail.find(*variable) != assignment_detail.end());
            constant_Arguments[index++] = assignment_detail[*variable];
        } else if (argument_symbol->test_cast<Planning::Constant>()) {
            auto constant = argument_symbol->cxx_get<Planning::Constant>();
            constant_Arguments[index++] = *constant;
        } else {
            UNRECOVERABLE_ERROR("Cannot deal with argument :: "<<*argument_symbol);
        }
    }
    
    
    NEW_object_referenced_WRAPPED
        (Formula::Action_Proposition
         , action_Proposition
         , action_Name
         , constant_Arguments);
    

    
    INTERACTIVE_VERBOSER(true, 10017, "Trying for  action :: "<<action_Proposition<<std::endl
                         <<"with precondition :: "<<precondition<<std::endl);
    
    /**/
    Domain_Action__to__Problem_Action
        domain_Action__to__Problem_Action(reinterpret_cast<basic_type::Runtime_Thread>(this),
                                          assignment_detail,
                                          state_Propositions,
                                          state_Functions,
                                          literals,
                                          disjunctive_Clauses,
                                          conjunctive_Normal_Form_Formulae,
                                          *domain_Data,
                                          problem_Data,
                                          action_Proposition,
                                          precondition,
                                          deterministic_actions,
                                          executable_actions_without_preconditions,
                                          probabilistic_actions,
                                          actions_validator,
                                          action_symbol__to__state_transformation);

    domain_Action__to__Problem_Action(__effect_formula);
    auto new_action = domain_Action__to__Problem_Action.get__answer();
    
    INTERACTIVE_VERBOSER(true, 10017, "Pushing an action :: "<<new_action<<std::endl);
}

/* --3-- */
void Problem_Grounding::
ground_action_schema(const Action_Name& action_Name,
                     Subformula& effect_formula,
                     Planning::Assignment& assignment_detail, /*explicit representation of results*/
                     const map<Variable, Constants>& potential_assignments, /* constants from which the result is formed.*/
                     const Argument_List& action_variables, /*Gives the order in which variables assignment should be made -- Some of these may be constant.*/
                     Subformula __precondition,
                     const std::vector<Variable>& variables_in_order,
                     uint variable_index
 )
{
    INTERACTIVE_VERBOSER(true, 10017, "Grounding at level --3-- :: "<<action_Name<<std::endl);
    
    if(variable_index >= variables_in_order.size()){
        INTERACTIVE_VERBOSER(true, 10017, "Pressing at level --3-- :: "<<action_Name<<std::endl);
        
        press_ground_action(action_Name,
                            __precondition,
                            effect_formula,
                            assignment_detail,
                            action_variables);
        return;
    }
    
    INTERACTIVE_VERBOSER(true, 10017, "Proceeding to ground formulae at level --3-- :: "<<action_Name<<std::endl);
        
    
    auto variable = variables_in_order[variable_index];
    assert(potential_assignments.find(variable) != potential_assignments.end());
    const Constants& constants = potential_assignments.find(variable)->second;

    if(!constants.size()){
        WARNING("There are no instances of objects that can be assigned to variable  :: "<<variable<<std::endl
                <<"of action :: "<<action_Name<<std::endl);
        
        
        return;
    }
    
    for(auto constant = constants.begin()
            ; constant != constants.end()
            ; constant++){
        assert(constant->get__runtime_Thread() == reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data));
        
        assignment_detail[variable] = *constant;

        
        INTERACTIVE_VERBOSER(true, 10017, "Applying assignment applicator for action :: "<<action_Name<<std::endl
                             <<"trying assignment of :: "<<variable<<" to "<<*constant);
        
//                         std::cerr<<std::endl<<std::endl;
//                     for(auto i = 0; ; i++){

//                         if(!Formula::State_Proposition::
//                            ith_exists(runtime_Thread, i)){
//                             break;
//                         }
                        
//                         auto symbol = Formula::State_Proposition::
//                             make_ith<Formula::State_Proposition>
//                             (runtime_Thread,
//                              i);
//                         std::cerr<<symbol<<"; "<<std::endl;
//                     }{char ch; std::cin>>ch;};
        auto _precondition = assignment_Applicator(__precondition, assignment_detail);

        
//                         std::cerr<<std::endl<<std::endl;
//                     for(auto i = 0; ; i++){

//                         if(!Formula::State_Proposition::
//                            ith_exists(runtime_Thread, i)){
//                             break;
//                         }
                        
//                         auto symbol = Formula::State_Proposition::
//                             make_ith<Formula::State_Proposition>
//                             (runtime_Thread,
//                              i);
//                         std::cerr<<symbol<<"; "<<std::endl;
//                     }{char ch; std::cin>>ch;};
        if(false == std::tr1::get<1>(_precondition)){
            VERBOSER(10017, "For action :: "<<action_Name<<std::endl
                     <<"Assignment of :: "<<variable<<" to "<<*constant<<" is INVALID."<<std::endl);
            
            assignment_detail.erase(variable);
            
            continue;
        }
        
        INTERACTIVE_VERBOSER(true, 10017, "For action :: "<<action_Name<<std::endl
                 <<"Assignment of :: "<<*constant<<" to "<<variable<<" is VALID."<<std::endl);
        
        
        auto precondition = std::tr1::get<0>(_precondition);
        
        INTERACTIVE_VERBOSER(true, 10004, "For action :: "<<action_Name<<std::endl
                 <<"we got a new precondition :: "<<precondition<<std::endl);
        
        
        ground_action_schema(action_Name,
                             effect_formula,
                             assignment_detail,
                             potential_assignments,
                             action_variables,
                             precondition,
                             variables_in_order,
                             variable_index + 1);
        
        assignment_detail.erase(variable);
    }
}

/* --2-- */
void Problem_Grounding::
ground_action_schema(const Action_Name& action_Name,
                     Subformula& effect_formula,
                     Planning::Assignment& assignment_detail, /*explicit representation of results*/
                     const map<Variable, Constants>& potential_assignments, /* constants from which the result is formed.*/
                     const Argument_List& action_variables, /*Gives the order in which variables assignment should be made -- Some of these may be constant.*/
                     Subformula precondition
 )
{
    INTERACTIVE_VERBOSER(true, 10017, "Grounding at level --2-- :: "<<action_Name<<std::endl);
    
    
    Planning_Formula__to__Variable_Ordering planning_Formula__to__Variable_Ordering(*domain_Data);
    planning_Formula__to__Variable_Ordering(precondition);
    std::vector<Variable> order_in_which_to_make_assignments = planning_Formula__to__Variable_Ordering.get__answer();

    QUERY_WARNING(0 == order_in_which_to_make_assignments.size(),
                  "For action ::"<<action_Name<<" we could not show a preference in what"<<std::endl
                  <<" order to make assignments to argument variables for grounding...");
    
    INTERACTIVE_VERBOSER(true, 10017, "Will be making assignments in the following order :: "<<order_in_which_to_make_assignments<<std::endl);
    
    
    std::set<Variable> variables_that_need_assignment;
    for(auto argument = action_variables.begin()
            ; argument != action_variables.end()
            ; argument++){
        if((*argument).test_cast<Variable>()){
            variables_that_need_assignment.insert(*(*argument).cxx_get<Variable>());
        }
    }

    
    INTERACTIVE_VERBOSER(true, 10017, "All variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
    
    for(auto variable = order_in_which_to_make_assignments.begin()
            ; variable != order_in_which_to_make_assignments.end()
            ; variable++){
        assert(variables_that_need_assignment.find(*variable) != variables_that_need_assignment.end());
        variables_that_need_assignment.erase(*variable);
    }

    INTERACTIVE_VERBOSER(true, 10017, "Unconsidered variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
    if(variables_that_need_assignment.size()){
        for(auto variable = variables_that_need_assignment.begin()
                ; variable != variables_that_need_assignment.end()
                ; variable++){

            WARNING("Could not make a preference for when to make an assignment to :: "<<*variable<<std::endl
                    <<"during action grounding.");
            
            order_in_which_to_make_assignments.push_back(*variable);
        }
    }
    
    
    ground_action_schema(action_Name,
                         effect_formula,
                         assignment_detail,
                         potential_assignments,
                         action_variables, 
                         precondition,
                         order_in_which_to_make_assignments,
                         0);
    
}
/* --1-- */ 
void Problem_Grounding::ground_action_schema(Planning::Action_Schema& action_Schema)
{
    
    /* First step, we alter the action precondition formula so that it
     * corresponds to propositional CNF.*/
    simplify_action_schema_precondition(action_Schema);
    
    auto action_header = action_Schema.get__header();
    auto action_Name = action_header.get__name();
    auto arguments = action_header.get__arguments();
    auto action_Arguments = get__symbols(arguments);
    auto argument_Types = get__types(arguments);

    INTERACTIVE_VERBOSER(true, 10017, "Grounding at level --1-- :: "<<action_Schema<<std::endl);
    

    
    /* ASSERTION -- 6 */
    grow__cached_constants_of_types(argument_Types);
    
    std::map<Variable,  Constants> potential_assignments;
    assert(argument_Types .size() == action_Arguments.size());
    for(uint index = 0; index < argument_Types.size(); index++){
        auto types = argument_Types[index];
        auto _variable = action_Arguments[index];

        if(_variable.test_cast<Planning::Variable>()){
            const Planning::Variable& variable = *_variable.cxx_get<Planning::Variable>();
            
            if(types.size() == 1){
                
                QUERY_UNRECOVERABLE_ERROR(extensions_of_types.find(*types.begin()) == extensions_of_types.end(),
                                          "Could not find object of type :: "<<*types.begin()<<" when computing"<<std::endl
                                          <<"possible assignment for :: "<<variable<<std::endl);

                QUERY_WARNING(!extensions_of_types.find(*types.begin())->second.size(),
                              "Could not find objects of type :: "<<*types.begin()<<" when computing"<<std::endl
                              <<"possible assignment for :: "<<variable<<std::endl);
#ifndef NDEBUG
                if(!extensions_of_types.find(*types.begin())->second.size()){/*Making the above query interactive.*/
                    char ch; std::cin>>ch;
                }
#endif
                
                potential_assignments[variable] = extensions_of_types.find(*types.begin())->second;
            } else {
                assert(types.size());
                assert(cached_constants_of_types.end() != cached_constants_of_types.find(types));/* ASSERTION -- 6 */
                potential_assignments[variable] = cached_constants_of_types.find(types)->second;
            }
        } else if(_variable.test_cast<Planning::Constant>()) {
            continue;
        }
    }

    Planning::Assignment assignment_detail;

    assert(reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data)
           != reinterpret_cast<basic_type::Runtime_Thread>(dynamic_cast<Planning::Parsing::Formula_Data*>(&problem_Data)));
    
    auto precondition = action_Schema.get__precondition();
    auto effect = action_Schema.get__effect();
    
//     const Action_Name& _action_Name = action_Name; 
//     Planning::Formula::Subformula& _effect_formula = effect;//action_Schema.get__effect(); 
//     Planning::Assignment& _assignment_detail = assignment_detail; 
//     const std::map<Variable, Constants>& _potential_assignments = potential_assignments; 
//     const Argument_List& _action_Arguments = action_Arguments; 
//     Planning::Formula::Subformula __precondition = precondition; 
    
    ground_action_schema(action_Name,
                         effect,
                         assignment_detail,
                         potential_assignments,
                         action_Arguments,
                         precondition);
}


void Problem_Grounding::ground_starting_states()
{
    auto starting_state = problem_Data.get__starting_state();
    
    Planning::Assignment assignment_detail;//();
    
    NEW_object_referenced_WRAPPED
        (Planning::Action_Name
         , action_Name
         , "STARTING-STATE");
    
    NEW_object_referenced_WRAPPED
        (Formula::Action_Proposition
         , action_Proposition
         , action_Name
         , Planning::Constant_Arguments());
    
    
    State_Formula::Conjunctive_Normal_Form_Formula__Pointer
        precondition;
    
    NEW_referenced_WRAPPED_deref_POINTER
        (&problem_Data, //this,
         State_Formula::Conjunctive_Normal_Form_Formula,
         _conjunct,
         State_Formula::List__Disjunctive_Clauses());
        
    precondition = _conjunct.cxx_deref_get<State_Formula::Conjunctive_Normal_Form_Formula>();
    
            
    Domain_Action__to__Problem_Action
        domain_Action__to__Problem_Action(reinterpret_cast<basic_type::Runtime_Thread>(this),
                                          assignment_detail,
                                          state_Propositions,
                                          state_Functions,
                                          literals,
                                          disjunctive_Clauses,
                                          conjunctive_Normal_Form_Formulae,
                                          *domain_Data,
                                          problem_Data,
                                          action_Proposition,
                                          precondition,
                                          deterministic_actions,
                                          executable_actions_without_preconditions,
                                          probabilistic_actions,
                                          actions_validator,
                                          action_symbol__to__state_transformation);
    
    INTERACTIVE_VERBOSER(true, 10004, "STARTING STATE generation ::"<<starting_state);
    domain_Action__to__Problem_Action(starting_state);
    executable_starting_states_generator = domain_Action__to__Problem_Action.get__answer();

    INTERACTIVE_VERBOSER(true, 6000, "Pushing executable starting state :: "
                         <<executable_starting_states_generator<<std::endl);
    
    assert(executable_actions_without_preconditions.size());
    assert(executable_actions_without_preconditions.end() !=
           executable_actions_without_preconditions.find(executable_starting_states_generator));

    executable_actions_without_preconditions.erase(executable_starting_states_generator);
}

