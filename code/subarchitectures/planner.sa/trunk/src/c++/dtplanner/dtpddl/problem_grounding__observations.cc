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

#include "planning_cnf_to_state_cnf.hh"
#include "planning_cnf_to_action_cnf.hh"

#include "planning_formula_to_variable_ordering.hh"
#include "domain_observation_to_problem_observation.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "action__literal.hh"
#include "action__disjunctive_clause.hh"
#include "action__conjunctive_normal_form_formula.hh"

#include "observation.hh"
#include "observation__probabilistic.hh"

#include "planning_state.hh"

using namespace Planning;

/* \module{planning_formula}*/
using Planning::Formula::Conjunction;
using Planning::Formula::Disjunction;
using Planning::Formula::Negation;
using Planning::Formula::Subformula;
using Planning::Formula::Subformulae;
using Planning::Formula::Vacuous;
using Planning::Derived_Predicate;


void Problem_Grounding::ground_observations()
{
    /*There is an action symbol assocaited with this ground problem. -- 1*/
    assert(Formula::Action_Proposition::indexed__Traversable_Collection.find(actions_validator.first)
           != Formula::Action_Proposition::indexed__Traversable_Collection.end());
    
    actions_validator.second =
        Formula::Action_Proposition::indexed__Traversable_Collection[actions_validator.first]->size();
    
    INTERACTIVE_VERBOSER(true, 8000, "Begin grounding the observation schemata."<<std::endl
                         <<"At this point we have :: "<<actions_validator.second<<" actions.\n"
                         <<"Associated with this problem :: "<<reinterpret_cast<basic_type::Runtime_Thread>(this)
                         <<" verifying :: "<<actions_validator.first<<std::endl
                         <<"We parsed :: "<<domain_Data->get__observation_Schemas().size()<<" observation schemata"<<std::endl);
    
    /*There is an action symbol assocaited with this ground problem. -- 2*/
    assert(actions_validator.second);
    
    Planning::Observation_Schemas& schemas = domain_Data->get__observation_Schemas();
    for(Planning::Observation_Schemas::iterator observation = schemas.begin()
            ; observation != schemas.end()
            ; observation++){
        Planning::Observation_Schema schema = *observation;
        ground_observation_schema(schema);
    }
}


void Problem_Grounding::simplify_observation_schema_preconditions(Planning::Observation_Schema& observation_Schema)
{
    if(!observation_Schema.get__precondition().test_cast<Vacuous>()){
        
        auto new_conjunction = simplify_formula(observation_Schema.get__precondition());
        
        observation_Schema.alter__precondition(new_conjunction);
    }

    if(!observation_Schema.get__execution_precondition().test_cast<Vacuous>()){
        
        auto new_conjunction = simplify_formula(observation_Schema.get__execution_precondition());
        
        observation_Schema.alter__execution_precondition(new_conjunction);
    }
}

/* --4-- */
void
Problem_Grounding::
press_ground_observation(const Observation_Name& observation_Name,
                         Planning::Formula::Subformula _precondition,
                         Planning::Formula::Subformula _execution_precondition,
                         Planning::Formula::Subformula __effect_formula,
                         Planning::Assignment& assignment_detail,
                         const Argument_List& observation_variables)
{
    
    /*HERE -- TURN CNF into formula with problem grounding references.*/

    INTERACTIVE_VERBOSER(true, 8000, "Pressing perception :: "<<observation_Name<<std::endl
                         <<"with precondition :: "<<_precondition<<std::endl);

    bool statically_obtained_perception = true;
    
    
    State_Formula::Conjunctive_Normal_Form_Formula__Pointer
        precondition;
    
    Action_Conjunctive_Normal_Form_Formula__Pointer
        execution_precondition;
    
    if((enum_types::formula_false == _precondition->get__type_name()) ||
       (enum_types::formula_false == _execution_precondition->get__type_name())){
        return ;
    }

    
    if (enum_types::formula_true == _execution_precondition->get__type_name()) {
        
        List__Action_Disjunctive_Clauses list__Disjunctive_Clauses;
        NEW_referenced_WRAPPED_deref_POINTER
            (&problem_Data,//this,
             Action_Conjunctive_Normal_Form_Formula,
             _conjunct,
             list__Disjunctive_Clauses);
        
        execution_precondition = _conjunct.cxx_deref_get<Action_Conjunctive_Normal_Form_Formula>();

        INTERACTIVE_VERBOSER(true, 8000, "Empty precondition :: "<<execution_precondition<<std::endl);
    } else {
        statically_obtained_perception = false;
        
        Planning_CNF__to__Action_CNF
            planning_CNF__to__Action_CNF
            (reinterpret_cast<basic_type::Runtime_Thread>(this)
             , action_Literals
             , action_Disjunctive_Clauses
             , action_Conjunctive_Normal_Form_Formulae
//              , negative_literals
             , action_symbol__to__state_transformation);

        INTERACTIVE_VERBOSER(true, 10003, "Attempting to convert :: "<<_execution_precondition);
        
        planning_CNF__to__Action_CNF(_execution_precondition);
        execution_precondition = planning_CNF__to__Action_CNF.get__answer();
        
        INTERACTIVE_VERBOSER(true, 8000, "Interesting observational precondition :: "<<execution_precondition<<std::endl);
    }
    
    
    if (enum_types::formula_true == _precondition->get__type_name()) {
        State_Formula::List__Disjunctive_Clauses list__Disjunctive_Clauses;
        NEW_referenced_WRAPPED_deref_POINTER
            (&problem_Data,//this,
             State_Formula::Conjunctive_Normal_Form_Formula,
             _conjunct,
             list__Disjunctive_Clauses);
        
        precondition = _conjunct.cxx_deref_get<State_Formula::Conjunctive_Normal_Form_Formula>();

        INTERACTIVE_VERBOSER(true, 8000, "Empty precondition :: "<<precondition<<std::endl);
    } else {
        statically_obtained_perception = false;
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
        INTERACTIVE_VERBOSER(true, 8000, "Interesting precondition :: "<<precondition<<std::endl);
    }

    Constant_Arguments constant_Arguments(observation_variables.size());
    auto index = 0;
    for(auto argument_symbol = observation_variables.begin()
            ; argument_symbol != observation_variables.end()
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
        (Formula::Observational_Proposition
         , observation_Proposition
         , observation_Name
         , constant_Arguments);
    
    INTERACTIVE_VERBOSER(true, 8000, "Trying for  observation :: "<<observation_Proposition<<std::endl
                         <<"with precondition :: "<<precondition<<std::endl
                         <<"with execution precondition :: "<<execution_precondition<<std::endl);
    
    /**/
    Domain_Observation__to__Problem_Observation
        domain_Observation__to__Problem_Observation
        (reinterpret_cast<basic_type::Runtime_Thread>(this),
         assignment_detail,
         state_Propositions,
         perceptual_Propositions,
         state_Functions,
         literals,
         disjunctive_Clauses,
         conjunctive_Normal_Form_Formulae,
//          acton_Literals,
//          action_Disjunctive_Clauses,
//          action_Conjunctive_Normal_Form_Formulae,
         *domain_Data,
         problem_Data,
         observation_Proposition,
         precondition,
         execution_precondition,
         observations,
         observations_without_preconditions,
         actions_validator);

    domain_Observation__to__Problem_Observation(__effect_formula);
    auto new_observation = domain_Observation__to__Problem_Observation.get__answer();
    
    INTERACTIVE_VERBOSER(true, 9089, "Pushing an observation :: "<<new_observation<<std::endl);
//     exit(0);
}

/* --3-- */ 
void
Problem_Grounding::
ground_observation_schema(const Observation_Name& observation_Name,
                          Planning::Formula::Subformula& effect_formula,
                          Planning::Assignment& assignment_detail,
                          const std::map<Planning::Variable, Planning::Constants/*FIX*/>& potential_assignments,
                          const Argument_List& observation_variables, 
                          Planning::Formula::Subformula __precondition,
                          Planning::Formula::Subformula __execution_precondition,
                          const std::vector<Variable>& order_in_which_to_make_assignments,
                          uint variable_index)
{
  
    INTERACTIVE_VERBOSER(true, 8000, "Grounding at level --3-- :: "<<observation_Name<<std::endl);
    
    if(variable_index >= order_in_which_to_make_assignments.size()){
        INTERACTIVE_VERBOSER(true, 8000, "Pressing at level --3-- :: "<<observation_Name<<std::endl);
        
        /*TO BE IMPLEMENTED -- another one below*/
        press_ground_observation(observation_Name,
                                 __precondition,
                                 __execution_precondition,
                                 effect_formula,
                                 assignment_detail,
                                 observation_variables);
        return;
    }
    
    INTERACTIVE_VERBOSER(true, 8000, "Proceeding to ground formulae at level --3-- :: "<<observation_Name<<std::endl);
        
    
    auto variable = order_in_which_to_make_assignments[variable_index];
    assert(potential_assignments.find(variable) != potential_assignments.end());
    const Constants& constants = potential_assignments.find(variable)->second;

    if(!constants.size()){
        WARNING("There are no instances of objects that can be assigned to variable  :: "<<variable<<std::endl
                <<"of  observation :: "<<observation_Name<<std::endl);
        
        return;
    }
    
    for(auto constant = constants.begin()
            ; constant != constants.end()
            ; constant++){
        assignment_detail[variable] = *constant;

        
        INTERACTIVE_VERBOSER(true, 8000, "Applying assignment applicator for observation :: "<<observation_Name<<std::endl
                             <<"trying assignment of :: "<<variable<<" to "<<*constant);
        
        auto _precondition = assignment_Applicator(__precondition, assignment_detail);
        auto _execution_precondition = assignment_Applicator(__execution_precondition, assignment_detail);
        
        
        if(false == std::tr1::get<1>(_precondition)){
            VERBOSER(3001, "For observation :: "<<observation_Name<<std::endl
                     <<"Assignment of :: "<<variable<<" to "<<*constant<<" is INVALID."<<std::endl);
            
            assignment_detail.erase(variable);
            continue;
        }
        
        if(false == std::tr1::get<1>(_execution_precondition)){
            VERBOSER(3001, "For observation :: "<<observation_Name<<std::endl
                     <<"Assignment of :: "<<variable<<" to "<<*constant<<" is INVALID."<<std::endl);
            
            assignment_detail.erase(variable);
            continue;
        }
        
        INTERACTIVE_VERBOSER(true, 8000, "For observation :: "<<observation_Name<<std::endl
                 <<"Assignment of :: "<<*constant<<" to "<<variable<<" is VALID."<<std::endl);
        
        
        auto precondition = std::tr1::get<0>(_precondition);
        auto execution_precondition = std::tr1::get<0>(_execution_precondition);
        
        INTERACTIVE_VERBOSER(true, 8000, "For observation :: "<<observation_Name<<std::endl
                 <<"we got a new precondition :: "<<precondition<<std::endl
                 <<"we got a new execution precondition :: "<<execution_precondition<<std::endl);
        
        ground_observation_schema(observation_Name,
                             effect_formula,
                             assignment_detail,
                             potential_assignments,
                             observation_variables,
                             precondition,
                             execution_precondition,
                             order_in_which_to_make_assignments,
                             variable_index + 1);
        
        assignment_detail.erase(variable);
    }  
}


/* --2-- */
void
Problem_Grounding::ground_observation_schema(const Observation_Name& observation_Name,
                                             Planning::Formula::Subformula& effect,
                                             Planning::Assignment& assignment_detail,
                                             const std::map<Planning::Variable, Planning::Constants/*FIX*/>& potential_assignments,
                                             const Argument_List& observation_variables,
                                             Planning::Formula::Subformula precondition,
                                             Planning::Formula::Subformula execution_precondition)
{
    
    INTERACTIVE_VERBOSER(true, 8000, "Grounding at level --2-- :: "<<observation_Name<<std::endl);
    
    
    Planning_Formula__to__Variable_Ordering planning_Formula__to__Variable_Ordering(*domain_Data);
    planning_Formula__to__Variable_Ordering(precondition);
    std::vector<Variable> order_in_which_to_make_assignments = planning_Formula__to__Variable_Ordering.get__answer();

    QUERY_WARNING(0 == order_in_which_to_make_assignments.size(),
                  "For observation ::"<<observation_Name<<" we could not show a preference in what"<<std::endl
                  <<" order to make assignments to argument variables for grounding...");
    
    INTERACTIVE_VERBOSER(true, 8000, "Will be making assignments in the following order :: "<<order_in_which_to_make_assignments<<std::endl);
    
    
    std::set<Variable> variables_that_need_assignment;
    for(auto argument = observation_variables.begin()
            ; argument != observation_variables.end()
            ; argument++){
        if((*argument).test_cast<Variable>()){
            variables_that_need_assignment.insert(*(*argument).cxx_get<Variable>());
        }
    }

    
    INTERACTIVE_VERBOSER(true, 8000, "All variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
    
    for(auto variable = order_in_which_to_make_assignments.begin()
            ; variable != order_in_which_to_make_assignments.end()
            ; variable++){
        assert(variables_that_need_assignment.find(*variable) != variables_that_need_assignment.end());
        variables_that_need_assignment.erase(*variable);
    }

    INTERACTIVE_VERBOSER(true, 8000, "Unconsidered variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
    if(variables_that_need_assignment.size()){
        for(auto variable = variables_that_need_assignment.begin()
                ; variable != variables_that_need_assignment.end()
                ; variable++){

            WARNING("Could not make a preference for when to make an assignment to :: "<<*variable<<std::endl
                    <<"during observation grounding.");
            
            order_in_which_to_make_assignments.push_back(*variable);
        }
    }
    
    
    ground_observation_schema(observation_Name,
                              effect,
                              assignment_detail,
                              potential_assignments,
                              observation_variables, 
                              precondition,
                              execution_precondition,
                              order_in_which_to_make_assignments,
                              0);
    
}

/* --1-- */
void Problem_Grounding::ground_observation_schema(Planning::Observation_Schema& observation_Schema)
{
    
    /* First step, we alter the observational schema's preconditions
     * formula so that it corresponds to propositional CNF.*/
    simplify_observation_schema_preconditions(observation_Schema);

    auto observation_header = observation_Schema.get__header();
    auto observation_Name = observation_header.get__name();
    auto arguments = observation_header.get__arguments();
    auto observation_Arguments = get__symbols(arguments);
    auto argument_Types = get__types(arguments);

    INTERACTIVE_VERBOSER(true, 8000, "Grounding schema :: "<<observation_Schema<<std::endl);
    
    std::map<Variable,  Constants> potential_assignments;
    assert(argument_Types .size() == observation_Arguments.size());
    for(uint index = 0; index < argument_Types.size(); index++){
        auto types = argument_Types[index];
        auto _variable = observation_Arguments[index];

        if(_variable.test_cast<Planning::Variable>()){
            const Planning::Variable& variable = *_variable.cxx_get<Planning::Variable>();
            
            if(types.size() == 1){
                
                QUERY_UNRECOVERABLE_ERROR(extensions_of_types.find(*types.begin()) == extensions_of_types.end(),
                                          "Could not find object of type :: "<<*types.begin()<<" when computing"<<std::endl
                                          <<"possible assignment for :: "<<variable<<std::endl);

                QUERY_WARNING(!extensions_of_types.find(*types.begin())->second.size(),
                              "Could not find objects of type :: "<<*types.begin()<<" when computing"<<std::endl
                              <<"possible assignment for :: "<<variable<<std::endl);
                if(!extensions_of_types.find(*types.begin())->second.size()){/*Making the above query interactive.*/
                    char ch; std::cin>>ch;
                }
                
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

    auto precondition = observation_Schema.get__precondition();
    auto execution_precondition = observation_Schema.get__execution_precondition();
    auto effect = observation_Schema.get__effect();

    
    ground_observation_schema(observation_Name,
                              effect,
                              assignment_detail,
                              potential_assignments,
                              observation_Arguments,
                              precondition,
                              execution_precondition);
}
