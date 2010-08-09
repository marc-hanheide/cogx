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

// #include "planning_formula_to_problem_formula.hh"
#include "planning_formula_to_variable_ordering.hh"
#include "planning_cnf_to_state_cnf.hh"
#include "domain_action_to_problem_action.hh"


#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"


/* Functionality for simplifying CNF formula. */
#include "turnstyle.hh"

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


#define NEW_CONJUNCTION(NAME, INPUT)            \
    NEW_referenced_WRAPPED_deref_visitable_POINTER            \
    (runtime_Thread                                 \
     , Planning::Formula::Conjunction               \
     , NAME                                         \
     , INPUT)                                       \
        
#define NEW_DISJUNCTION(NAME, INPUT)            \
    NEW_referenced_WRAPPED_deref_visitable_POINTER        \
    (runtime_Thread                                 \
     , Planning::Formula::Disjunction               \
     , NAME                                         \
     , INPUT)                                       \
        
    
#define NEW_NEGATION(NAME, INPUT)                       \
    NEW_referenced_WRAPPED_deref_visitable_POINTER      \
    (runtime_Thread                                     \
     , Planning::Formula::Negation                      \
     , NAME                                             \
     , INPUT)                                           \
    

Problem_Grounding::Problem_Grounding(Parsing::Problem_Data& _problem_Data,
                                     CXX__PTR_ANNOTATION(Parsing::Domain_Data) _domain_Data,
                                     const Planning::Constants_Description& constants_Description,
                                     const std::map<Type, Constants>&extensions_of_types)
    :problem_Data(_problem_Data),
     domain_Data(_domain_Data),
     assignment_Applicator(reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
                           , *_domain_Data
                           , _problem_Data),
     constants_Description(constants_Description),
     extensions_of_types(extensions_of_types)
{
    
    INTERACTIVE_VERBOSER(true, 3101, "Problem is at :: "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)<<std::endl
                         <<"Domain is at :: "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(_domain_Data.get())<<std::endl);
    
    assert(_domain_Data->get__action_Schemas().size());
    auto first_action = _domain_Data->get__action_Schemas().begin();
    auto first_actionx_precondition = first_action->get__precondition();

    this->runtime_Thread = first_actionx_precondition->get__runtime_Thread();
}

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


void  Problem_Grounding::ground_derived_predicates()
{
    auto derived_Predicates = domain_Data->get__derived_Predicates();

    for(auto _derived_Predicate = derived_Predicates.begin()
            ; _derived_Predicate != derived_Predicates.end()
            ; _derived_Predicate++){

        auto derived_Predicate = *_derived_Predicate;
        ground_derived_predicate_schema(derived_Predicate);//const_cast<Planning::Derived_Predicate&>(*derived_Predicate));
    }   
}

void  Problem_Grounding::ground_derived_perceptions()
{
    WARNING("Grounding of derived perception predicates is being called, but is redundant.");
    
    auto derived_Percepts = domain_Data->get__derived_Percepts();

    for(auto _derived_Percept = derived_Percepts.begin()
            ; _derived_Percept != derived_Percepts.end()
            ; _derived_Percept++){
        auto derived_Percept = *_derived_Percept;
        ground_derived_percept_schema(derived_Percept);//const_cast<Planning::Derived_Percept&>(*derived_Percept));
    }
}

Subformula Problem_Grounding::simplify_formula(Planning::Formula::Subformula subformula)
{
    /* \module{turnstyle} */
    using namespace Turnstyle;
    typedef CNF::Clause Clause;
    typedef CNF::Problem_Data Problem_Data;


    /* Atoms that formulate the \argument{action_Schema} preconditions*/
    vector<Subformula> atoms;

    /* For each atom, we map it to an integer index.*/
    map<Subformula, uint> atom_id;

    /* First step, we convert the formula into a CNF.*/
    CNF::Problem_Data problem_Data;

    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(subformula);//action_Schema.get__precondition());

    if(precondition_as_cnf.test_cast<Vacuous>()){
        return subformula;
    }

    
    QUERY_UNRECOVERABLE_ERROR(!precondition_as_cnf.test_cast<Conjunction>(),
                              "Converted a formula :: "<<subformula<<std::endl
                              <<"to CNF format, but ended up with something that is not a conjunct.");
    
    /* Make sure that the conversion to CNF just undertaken has worked.*/
    auto _conjunction = precondition_as_cnf.do_cast<Conjunction>();

    /* Get the bubformulae associated with the conjunction. */
    auto conjunction = _conjunction->get__subformulae();
    assert(conjunction.size());
    for(auto __disjunction = conjunction.begin() /* The precondition is
                                                 * not in CNF format,
                                                 * therefore each
                                                 * subformulae must be
                                                 * a disjunction. */
            ; __disjunction != conjunction.end()
            ; __disjunction++){

        QUERY_UNRECOVERABLE_ERROR(
            !((*__disjunction).test_cast<Disjunction>()),
            "Formula :: "<<(*__disjunction)<<std::endl
            <<"is supposed to be an element in a CNF, however it is not a disjunction.\n");
        
//         auto _disjunction = (*__disjunction).do_cast<Disjunction>();
        
        auto disjunction = __disjunction->cxx_get<Disjunction>()->get__subformulae();//_disjunction->get__subformulae();

        Clause clause;
        for(auto _literal = disjunction.begin() /* Every element in
                                                 * the disjunction
                                                 * should be a literal
                                                 * -- positive or
                                                 * negative atom.*/
                ; _literal != disjunction.end()
                ; _literal++){

            bool negation = _literal->test_cast<Negation>();

            /* Get the atom associated with the literal.*/
            auto __atom = ((negation)
                           ?((_literal->do_cast<Negation>())->get__subformula())
                           :(*_literal));

            /* Get the integer we associate with this atom.*/
            auto _atom = atom_id.find(__atom);

            /* If that failed, then make a new integer associated with
             * that atom, and make sure the \type{Subformula}
             * associated with that atom is mapped to this new
             * integer/index in \local{atom_id}.*/
            if(_atom == atom_id.end()){
                auto index = atoms.size();
                atoms.push_back(__atom);
                atom_id[__atom] = index + 1;
                _atom = atom_id.find(__atom);
            }
            
            auto atom = _atom->second;

            assert(atom != 0);
            
            
            if(negation){
                clause.insert(-1 * atom);
            } else {
                clause.insert(atom);
            }

        }
        
        problem_Data.insert(clause);


        VERBOSER(3001, "Pushing clause :: "<<problem_Data<<std::endl);
    }
    

    VERBOSER(3001, "CNF problem data is :: "<<problem_Data<<std::endl);
    
    /* Construction of a CNF does implicit simplification of that CNF
     * data.*/
    CNF cnf(problem_Data);

    
    VERBOSER(3001, "CNF having been simplified is :: "<<cnf<<std::endl);

    
    
    Subformulae conjunctive_data;
    for(auto clause = cnf.problem_Data.begin()
            ; clause != cnf.problem_Data.end()
            ; clause++){

        
        Subformulae disjunctive_data;
        
        for(auto _literal = clause->begin()
                ; _literal != clause->end()
                ; _literal++){
            auto literal = *_literal;

            uint index = abs(literal);
            
            assert(index - 1 >= 0);
            assert(index - 1 < atoms.size());
            
            auto atom = atoms[index - 1];
            
            if(literal < 0){
                NEW_NEGATION(literal, atom);
                disjunctive_data.push_back(literal);
            } else if (literal > 0) {
                disjunctive_data.push_back(atom);
            } else {
                assert(0);
            }
        }
        NEW_DISJUNCTION(new_disjunction, disjunctive_data);
        
        conjunctive_data.push_back(new_disjunction);
    }
    
    NEW_CONJUNCTION(new_conjunction, conjunctive_data);

    
    VERBOSER(3001, "Original formula is :: "<<subformula<<std::endl);
    VERBOSER(3001, "translated formula is :: "<<precondition_as_cnf<<std::endl);
    VERBOSER(3001, "formula after simplification is :: "<<new_conjunction<<std::endl);

    return new_conjunction;
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
                    std::map<Variable, Constant>& assignment_detail,
                    const Argument_List& action_variables
                    )
{
    /*HERE -- TURN CNF into formula with problem grounding references.*/

    VERBOSER(3101, "Pressing action :: "<<action_Name<<std::endl);

    bool statically_executable_action = false;
    
    
    State_Formula::Conjunctive_Normal_Form_Formula__Pointer
        precondition;
    
    if(enum_types::formula_false == _precondition->get__type_name()){
        return ;
    } else if (enum_types::formula_true == _precondition->get__type_name()) {
        statically_executable_action = true;
        State_Formula::List__Disjunctive_Clause list__Disjunctive_Clause;
        NEW_referenced_WRAPPED_deref_POINTER
            (this,
             State_Formula::Conjunctive_Normal_Form_Formula,
             _conjunct,
             list__Disjunctive_Clause);
        
        precondition = _conjunct.cxx_deref_get<State_Formula::Conjunctive_Normal_Form_Formula>();
    } else {
        Planning_CNF__to__State_CNF
            planning_CNF__to__State_CNF
            (reinterpret_cast<basic_type::Runtime_Thread>(this)
             , state_Propositions
             , literals
             , disjunctive_Clauses
             , conjunctive_Normal_Form_Formulae);
    

        planning_CNF__to__State_CNF(_precondition);

        precondition = planning_CNF__to__State_CNF.get__answer();
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
    

    /**/
    Domain_Action__to__Problem_Action
        domain_Action__to__Problem_Action(reinterpret_cast<basic_type::Runtime_Thread>(this),
                                          assignment_detail,
                                          state_Propositions,
                                          literals,
                                          disjunctive_Clauses,
                                          conjunctive_Normal_Form_Formulae,
                                          *domain_Data,
                                          problem_Data,
                                          action_Proposition,
                                          precondition,
                                          deterministic_actions,
                                          executable_actions_without_preconditions,
                                          probabilistic_actions);
}


/* --3-- */
void Problem_Grounding::
ground_action_schema(const Action_Name& action_Name,
                     Subformula& effect_formula,
                     map<Variable, Constant>& assignment_detail, /*explicit representation of results*/
                     const map<Variable, Constants>& potential_assignments, /* constants from which the result is formed.*/
                     const Argument_List& action_variables, /*Gives the order in which variables assignment should be made -- Some of these may be constant.*/
                     Subformula __precondition,
                     const std::vector<Variable>& variables_in_order,
                     uint variable_index
 )
{
    INTERACTIVE_VERBOSER(true, 3101, "Grounding at level --3-- :: "<<action_Name<<std::endl);
    
    if(variable_index >= variables_in_order.size()){
        INTERACTIVE_VERBOSER(true, 3101, "Pressing at level --3-- :: "<<action_Name<<std::endl);
        
        press_ground_action(action_Name,
                            __precondition,
                            effect_formula,
                            assignment_detail,
                            action_variables);
        return;
    }
    
    INTERACTIVE_VERBOSER(true, 3101, "Proceeding to ground formulae at level --3-- :: "<<action_Name<<std::endl);
        
    
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
        assignment_detail[variable] = *constant;

        
        INTERACTIVE_VERBOSER(true, 3101, "Applying assignment applicator for action :: "<<action_Name<<std::endl
                             <<"trying assignment of :: "<<variable<<" to "<<*constant);
        
        auto _precondition = assignment_Applicator(__precondition, assignment_detail);

        
        if(false == std::tr1::get<1>(_precondition)){
            VERBOSER(3001, "For action :: "<<action_Name<<std::endl
                     <<"Assignment of :: "<<variable<<" to "<<*constant<<" is INVALID."<<std::endl);
            
            continue;
        }
        
        INTERACTIVE_VERBOSER(true, 3101, "For action :: "<<action_Name<<std::endl
                 <<"Assignment of :: "<<*constant<<" to "<<variable<<" is VALID."<<std::endl);
        
        
        auto precondition = std::tr1::get<0>(_precondition);
        
        INTERACTIVE_VERBOSER(true, 3101, "For action :: "<<action_Name<<std::endl
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
                     map<Variable, Constant>& assignment_detail, /*explicit representation of results*/
                     const map<Variable, Constants>& potential_assignments, /* constants from which the result is formed.*/
                     const Argument_List& action_variables, /*Gives the order in which variables assignment should be made -- Some of these may be constant.*/
                     Subformula precondition
 )
{
    INTERACTIVE_VERBOSER(true, 3101, "Grounding at level --2-- :: "<<action_Name<<std::endl);
    
    
    Planning_Formula__to__Variable_Ordering planning_Formula__to__Variable_Ordering(*domain_Data);
    planning_Formula__to__Variable_Ordering(precondition);
    std::vector<Variable> order_in_which_to_make_assignments = planning_Formula__to__Variable_Ordering.get__answer();

    QUERY_WARNING(0 == order_in_which_to_make_assignments.size(),
                  "For action ::"<<action_Name<<" we could not show a preference in what"<<std::endl
                  <<" order to make assignments to argument variables for grounding...");
    
    INTERACTIVE_VERBOSER(true, 3101, "Will be making assignments in the following order :: "<<order_in_which_to_make_assignments<<std::endl);
    
    
    std::set<Variable> variables_that_need_assignment;
    for(auto argument = action_variables.begin()
            ; argument != action_variables.end()
            ; argument++){
        if((*argument).test_cast<Variable>()){
            variables_that_need_assignment.insert(*(*argument).cxx_get<Variable>());
        }
    }

    
    INTERACTIVE_VERBOSER(true, 3101, "All variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
    
    for(auto variable = order_in_which_to_make_assignments.begin()
            ; variable != order_in_which_to_make_assignments.end()
            ; variable++){
        assert(variables_that_need_assignment.find(*variable) != variables_that_need_assignment.end());
        variables_that_need_assignment.erase(*variable);
    }

    INTERACTIVE_VERBOSER(true, 3101, "Unconsidered variables that need assignment :: "<<variables_that_need_assignment<<std::endl);
    
    
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

    INTERACTIVE_VERBOSER(true, 3101, "Grounding at level --1-- :: "<<action_Schema<<std::endl);
    

    
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

    std::map<Variable, Constant> assignment_detail;

    assert(reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data)
           != reinterpret_cast<basic_type::Runtime_Thread>(dynamic_cast<Planning::Parsing::Formula_Data*>(&problem_Data)));
    
//     Planning_Formula__to__Problem_Formula
//         planning_Formula__to__Problem_Formula(reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data),
//                                               problem_Data);

    
    
//     auto precondition = planning_Formula__to__Problem_Formula(action_Schema.get__precondition());

    
//     VERBOSER(3001, "Old precondition was :: "<<precondition<<std::endl
//              <<"New precondition is :: "<<action_Schema.get__precondition()<<std::endl);
    
    auto precondition = action_Schema.get__precondition();
    auto effect = action_Schema.get__effect();
    
//     const Action_Name& _action_Name = action_Name; 
//     Planning::Formula::Subformula& _effect_formula = effect;//action_Schema.get__effect(); 
//     std::map<Variable, Constant>& _assignment_detail = assignment_detail; 
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


void Problem_Grounding::grow__cached_constants_of_types(const Types& types)
{
    Constants constants;
    
    for(auto type = types.begin()
            ; type != types.end()
            ; type++){
        auto consts = extensions_of_types.find(*type)->second;
        for(auto c = consts.begin()
                ; c != consts.end()
                ; c++){
            constants.insert(*c);
        }        
    }

    cached_constants_of_types[types] = std::move<>(constants);
}

void Problem_Grounding::grow__cached_constants_of_types(const Argument_Types& argument_Types)
{
    for(auto arg_Types = argument_Types.begin()
            ; arg_Types != argument_Types.end()
            ; arg_Types++){
        if(arg_Types->size() == 1) continue;
        assert(0 != arg_Types->size());
        if(cached_constants_of_types.find(*arg_Types) == cached_constants_of_types.end()){
            grow__cached_constants_of_types(*arg_Types);
        }
    }
}


void Problem_Grounding::simplify_derived_predicate_trigger(Planning::Derived_Predicate& derived_Predicate)
{
    
    auto new_conjunction = simplify_formula(derived_Predicate.get__formula());
    derived_Predicate.alter__formula(new_conjunction);
}

void Problem_Grounding::ground_derived_predicate_schema(Planning::Derived_Predicate& derived_Predicate)
{
    simplify_derived_predicate_trigger(derived_Predicate);
}


void Problem_Grounding::simplify_derived_percept_trigger(Planning::Derived_Percept& derived_Percept)
{
    
    auto new_conjunction = simplify_formula(derived_Percept.get__formula());
    derived_Percept.alter__formula(new_conjunction);
}

void Problem_Grounding::ground_derived_percept_schema(Planning::Derived_Percept& derived_Percept)
{
   
    simplify_derived_percept_trigger(derived_Percept); 
}
