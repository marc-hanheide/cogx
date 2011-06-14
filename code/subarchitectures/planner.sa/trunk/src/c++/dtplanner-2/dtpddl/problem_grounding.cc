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
// #include "planning_formula_to_variable_ordering.hh"
// #include "domain_action_to_problem_action.hh"
// #include "domain_observation_to_problem_observation.hh"

// #include "planning_cnf_to_state_cnf.hh"
// #include "planning_cnf_to_action_cnf.hh"

// #include "state_formula__literal.hh"
// #include "state_formula__disjunctive_clause.hh"
// #include "state_formula__conjunctive_normal_form_formula.hh"

// #include "action__literal.hh"
// #include "action__disjunctive_clause.hh"
// #include "action__conjunctive_normal_form_formula.hh"

// #include "observation.hh"
// #include "observation__probabilistic.hh"


// #include "action__state_transformation.hh"
// #include "action__probabilistic_state_transformation.hh"

#include "planning_state.hh"


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
                                     const std::map<Type, Constants>& extensions_of_types)
    :problem_Data(_problem_Data),
     domain_Data(_domain_Data),
     constants_Description(constants_Description),
     extensions_of_types(extensions_of_types),
     actions_validator(reinterpret_cast<basic_type::Runtime_Thread>(this), 0),
     assignment_Applicator(reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
          , *_domain_Data
          , _problem_Data
          , actions_validator)
{

//     actions_validator = std::pair<basic_type::Runtime_Thread, ID_TYPE>
//         (reinterpret_cast<basic_type::Runtime_Thread>(this), 0);
    
//      assignment_Applicator =
//          CNF_Assignment_Applicator
//          (reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
//           , *_domain_Data
//           , _problem_Data
//           , actions_validator);
     
    INTERACTIVE_VERBOSER(true, 10003, "Grounding is at :: "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(this)<<std::endl
                         <<"Problem is at :: "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)<<std::endl
                         <<"Domain is at :: "
                         <<reinterpret_cast<basic_type::Runtime_Thread>(_domain_Data.get())<<std::endl);
    
    assert(_domain_Data->get__action_Schemas().size());
    auto first_action = _domain_Data->get__action_Schemas().begin();
    auto first_actionx_precondition = first_action->get__precondition();

    this->runtime_Thread = first_actionx_precondition->get__runtime_Thread();


//     negative_literals = CXX__PTR_ANNOTATION(List__Action_Literals)(new List__Action_Literals(0));
}


const Action_Literals& Problem_Grounding::get__action_Literals() const
{
    return action_Literals;
}


const Action_Conjunctive_Normal_Form_Formulae& Problem_Grounding::get__action_Conjunctive_Normal_Form_Formulae() const
{
    return action_Conjunctive_Normal_Form_Formulae;
}

const Action_Disjunctive_Clauses& Problem_Grounding::get__action_Disjunctive_Clauses() const 
{
    return action_Disjunctive_Clauses;
}


const Observations& Problem_Grounding::get__observations() const
{
    return observations;
}
    
const Observations& Problem_Grounding::get__observations_without_preconditions() const
{
    return observations_without_preconditions;
}
    
const Formula::Perceptual_Propositions& Problem_Grounding::get__perceptual_Propositions() const
{
    return perceptual_Propositions;
}
    
void Problem_Grounding::ground_objective_function()
{
    auto objective_function = problem_Data.get__objective_function();

    if(!objective_function.use_count()){/*"potentially" expensive.*/
        WARNING("I have been given a DTP domain that doesn't seem to have a numeric objective.\n"
                <<"I suspect this is because the problem description does not have a:\n"
                <<"(:metric maximize (reward )) element, or equivalent.");
        is_a_numeric_objective = false;
        return;
    }
    
    is_a_numeric_objective = true;
    
    basic_type::Runtime_Thread formula_runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Parsing::Formula_Data*>(&problem_Data));
            
    integer_valued_objective = false;
    double_valued_objective = false;

    
    switch(objective_function->get__type_name()){
        case enum_types::state_ground_function:
        {
            objective_index
                = objective_function.cxx_get<Formula::State_Ground_Function>()
                ->get__id();
            
            if(!Formula::State_Ground_Function::
               ith_exists(formula_runtime_Thread,
                          objective_index)){
                UNRECOVERABLE_ERROR("Got the objective :: "<<objective_function
                                    <<" that is not registered as a state function.");
            }
        }
        break;
        default:
            UNRECOVERABLE_ERROR("Got the objective :: "<<objective_function
                                <<" that is not a state function.");
            break;
    }

    assert(Formula::State_Ground_Function::
           ith_exists
           (formula_runtime_Thread,
            objective_index));
    
    auto function_symbol = Formula::State_Ground_Function::
        make_ith<Formula::State_Ground_Function>
        (formula_runtime_Thread,
         objective_index);

    QUERY_UNRECOVERABLE_ERROR(problem_Data.has_static_value(function_symbol)
                              , "Trying to optimise over a static function symbol.");
    
    INTERACTIVE_VERBOSER(true, 9096, "Objective is :: "
                         <<function_symbol<<std::endl);
    
    if(domain_Data->is_type__double(function_symbol.get__name())){
        double_valued_objective = true;
    } else if (domain_Data->is_type__int(function_symbol.get__name()) || domain_Data->is_type__number(function_symbol.get__name())) {
        integer_valued_objective = true;;
    } 
}

uint Problem_Grounding::get__objective_index() const
{
    return objective_index;
}

double Problem_Grounding::get__objective_value(const State& state) const
{
    if(!is_a_numeric_objective){
        return 0.0;
    }
    
    if(integer_valued_objective){
        auto value = state.get__int(objective_index);
        return static_cast<double>(value);
    }

    if(double_valued_objective){
        auto value = state.get__float(objective_index);
        return value;
    }

    return 0.0;
}

void Problem_Grounding::set__objective_value(State& state, double value) const
{
    INTERACTIVE_VERBOSER(true, 17000, "Setting objective value to :: "<<value<<std::endl);
    
    if(!is_a_numeric_objective){
        return ;
    }
    
    if(integer_valued_objective){
        assert(static_cast<int>(value) <= std::numeric_limits<int>::max());
        state.set__int(objective_index, static_cast<int>(value));
    }

    if(double_valued_objective){
        state.set__float(objective_index, value);
    }

}

void Problem_Grounding::set__objective_value(State& state, int value) const
{
    if(!is_a_numeric_objective){
        return ;
    }
    
    if(integer_valued_objective){
        state.set__int(objective_index, value);
    }

    if(double_valued_objective){
        state.set__float(objective_index, static_cast<double>(value));
    }
}



const State_Transformation__Pointer& Problem_Grounding::
get__executable_starting_states_generator() const
{
    return executable_starting_states_generator;
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

    VERBOSER(10504, "Trying to convert precondition :: "<<subformula);
    
    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(subformula);//action_Schema.get__precondition());

    if(precondition_as_cnf.test_cast<Vacuous>()){
        WARNING("Got a \"vacuous\" precondition.");
        return precondition_as_cnf;//subformula;
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
            
            assert(static_cast<int>(index) - 1 >= 0);
            assert(static_cast<int>(index) - 1 < static_cast<int>(atoms.size()));
            
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

const Formula::State_Propositions& Problem_Grounding::get__state_Propositions() const
{
    return state_Propositions;
}

const Formula::State_Ground_Functions& Problem_Grounding::get__state_Functions() const
{
    return state_Functions;
}

const State_Formula::Literals& Problem_Grounding::get__literals() const
{
    return literals;
}

const State_Formula::Disjunctive_Clauses& Problem_Grounding::get__disjunctive_Clauses() const
{
    return disjunctive_Clauses;
}

const State_Formula::Conjunctive_Normal_Form_Formulae& Problem_Grounding::get__conjunctive_Normal_Form_Formulae() const
{
    return conjunctive_Normal_Form_Formulae;
}

const State_Transformations& Problem_Grounding::get__deterministic_actions() const
{
    return deterministic_actions;
}

const State_Transformations& Problem_Grounding::get__executable_actions_without_preconditions() const
{
    return executable_actions_without_preconditions;
}

const Probabilistic_State_Transformations& Problem_Grounding::get__probabilistic_actions() const
{
    return probabilistic_actions;
}

const std::map<Formula::Action_Proposition
               , State_Transformation__Pointer>&
Problem_Grounding::get__action_symbol__to__state_transformation() const
{
    return action_symbol__to__state_transformation;
}
