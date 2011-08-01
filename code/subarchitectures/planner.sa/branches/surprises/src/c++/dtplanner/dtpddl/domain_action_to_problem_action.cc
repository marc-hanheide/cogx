
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

#include "domain_action_to_problem_action.hh"

#include "planning_cnf_to_state_cnf.hh"

#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"
#include "action__simple_numeric_change.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "dtp_pddl_parsing_data_domain.hh"
#include "dtp_pddl_parsing_data_problem.hh"

/* Functionality for simplifying CNF formula. */
#include "turnstyle.hh"

using namespace Planning;
using namespace Planning::State_Formula;

Are_Doubles_Close Domain_Action__to__Problem_Action::are_Doubles_Close(1e-9);

Planning_Formula__to__CNF Domain_Action__to__Problem_Action::planning_Formula__to__CNF;


IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Domain_Action__to__Problem_Action,
                                            basic_type)



Domain_Action__to__Problem_Action::
Domain_Action__to__Problem_Action
(basic_type::Runtime_Thread runtime_Thread,
 Planning::Assignment& assignment,
 Formula::State_Propositions& state_Propositions,
 Formula::State_Ground_Functions& state_Ground_Functions,
 State_Formula::Literals& problem__literals,
 State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses,
 State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae,
 const Planning::Parsing::Domain_Data& _domain_Data,
 const Planning::Parsing::Problem_Data& _problem_Data,
 const Formula::Action_Proposition& action_Proposition,
 State_Formula::Conjunctive_Normal_Form_Formula__Pointer& precondition,
 Planning::State_Transformations& state_Transformations,
 State_Transformations& executable_actions_without_preconditions,
 Probabilistic_State_Transformations& probabilistic_actions,
 std::pair<basic_type::Runtime_Thread, ID_TYPE>& actions_validator,
 std::map<Formula::Action_Proposition
 , State_Transformation__Pointer>& action_symbol__to__state_transformation)
    :runtime_Thread(runtime_Thread),
     assignment(assignment),
     problem__state_Propositions(state_Propositions),
     problem__state_Functions(state_Ground_Functions),
     problem__literals(problem__literals),
     problem__disjunctive_Clauses(problem__disjunctive_Clauses),
     problem__conjunctive_Normal_Form_Formulae(problem__conjunctive_Normal_Form_Formulae),
     domain_Data(_domain_Data),
     problem_Data(_problem_Data),
     action_Proposition(action_Proposition),
     assignment_Applicator(reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
                           , _domain_Data
                           , _problem_Data
                           , actions_validator),
     problem__actions(state_Transformations),
     executable_actions_without_preconditions(executable_actions_without_preconditions),
     probabilistic_actions(probabilistic_actions),
     action_symbol__to__state_transformation(action_symbol__to__state_transformation),
     processing_negative(false),
     count_of_actions_posted(0),
     level(0),
     probability(1.0),
     last_function_symbol_id(-1),
     last_double_traversed(-1.0)
{

    /* Some actions and transformations have void preconditions. Here
     * we store a single formula to represent that case. */
    List__Disjunctive_Clauses list__Disjunctive_Clauses;
    NEW_referenced_WRAPPED_deref_POINTER
        (&problem_Data,//runtime_Thread,
         State_Formula::Conjunctive_Normal_Form_Formula,
         _conjunct,
         list__Disjunctive_Clauses);


    true_cnf = CXX__deref__shared_ptr<State_Formula::Conjunctive_Normal_Form_Formula>(_conjunct);


    INTERACTIVE_VERBOSER(true, 3110, "Empty conjunct is :: "<<true_cnf);
            
    /*Action precondition, for the ground action that this factory shall build.*/
    preconditions.push(precondition); /*(see \argument{precondition})*/
    
    INTERACTIVE_VERBOSER(true, 3110, "Making actions with precondition :: "<<preconditions.top());    
}


bool Domain_Action__to__Problem_Action
::deal_with_a_missing_conjunctive_parent(const Formula::Subformula& input) 
{
    if(!list__Listeners.size()){
                
        Formula::Subformulae elements;
        elements.push_back(input);
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread,
             Formula::Conjunction,
             conjunction,
             elements);
        (*this)(conjunction);
                
        return true;
    }

    return false;
}

const Planning::State_Transformation__Pointer&  Domain_Action__to__Problem_Action
::generate__null_action(double local_probability) 
{
    NEW_referenced_WRAPPED(runtime_Thread
                           , Planning::Action_Name
                           , new__action_name
                           , "no-op");
    
    NEW_referenced_WRAPPED(runtime_Thread
                           , Formula::Action_Proposition
                           , new__action_proposition
                           , new__action_name
                           , Planning::Constant_Arguments());

    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread
         , Planning::State_Transformation
         , _state_Transformation
         , new__action_proposition
         , true_cnf
         , State_Formula::List__Literals()
         , true/*All no-op actions are subactions of a probabilistic effect, and therefore compulsory.*/
         , false
         , local_probability
         , 0);
            
    auto problem_action = problem__actions
        .find(State_Transformation__Pointer(_state_Transformation));
    
    if(problem__actions.end() == problem_action){
        problem__actions.insert(State_Transformation__Pointer(_state_Transformation));
        problem_action = problem__actions
            .find(State_Transformation__Pointer(_state_Transformation));
    } else {
        INTERACTIVE_VERBOSER(true, 3510, "Re-using null action."<<std::endl)
    }
    
    return *problem_action;
}

Planning::State_Transformation__Pointer Domain_Action__to__Problem_Action::get__answer() const 
{
    assert(executable_actions_without_preconditions.find(result)
           != executable_actions_without_preconditions.end() ||
           problem__actions.find(result)
           != problem__actions.end());
    
    return result;
}

void Domain_Action__to__Problem_Action::operator()(const Formula::Subformula& input)
{
    switch(input->get__type_name()){
        case enum_types::number:
        {
            assert(input.test_cast<Formula::Number>());
            last_double_traversed = input.cxx_get<Formula::Number>()->get__value();
            
            INTERACTIVE_VERBOSER(true, 4110, "Reading a number :: "<<input);
        }
        break;
        case enum_types::negation:
        {
            assert(input.test_cast<Planning::Formula::Negation>());
            processing_negative = true;
            deref_VISITATION(Planning::Formula::Negation, input, get__subformula());
            processing_negative = false;
        }
        break;
        case enum_types::conjunction:
        {
            assert(input.test_cast<Formula::Conjunction>());
            
            double local_probability = probability;
            
            INTERACTIVE_VERBOSER(true, 10504, "LEVEL ::"<<level
                                 <<" conjunctive formula :: "<<input);
            

//             if(!input.cxx_get<Planning::Formula::Conjunction>()->get__subformulae().size()){
//                 return;
//             }
            

            /* PUSH LISTENERS */
            list__Listeners.push(State_Formula::List__Listeners());
            listeners.push(State_Formula::Listeners());
            
            /* PUSH LITERALS */
            literals_at_levels.push(State_Formula::List__Literals());
            
            /* PUSH PRECONDITIONS */
            preconditions.push(true_cnf);
            level++;
            deref_VISITATIONS(Planning::Formula::Conjunction, input, get__subformulae());
            level--;
            
            /* POP PRECONDITIONS */
            preconditions.pop();
            
            /* Action effects. Either an add or delete effect. */
            auto& conjunction = literals_at_levels.top();

            /* Any of he subactions that this might trigger. */
            auto& _list__Listeners = list__Listeners.top();
            
            assert(preconditions.size());
            State_Formula::Conjunctive_Normal_Form_Formula__Pointer
                _precondition = preconditions.top();

            
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Planning::Action_Name
                                   , new__action_name
                                   , "leaf-op");
            
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Formula::Action_Proposition
                                   , _new__action_proposition
                                   , new__action_name
                                   , Planning::Constant_Arguments());

            
            auto new__action_proposition =
                ((level == 0)/*Primary action?*/
                 ?process__generate_name()
                 :((_list__Listeners.size() == 0)/*No children?*/
                   ?(_new__action_proposition)
                   :process__generate_name()));
            
            INTERACTIVE_VERBOSER(true, 8000, "1--"
                                 <<new__action_proposition.get__runtime_Thread()
                                 <<"::"<<new__action_proposition);
            
//             INTERACTIVE_VERBOSER(true, 3110, "2--::"<<_precondition);
//             INTERACTIVE_VERBOSER(true, 3110, "3--::"<<conjunction);
//             INTERACTIVE_VERBOSER(true, 3110, "4--::"<<probability);
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Planning::State_Transformation
                 , _state_Transformation
                 , new__action_proposition
                 , _precondition
                 , conjunction
                 , (level == 0)?false:true
                 , false
                 , local_probability//probability
                 , 0);

            
            auto problem_action = problem__actions
                .find(State_Transformation__Pointer(_state_Transformation));
            
            if(problem__actions.end() == problem_action){
                problem__actions.insert(State_Transformation__Pointer(_state_Transformation));
                problem_action = problem__actions
                    .find(State_Transformation__Pointer(_state_Transformation));
            }
            
            auto state_Transformation = *problem_action;
            
            INTERACTIVE_VERBOSER(true, 9092, "::INTERMEDIATE ACTION:: "<<state_Transformation<<std::endl
                                 <<"with  "<<_list__Listeners.size()
                                 <<" listeners to wake on execution"<<std::endl);
            
            result = state_Transformation;
            
            if(0 == level){
                auto symbol_to_transformation
                    = action_symbol__to__state_transformation.find(new__action_proposition);
                if(symbol_to_transformation
                   == action_symbol__to__state_transformation.end()){
                    
                    INTERACTIVE_VERBOSER(true, 8001, "Registration of transformation :: "
                                         <<new__action_proposition<<" "
                                         <<new__action_proposition.get__runtime_Thread()<<std::endl)
                    action_symbol__to__state_transformation[new__action_proposition] = state_Transformation;
                } else {
                    assert(symbol_to_transformation->second == state_Transformation);
                }
            }
            
            for(auto listener = _list__Listeners.begin()
                    ; listener != _list__Listeners.end()
                    ; listener++){

                /*An added listener could still be rejected at this
                 * point, if it was added on a previous run.*/
                if(state_Transformation
                   ->add__sleeper(*listener)/*add__listener(*listener)*/){
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<*listener<<std::endl
                            <<"to transformation :: "<<state_Transformation<<std::endl)
                }
                
            }

            /*If the precondition is interesting.*/
            if(_precondition->get__disjunctive_clauses().size()){
                auto deref__st = state_Transformation.cxx_deref_get<basic_type>();
                if(_precondition->add__listener(deref__st)){
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<deref__st<<std::endl
                            <<"to formula :: "<<_precondition<<std::endl)
                }
            } else if (level == 0) {
                INTERACTIVE_VERBOSER(true, 6000, " :: zero precondition conjunction :: "
                                     <<input);
                
                executable_actions_without_preconditions.insert(state_Transformation);
                assert(executable_actions_without_preconditions.size());
            }
            

            
            /* POP LITERALS */
            literals_at_levels.pop();
            
            /* POP LISTENERS */
            list__Listeners.pop();
            listeners.pop();
            
            assert(preconditions.size());

            if(list__Listeners.size()){/*If this action has a parent.*/
                assert(listeners.size());
                auto& set_of_listeners = listeners.top();

                
                
                auto deref__st = state_Transformation.cxx_deref_get<basic_type>();
                if(set_of_listeners.find(deref__st) == set_of_listeners.end()){
                    auto& list_of_listeners = list__Listeners.top();

                    auto old_size = list__Listeners.top().size();
                    
                    list_of_listeners.push_back(deref__st);
                    assert(list__Listeners.top().size() != old_size);
                    
                    set_of_listeners.insert(deref__st);
                    
                    INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "
                                         <<state_Transformation<<std::endl
                                         <<"requesting to be woken..."<<std::endl); 
                } else {
                    INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "
                                         <<state_Transformation<<std::endl
                                         <<"ALREADY requested to be woken..."<<std::endl);
                }
                
            }
            
            return;
        }
        break;
        case enum_types::state_ground_function:
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
            
            assert(literals_at_levels.size());
            assert(input.test_cast<Planning::Formula::State_Ground_Function>());
            auto _symbol = input.cxx_get<Planning::Formula::State_Ground_Function>();
            
            /*In case the symbol is a reference to a number.*/
            interpret__as_double_valued_ground_state_function(*_symbol.get());//_symbol->get__id());
            
            /*And more generally, if the symbol is a number that characterises states.*/
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Formula::State_Ground_Function,
                 symbol,
                 _symbol->get__name(),
                 _symbol->get__arguments());
            
            problem__state_Functions
                .insert(*symbol.cxx_get<Planning::Formula::State_Ground_Function>());

            last_function_symbol_id = symbol->get__id();            
        }
        break;
        case enum_types::state_function:
        {
            assert(literals_at_levels.size());
            
            assert(input.test_cast<Planning::Formula::State_Function>());
            
            auto symbol = input.cxx_get<Formula::State_Function>();

            auto argument_List = symbol->get__arguments();
            auto predicate_Name = symbol->get__name();
            
            Constant_Arguments constant_Arguments(argument_List.size());
            for(uint index = 0; index < argument_List.size(); index++){
                if(argument_List[index].test_cast<Planning::Variable>()){
                    auto variable = *(argument_List[index].cxx_get<Planning::Variable>());

                    assert(assignment.find(variable) != assignment.end());
                    
                    constant_Arguments[index] = assignment.find(variable)->second;
                } else {
                    assert(argument_List[index].test_cast<Planning::Constant>());
                    auto constant = *(argument_List[index].cxx_get<Planning::Constant>());

                    constant_Arguments[index] = constant;
                }
            }

            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Formula::State_Ground_Function,
                 ground_function,
                 symbol->get__name(),
                 constant_Arguments);

            (*this)(Formula::Subformula(ground_function));
            
            return;
        }
        break;
        case enum_types::state_proposition:
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
            
            
//             /*If this is an effect without a conjunctive parent.*/
//             if(!list__Listeners.size()){
                
//                 Formula::Subformulae elements;
//                 elements.push_back(input);
//                 NEW_referenced_WRAPPED_deref_visitable_POINTER
//                     (runtime_Thread,
//                      Formula::Conjunction,
//                      conjunction,
//                      elements);
//                 (*this)(conjunction);
                
//                 return;
//             }
            
            assert(input.test_cast<Planning::Formula::State_Proposition>());
            
            auto _proposition = input.cxx_get<Planning::Formula::State_Proposition>();
            
            Constant_Arguments constant_Arguments;
            auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
            auto arguments = _proposition->get__arguments();
            bool no_spurious_constants = true;
            for(auto argument = arguments.begin()
                    ; argument != arguments.end()
                    ; argument++ ){
                if(argument->get__runtime_Thread() != problem_thread) {
                    WARNING("Constant :: "<<*argument<<" was not owned by the problem_data."<<std::endl
                            <<"Expecting :: "<<problem_thread<<std::endl
                            <<"But got :: "<<argument->get__runtime_Thread()<<std::endl);
                    no_spurious_constants = false;
                    break;
                }
            }

            if(!no_spurious_constants){
                constant_Arguments = Constant_Arguments(arguments.size());
                assert(arguments.size() == constant_Arguments.size());
                for(uint index = 0
                        ; index != constant_Arguments.size()
                        ; index++ ){
                    assert(index < arguments.size());
                    assert(index < constant_Arguments.size());
            
                    if(arguments[index].get__runtime_Thread() != runtime_Thread) {
                        NEW_referenced_WRAPPED
                            (problem_thread
                             , Planning::Constant
                             , constant
                             , arguments[index].get__name());
                        constant_Arguments[index] = constant;
                    } else {
                        constant_Arguments[index] = arguments[index];
                    }
                }
            }

            std::size_t id = 0;
            if(!no_spurious_constants){
                assert(problem_thread == reinterpret_cast<basic_type::Runtime_Thread>
                       (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
        
                NEW_referenced_WRAPPED
                    (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
                     , Planning::Formula::State_Proposition
                     , proposition
                     , _proposition->get__name()
                     , constant_Arguments);

                
                problem__state_Propositions
                    .insert(proposition);
                id = proposition.get__id();

#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 10000
                /*GARDED*/if(id >= problem__state_Propositions.size()){
                    /*GARDED*/for(auto prop = problem__state_Propositions.begin()
                      /*GARDED*/      ; prop != problem__state_Propositions.end()
                        /*GARDED*/    ; prop++){
                        /*GARDED*/std::cerr<<*prop<<std::endl;
                    /*GARDED*/}

                        /*GARDED*/std::cerr<<std::endl<<std::endl;
                    /*GARDED*/for(auto i = 0; ; i++){

                        /*GARDED*/if(!Formula::State_Proposition::
                         /*GARDED*/  ith_exists(runtime_Thread, i)){
                         /*GARDED*/   break;
                        /*GARDED*/}
                        
                       /*GARDED*/ auto symbol = Formula::State_Proposition::
                          /*GARDED*/  make_ith<Formula::State_Proposition>
                          /*GARDED*/  (runtime_Thread,
                         /*GARDED*/    i);
                        /*GARDED*/std::cerr<<symbol<<"; "<<std::endl;
                    /*GARDED*/}
                    
                /*GARDED*/}
#endif 
#endif 
#endif 
                
                assert(problem__state_Propositions.find(proposition) != problem__state_Propositions.end());
                QUERY_UNRECOVERABLE_ERROR(id >= problem__state_Propositions.size(),
                                          proposition<<" with ID :: "<<id<<std::endl
                                          <<"Was not registered with the solver.");
            } else {
                
                NEW_referenced_WRAPPED
                    (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
                     , Planning::Formula::State_Proposition
                     , proposition
                     , _proposition->get__name()
                     , arguments);

                
                problem__state_Propositions
                    .insert(proposition);
                
                id = proposition.get__id();
                
                assert(problem__state_Propositions.find(proposition) != problem__state_Propositions.end());
                QUERY_UNRECOVERABLE_ERROR(id >= problem__state_Propositions.size(),
                                          proposition<<" with ID :: "<<id<<std::endl
                                          <<"Was not registered with the solver.");
            }
            
//             problem__state_Propositions
//                 .insert(*proposition.cxx_get<Planning::Formula::State_Proposition>());
            
//             auto id = proposition->get__id();

            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 State_Formula::Literal,
                 _literal,
                 id,
                 processing_negative);
            
            auto literal = CXX__deref__shared_ptr<State_Formula::Literal>(_literal);
            auto _literal__pointer = problem__literals.find(literal);
            
            if(_literal__pointer == problem__literals.end()){
                
#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 10000

                /*GARDED*/for(auto tmp_literal = problem__literals.begin()
                /*GARDED*/         ; tmp_literal != problem__literals.end()
                /*GARDED*/        ; tmp_literal++){
                /*GARDED*/    std::cerr<<(*tmp_literal)->get__runtime_Thread()<<"::"<<*tmp_literal<<std::endl;
                /*GARDED*/}
#endif 
#endif 
#endif
                
                INTERACTIVE_VERBOSER(true, 9093, "Adding new problem literal :: "
                                     <<literal->get__runtime_Thread()<<"::"<<literal<<std::endl);
                
                problem__literals.insert(literal);
                _literal__pointer = problem__literals.find(literal);
            }
            auto literal__pointer = *_literal__pointer;
            
            literal__pointer->configure__complement(literal__pointer, problem__literals);
            literals_at_levels.top().push_back(literal__pointer);
            
            return;
        }
        break;
        case enum_types::state_predicate:
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
//             /*If this is an effect without a conjunctive parent.*/
//             if(!list__Listeners.size()){
                
//                 Formula::Subformulae elements;
//                 elements.push_back(input);
//                 NEW_referenced_WRAPPED_deref_visitable_POINTER
//                     (runtime_Thread,
//                      Formula::Conjunction,
//                      conjunction,
//                      elements);
//                 (*this)(conjunction);
                
//                 return;
//             }
            
            /* -- ground it and try again -- */
            assert(input.test_cast<Formula::State_Predicate>());
            auto predicate = input.cxx_get<Formula::State_Predicate>();

            auto argument_List = predicate->get__arguments();
            auto predicate_Name = predicate->get__name();
            
            Constant_Arguments constant_Arguments(argument_List.size());
            for(uint index = 0; index < argument_List.size(); index++){
                if(argument_List[index].test_cast<Planning::Variable>()){
                    auto variable = *(argument_List[index].cxx_get<Planning::Variable>());

                    assert(assignment.find(variable) != assignment.end());
                    
                    auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
                    assert(assignment.find(variable)->second.get__runtime_Thread() == problem_thread);
                    constant_Arguments[index] = assignment.find(variable)->second;
                } else {
                    assert(argument_List[index].test_cast<Planning::Constant>());
                    auto constant = argument_List[index].cxx_get<Planning::Constant>();

                    auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
                    if(problem_thread != constant->get__runtime_Thread()){
                        NEW_referenced_WRAPPED_deref_visitable_POINTER
                            (problem_thread
                             , Planning::Constant
                             , _constant
                             , constant->get__name());
                        constant = _constant.cxx_get<Planning::Constant>();
                    }
                    
                    assert(constant->get__runtime_Thread() == problem_thread);
                    
                    constant_Arguments[index] = *constant;
                }
            }

            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Planning::Formula::State_Proposition,
                 proposition,
                 predicate->get__name(),
                 constant_Arguments);

            (*this)(Formula::Subformula(proposition));
            
            return;
        }
        break;
        case enum_types::probabilistic_effect:
        {
            assert(input.test_cast<Formula::Probabilistic>());
            
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
            
            
            assert(level != 0);
            auto new__action_proposition = process__generate_name(":probabilistic:");

            auto probabilistic_transformation = input.cxx_get<Formula::Probabilistic>();
            auto naturex_choices = probabilistic_transformation->get__formulae();
            auto _probabilities = probabilistic_transformation->get__probabilities();

            /*Sum of effect probabilities.*/
            double sum_of_effect_probabilities = 0.0;
            std::vector<double> probabilities(_probabilities.size());
            {
                uint index = 0;
                for(auto probability = _probabilities.begin()
                        ; probability != _probabilities.end()
                        ; probability++, index++){
                    QUERY_WARNING((*probability)->get__type_name() != enum_types::number,
                                  "Was expecting a number, but got :: "<<*probability);
                    
                    QUERY_WARNING((*probability)->get__type_name() != enum_types::state_ground_function,
                                  "Was expecting ground state function, but got :: "<<*probability);
                    
                    INTERACTIVE_VERBOSER(true, 18000, "Reading a number :: "<<*probability<<" "
                                         <<((*probability)->get__type_name() == enum_types::number)<<" "<<std::endl
                                         <<((*probability)->get__type_name() == enum_types::state_ground_function)<<std::endl);
            
                    VISIT(*probability);
                    
                    assert(last_double_traversed >= 0.0);
                    assert(last_double_traversed <= 1.0);
                    
                    probabilities[index] = last_double_traversed;
                    sum_of_effect_probabilities += probabilities[index];
                    
                    INTERACTIVE_VERBOSER(true, 7001, "Adding probability :: "
                                         <<last_double_traversed<<std::endl);
                    
                    last_double_traversed = -1.0;
                    last_function_symbol_id = -1;
                }
            }

            double null_effect__probability = 1.0 - sum_of_effect_probabilities;
//             if(!input.cxx_get<Formula::Probabilistic>()->sanity()){
//                 double sum = 0.0;
//                 for(auto p = probabilities.begin()
//                         ; p != probabilities.end()
//                         ; p++ ){
//                     sum += *p;
//                     INTERACTIVE_VERBOSER(true, 7001, "One effect with probability :: "
//                                          <<*p<<std::endl)
//                 }
                
//                 null_effect__probability -= sum;
//             }

            assert(are_Doubles_Close(0.0, null_effect__probability) ||
                   null_effect__probability > 0.0);
            assert(null_effect__probability <= 1.0);
            
            bool has_null_effect = !are_Doubles_Close(0.0, null_effect__probability);

            if(has_null_effect){
                INTERACTIVE_VERBOSER(true, 7001, "Got a null effect with probability :: "
                                     <<null_effect__probability<<std::endl);
            } else {
                INTERACTIVE_VERBOSER(true, 7001, "Got probabilistic action with no null effect.");
            }
            
            
            Formula::Subformulae conjunctive_naturex_choices;
            for(auto naturex_choice = naturex_choices.begin()
                    ; naturex_choice != naturex_choices.end()
                    ; naturex_choice++){

                
                auto _conjunctive_naturex_choice = *naturex_choice;
                
                Formula::Subformula conjunctive_naturex_choice;
                if(enum_types::conjunction != _conjunctive_naturex_choice->get__type_name()){
                    Formula::Subformulae elements;
                    elements.push_back(_conjunctive_naturex_choice);
                    NEW_referenced_WRAPPED_deref_visitable_POINTER
                        (runtime_Thread,
                         Formula::Conjunction,
                         conjunction,
                         elements);

                    conjunctive_naturex_choice = conjunction;
                } else {
                    conjunctive_naturex_choice = _conjunctive_naturex_choice;
                }

                conjunctive_naturex_choices.push_back(conjunctive_naturex_choice);
            }
            
            
            /* PUSH LISTENERS */
            list__Listeners.push(State_Formula::List__Listeners());
            listeners.push(State_Formula::Listeners());
            
            level++;
            uint index = 0;
            assert(probabilities.size() == conjunctive_naturex_choices.size());
            for(auto conjunctive_naturex_choice = conjunctive_naturex_choices.begin()
                    ; conjunctive_naturex_choice != conjunctive_naturex_choices.end()
                    ; conjunctive_naturex_choice++, index++){
                probability = probabilities[index];
                VISIT(*conjunctive_naturex_choice);
            }
            probability = 1.0;
            level--;
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Planning::Probabilistic_State_Transformation
                 , probabilistic_state_Transformation
                 , new__action_proposition);

            
            /* Any of the subactions that this might trigger. */
            auto& _list__Listeners = list__Listeners.top();
            for(auto listener = _list__Listeners.begin()
                    ; listener != _list__Listeners.end()
                    ; listener++){

                /*An added listener could still be rejected at this
                 * point, if it was added on a previous run.*/
                if(probabilistic_state_Transformation.
                   cxx_get<Planning::Probabilistic_State_Transformation>()
                   ->add__sleeper(*listener)
//                    ->add__listener(*listener)
                   ){
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<*listener<<std::endl
                            <<"to transformation :: "<<probabilistic_state_Transformation<<std::endl)
                }
            }
            
            /* POP LISTENERS */
            list__Listeners.pop();
            listeners.pop();

            /* Adding  NULL ACTION :i.e., ("no-op"): as required. */
            if(has_null_effect){
                auto action = generate__null_action(null_effect__probability);
                
                auto deref__st = action.cxx_deref_get<basic_type>();
                if(probabilistic_state_Transformation.
                   cxx_get<Planning::Probabilistic_State_Transformation>()
                   ->add__sleeper(deref__st)// add__listener(deref__st)
                   ){
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<action<<std::endl
                            <<"to transformation :: "<<probabilistic_state_Transformation<<std::endl)
                }
            }

            /* Notify parent to wake me up when I should be executed...*/
            assert(list__Listeners.size());
            assert(listeners.size());
            
            auto& set_of_listeners = listeners.top();    
            auto deref__st = probabilistic_state_Transformation
                .cxx_deref_get<basic_type>();
            
            if(set_of_listeners.find(deref__st) == set_of_listeners.end()){
                auto& list_of_listeners = list__Listeners.top();
                auto old_size = list__Listeners.top().size();
                list_of_listeners.push_back(deref__st);
                assert(list__Listeners.top().size() != old_size);
                    
                set_of_listeners.insert(deref__st);
                    
                INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "<<probabilistic_state_Transformation<<std::endl
                                     <<"requesting to be woken..."<<std::endl); 
            } else {
                INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "<<probabilistic_state_Transformation<<std::endl
                                     <<"ALREADY requested to be woken..."<<std::endl);
            }
            
            return;
        }
        break;
        case enum_types::conditional_effect:
        {
            
            assert(input.test_cast<Formula::Conditional_Effect>());
            
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
            
            assert(level != 0);
            
            auto conditional_Effect = input.cxx_get<Formula::Conditional_Effect>();
            auto ___condition =  conditional_Effect->get__condition();
            auto __condition = simplify_formula(___condition, runtime_Thread);
            auto _condition = assignment_Applicator(__condition, assignment);

            /* If this condition is statically not executable.*/
            if(std::tr1::get<1>(_condition) == false){

                /*This could be a warning. But it is untested as such...*/
                UNRECOVERABLE_ERROR("Got case "<<action_Proposition
                                   <<" where conditional effect is statically impossible :: "<<input);
//                 exit(0);
                return;
            } else {
                
                INTERACTIVE_VERBOSER(true, 11000, "Got a conditional element :: "
                                     <<std::tr1::get<0>(_condition)<<std::endl);
            }
            
            
            auto condition = std::tr1::get<0>(_condition);
            if(enum_types::formula_false == condition->get__type_name()){
                /*This could be a warning. But it is untested as such...*/
                UNRECOVERABLE_ERROR("Got case "<<action_Proposition
                                    <<" where conditional effect is statically impossible :: "<<input);
                return ;
            }

            
            auto _effect = conditional_Effect->get__effect();
            Formula::Subformula effect;
            if(enum_types::conjunction != _effect->get__type_name()){
                Formula::Subformulae elements;
                elements.push_back(_effect);
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread,
                     Formula::Conjunction,
                     conjunction,
                     elements);

                effect = conjunction;
            } else {
                effect = _effect;
            }

            
            
            State_Formula::Conjunctive_Normal_Form_Formula__Pointer cnf_condition;
            if (enum_types::formula_true == condition->get__type_name()) {
                INTERACTIVE_VERBOSER(true, 11000, "Got case "<<action_Proposition
                                     <<" where conditional effect is statically satisfied :: "<<input);
                cnf_condition = true_cnf;
            } else {
                Planning_CNF__to__State_CNF
                    planning_CNF__to__State_CNF
                    (runtime_Thread
                     , problem__state_Propositions
                     , problem__literals
                     , problem__disjunctive_Clauses
                     , problem__conjunctive_Normal_Form_Formulae
                     , problem_Data);
                

                planning_CNF__to__State_CNF(condition);

                cnf_condition = planning_CNF__to__State_CNF.get__answer();
                
                INTERACTIVE_VERBOSER(true, 11000, "Conditional state transformation with precondition :: ");
                INTERACTIVE_VERBOSER(true, 11000, ""<<cnf_condition);
            }

            level++;
            preconditions.push(cnf_condition);
            (*this)(effect);
            level--;
            
            preconditions.pop();
            return;
        }
        break;
        case enum_types::increase:
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
//             /*If this is an effect without a conjunctive parent.*/
//             if(!list__Listeners.size()){
                
//                 Formula::Subformulae elements;
//                 elements.push_back(input);
//                 NEW_referenced_WRAPPED_deref_visitable_POINTER
//                     (runtime_Thread,
//                      Formula::Conjunction,
//                      conjunction,
//                      elements);
//                 (*this)(conjunction);
                
//                 return;
//             }
            
            
            assert(input.test_cast<Formula::Increase>());
            auto subject = input.cxx_get<Formula::Increase>()->get__subject();
            deref_VISITATION(Formula::Increase, input, get__subject());
            auto modification = input.cxx_get<Formula::Increase>()->get__modification();
            process__Function_Modifier(modification, input->get__type_name());
        }
        break;
        case enum_types::decrease:
        {
            /*If this is an effect without a conjunctive parent.*/
            if(!list__Listeners.size()){
                
                Formula::Subformulae elements;
                elements.push_back(input);
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread,
                     Formula::Conjunction,
                     conjunction,
                     elements);
                (*this)(conjunction);
                
                return;
            }
            
            assert(input.test_cast<Formula::Decrease>());
            auto subject = input.cxx_get<Formula::Decrease>()->get__subject();
            deref_VISITATION(Formula::Decrease, input, get__subject());
            auto modification = input.cxx_get<Formula::Decrease>()->get__modification();
            process__Function_Modifier(modification, input->get__type_name());
        }
        break;
        case enum_types::assign:
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
//             /*If this is an effect without a conjunctive parent.*/
//             if(!list__Listeners.size()){
                
//                 Formula::Subformulae elements;
//                 elements.push_back(input);
//                 NEW_referenced_WRAPPED_deref_visitable_POINTER
//                     (runtime_Thread,
//                      Formula::Conjunction,
//                      conjunction,
//                      elements);
//                 (*this)(conjunction);
                
//                 return;
//             }

            
            assert(input.test_cast<Formula::Assign>());
            auto subject = input.cxx_get<Formula::Assign>()->get__subject();
            deref_VISITATION(Formula::Assign, input, get__subject());
            auto modification = input.cxx_get<Formula::Assign>()->get__modification();
            process__Function_Modifier(modification, input->get__type_name());
        }
        break;
        case enum_types::vacuous:
        {
            WARNING("VACUOUS effect formula :: "<<input);
            
            Formula::Subformulae elements;
            NEW_referenced_WRAPPED_deref_visitable_POINTER
                (runtime_Thread,
                 Formula::Conjunction,
                 conjunction,
                 elements);
            (*this)(conjunction);
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Unable to generate actions from effect formula :: "<<input);
        }
        break;
        
    }
    
    
}

void  Domain_Action__to__Problem_Action::
interpret__as_double_valued_ground_state_function(Formula::State_Ground_Function& function_symbol)
{
    basic_type::Runtime_Thread formula_runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Parsing::Formula_Data*>(&problem_Data));

    if(problem_Data.has_static_value(function_symbol)){
        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 3510, "Reading double value :: "
                                 <<last_double_traversed);
        }
    } else {
        WARNING("Hope "<<function_symbol<<" is supposed to be a referenced read to a dynamic"<<std::endl
                <<"double-valued number. If it is not, we're stuffed."<<std::endl
                <<domain_Data.is_type__double(function_symbol.get__name())<<std::endl
                <<domain_Data.is_type__int(function_symbol.get__name())<<std::endl
                <<domain_Data.is_type__number(function_symbol.get__name())<<std::endl);
        
        
        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 18000, "Reading double value :: "
                                 <<last_double_traversed);
        }
    }
    
}


void  Domain_Action__to__Problem_Action::
interpret__as_double_valued_ground_state_function(ID_TYPE function_symbol_id)
{
    UNRECOVERABLE_ERROR("Buggy code.");
    
    assert(function_symbol_id != -1);
    
    basic_type::Runtime_Thread formula_runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Parsing::Formula_Data*>(&problem_Data));
    
    if(!Formula::State_Ground_Function::
       ith_exists(formula_runtime_Thread,
                  function_symbol_id)){
        
        return;
    }
    
    auto function_symbol = Formula::State_Ground_Function::
        make_ith<Formula::State_Ground_Function>
        (formula_runtime_Thread,
         function_symbol_id);
    
    INTERACTIVE_VERBOSER(true, 18000, "Reading double value for :: "<<function_symbol<<std::endl
                         <<" with id :: "<<function_symbol_id);
    
    if(problem_Data.has_static_value(function_symbol)){
        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 3510, "Reading double value :: "
                                 <<last_double_traversed);
        }
    } else {
        WARNING("Hope that wasn't supposed to be a referenced read to a dynamic"<<std::endl
                <<"double-valued number. If it was, we're stuffed...");

        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 3510, "Reading double value :: "
                                 <<last_double_traversed);
        }  
    }
}


void Domain_Action__to__Problem_Action::
process__Function_Modifier(Formula::Subformula& modification,
                           ID_TYPE modification_type)
{
    assert(last_function_symbol_id != -1);

    QUERY_UNRECOVERABLE_ERROR(!list__Listeners.size(),
                              "Processing function modifier with modification :: "<<modification<<std::endl
                              <<"However, the modification has no parents.");
    assert(list__Listeners.size());
    assert(listeners.size());
    auto& list_of_listeners = list__Listeners.top();
    auto& set_of_listeners = listeners.top();
    
    auto new__action_proposition = process__generate_name(":numeric:");

    /* id of the subject of the modification. */
    assert(Formula::State_Ground_Function::
           ith_exists(runtime_Thread,
                      last_function_symbol_id));
    
    auto last_function_symbol = Formula::State_Ground_Function::
        make_ith<Formula::State_Ground_Function>
        (runtime_Thread, //dynamic_cast<Planning::Parsing::Formula_Data>(&problem_Data),????
         last_function_symbol_id);
    
    if(problem_Data.has_static_value(modification, assignment)){//last_function_symbol.get__name())){
        if(domain_Data.is_type__double(last_function_symbol.get__name())){
            double value = problem_Data
                .read__static_value<double>(modification, assignment);
            assert(last_function_symbol_id >= 0);
            
            
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Planning::Simple_Double_Transformation
                 , transformation
                 , new__action_proposition
                 , last_function_symbol_id
                 , value
                 , modification_type);

            
            auto deref__st = transformation.cxx_deref_get<basic_type>();
            list_of_listeners.push_back(deref__st);
            set_of_listeners.insert(deref__st);   
            
            INTERACTIVE_VERBOSER(true, 17000, "Got a simple double transformation :: "<<transformation<<std::endl
                                 <<" to function with index :: "<<last_function_symbol_id);
    
        } else if ( domain_Data.is_type__int(last_function_symbol.get__name()) ||
                    domain_Data.is_type__number(last_function_symbol.get__name()) ) {
            
            int value = problem_Data
                .read__static_value<int>(modification, assignment);
            
            assert(last_function_symbol_id >= 0);

            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Simple_Int_Transformation
                 , transformation
                 , new__action_proposition
                 , last_function_symbol_id
                 , value
                 , modification_type);

            CXX__deref__shared_ptr<basic_type> deref__st = transformation;//.cxx_deref_get<basic_type>();
            list_of_listeners.push_back(deref__st);
            set_of_listeners.insert(deref__st);
            
            INTERACTIVE_VERBOSER(true, 17000, "Got a simple int transformation :: "<<transformation<<std::endl
                                 <<" to function with index :: "<<last_function_symbol_id);

            
        } else {
            UNRECOVERABLE_ERROR("No support for non-static function symbols in modification elements....");
        }
    } else {
        UNRECOVERABLE_ERROR("unimplemented");
    }

    last_function_symbol_id = -1;
}

Formula::Action_Proposition Domain_Action__to__Problem_Action::
process__generate_name(std::string annotation) 
{
    std::ostringstream __new_action_name;
    Planning::Action_Name old__action_name = action_Proposition.get__name();
    Planning::Constant_Arguments arguments = action_Proposition.get__arguments();
    if(level == 0){
//         basic_type::Runtime_Thread formula_runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
//             (dynamic_cast<const Parsing::Formula_Data*>(&problem_Data));
            
        __new_action_name<<old__action_name;
        auto _new_action_name = __new_action_name.str();

        
        NEW_referenced_WRAPPED(&domain_Data//formula_runtime_Thread//runtime_Thread
                               , Planning::Action_Name
                               , new__action_name
                               , _new_action_name);
        
        NEW_referenced_WRAPPED(runtime_Thread
                               , Formula::Action_Proposition
                               , new__action_proposition
                               , new__action_name
                               , arguments);
        
        return std::move<>(new__action_proposition);
    } else {
        __new_action_name<<old__action_name
                         <<"+"<<annotation
                         <<count_of_actions_posted++;
        
        auto _new_action_name = __new_action_name.str();
        
        NEW_referenced_WRAPPED(runtime_Thread
                               , Planning::Action_Name
                               , new__action_name
                               , _new_action_name);
        
        NEW_referenced_WRAPPED(runtime_Thread
                               , Formula::Action_Proposition
                               , new__action_proposition
                               , new__action_name
                               , arguments);
        
        return std::move<>(new__action_proposition);
    }
}




#define NEW_CONJUNCTION(NAME, INPUT)                                 \
    NEW_referenced_WRAPPED_deref_visitable_POINTER                   \
    (runtime_Thread                                                  \
     , Formula::Conjunction                                          \
     , NAME                                                          \
     , INPUT)                                                        \
        
#define NEW_DISJUNCTION(NAME, INPUT)                             \
    NEW_referenced_WRAPPED_deref_visitable_POINTER               \
    (runtime_Thread                                              \
     , Formula::Disjunction                                      \
     , NAME                                                      \
     , INPUT)                                                    \
        

#define NEW_NEGATION(NAME, INPUT)                       \
    NEW_referenced_WRAPPED_deref_visitable_POINTER      \
    (runtime_Thread                                     \
     , Formula::Negation                                \
     , NAME                                             \
     , INPUT)                                           \




Formula::Subformula Domain_Action__to__Problem_Action::
simplify_formula(Formula::Subformula subformula,
                 basic_type::Runtime_Thread runtime_Thread)
{
    /* \module{turnstyle} */
    using namespace Turnstyle;
    typedef CNF::Clause Clause;

    /* Atoms that formulate the \argument{action_Schema} preconditions*/
    std::vector<Formula::Subformula> atoms;

    /* For each atom, we map it to an integer index.*/
    std::map<Formula::Subformula, uint> atom_id;

    /* First step, we convert the formula into a CNF.*/
    CNF::Problem_Data problem_Data;

    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(subformula);

    if(precondition_as_cnf.test_cast<Formula::Vacuous>()){
        return subformula;
    }

    
    QUERY_UNRECOVERABLE_ERROR(!precondition_as_cnf.test_cast<Formula::Conjunction>(),
                              "Converted a formula :: "<<subformula<<std::endl
                              <<"to CNF format, but ended up with something that is not a conjunct.");
    
    /* Make sure that the conversion to CNF just undertaken has worked.*/
    auto _conjunction = precondition_as_cnf.do_cast<Formula::Conjunction>();

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
            !((*__disjunction).test_cast<Formula::Disjunction>()),
            "Formula :: "<<(*__disjunction)<<std::endl
            <<"is supposed to be an element in a CNF, however it is not a disjunction.\n");
        
//         auto _disjunction = (*__disjunction).do_cast<Disjunction>();
        
        auto disjunction = __disjunction->cxx_get<Formula::Disjunction>()->get__subformulae();//_disjunction->get__subformulae();

        Clause clause;
        for(auto _literal = disjunction.begin() /* Every element in
                                                 * the disjunction
                                                 * should be a literal
                                                 * -- positive or
                                                 * negative atom.*/
                ; _literal != disjunction.end()
                ; _literal++){

            bool negation = _literal->test_cast<Formula::Negation>();

            /* Get the atom associated with the literal.*/
            auto __atom = ((negation)
                           ?((_literal->do_cast<Formula::Negation>())->get__subformula())
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

    
    
    Formula::Subformulae conjunctive_data;
    for(auto clause = cnf.problem_Data.begin()
            ; clause != cnf.problem_Data.end()
            ; clause++){

        
        Formula::Subformulae disjunctive_data;
        
        for(auto _literal = clause->begin()
                ; _literal != clause->end()
                ; _literal++){
            auto literal = *_literal;

            uint index = abs(literal);
            
            assert(index > 0);
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
