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


#include "domain_observation_to_problem_observation.hh"


#include "planning_cnf_to_state_cnf.hh"

#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"
#include "action__simple_numeric_change.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "action__literal.hh"
#include "action__disjunctive_clause.hh"
#include "action__conjunctive_normal_form_formula.hh"


#include "observation.hh"
#include "observation__probabilistic.hh"

#include "dtp_pddl_parsing_data_domain.hh"
#include "dtp_pddl_parsing_data_problem.hh"

/* Functionality for simplifying CNF formula. */
#include "turnstyle.hh"


using namespace Planning;
using namespace Planning::State_Formula;

/*DONE*/
Are_Doubles_Close Domain_Observation__to__Problem_Observation::are_Doubles_Close(1e-9);

/*DONE*/
Planning_Formula__to__CNF Domain_Observation__to__Problem_Observation::planning_Formula__to__CNF;

/*DONE*/
IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Domain_Observation__to__Problem_Observation,
                                            basic_type)


/*DONE*/
Domain_Observation__to__Problem_Observation::
Domain_Observation__to__Problem_Observation
(basic_type::Runtime_Thread runtime_Thread,
 Planning::Assignment& assignment,
 Formula::State_Propositions& state_Propositions,
 Formula::Perceptual_Propositions& problem__perceptual_Propositions,
 Formula::State_Ground_Functions& state_Ground_Functions,
 State_Formula::Literals& problem__literals,
 State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses,
 State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae,
//  Action_Literals& problem__action_Literals,
//  Action_Disjunctive_Clauses& problem__action_Disjunctive_Clauses,
//  Action_Conjunctive_Normal_Form_Formulae& problem__action_Conjunctive_Normal_Form_Formulae,
 const Parsing::Domain_Data& _domain_Data,
 const Parsing::Problem_Data& _problem_Data,
 const Formula::Observational_Proposition& observational_Proposition,
 State_Formula::Conjunctive_Normal_Form_Formula__Pointer& precondition,
 Action_Conjunctive_Normal_Form_Formula__Pointer& execution_precondition,
 Observations& problem__observations,
 Observations& problem__observations_without_preconditions,
 std::pair<basic_type::Runtime_Thread, ID_TYPE>& actions_validator)
    :runtime_Thread(runtime_Thread),
     assignment(assignment),
     problem__state_Propositions(state_Propositions),
     problem__perceptual_Propositions(problem__perceptual_Propositions),
     problem__state_Functions(state_Ground_Functions),
     problem__literals(problem__literals),
     problem__disjunctive_Clauses(problem__disjunctive_Clauses),
     problem__conjunctive_Normal_Form_Formulae(problem__conjunctive_Normal_Form_Formulae),
//      problem__action_Literals(problem__action_Literals),
//      problem__action_Disjunctive_Clauses(problem__action_Disjunctive_Clauses),
//      problem__action_Conjunctive_Normal_Form_Formulae(problem__action_Conjunctive_Normal_Form_Formulae),
     domain_Data(_domain_Data),
     problem_Data(_problem_Data),
     observational_Proposition(observational_Proposition),
     assignment_Applicator(reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
                           , _domain_Data
                           , _problem_Data
                           , actions_validator),
     problem__observations(problem__observations),
     problem__observations_without_preconditions(problem__observations_without_preconditions),
     processing_negative(false),
     count_of_observations_posted(0),
     level(0),
     probability(1.0),
     last_function_symbol_id(-1),
     last_double_traversed(-1.0)
{

    /* Some actions and transformations have void preconditions. Here
     * we store a single formula to represent that case. */
    State_Formula::List__Disjunctive_Clauses list__Disjunctive_Clauses;
    NEW_referenced_WRAPPED_deref_POINTER
        (&problem_Data,//runtime_Thread,
         State_Formula::Conjunctive_Normal_Form_Formula,
         _conjunct,
         list__Disjunctive_Clauses);

    true_cnf = State_Formula::Conjunctive_Normal_Form_Formula__Pointer(_conjunct);
    
    List__Action_Disjunctive_Clauses list__Action_Disjunctive_Clauses;
    NEW_referenced_WRAPPED_deref_POINTER
        (&problem_Data,//runtime_Thread,
         Action_Conjunctive_Normal_Form_Formula,
         _action_conjunct,
         list__Action_Disjunctive_Clauses);

    true_action_cnf = Action_Conjunctive_Normal_Form_Formula__Pointer(_action_conjunct);
        

    INTERACTIVE_VERBOSER(true, 3110, "Empty conjunct is :: "<<true_cnf);
            
    /* Observations preconditions, for the ground perception that this
     * factory shall build.*/ 
    preconditions.push(precondition); /*(see \argument{precondition})*/
    execution_preconditions.push(execution_precondition); /*(see \argument{execution_precondition})*/
    
    INTERACTIVE_VERBOSER(true, 3110, "Making perception with precondition :: "<<preconditions.top()
                         <<"and execution precondition"<<execution_preconditions.top());    
}

bool Domain_Observation__to__Problem_Observation
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

/*DONE*/
const Planning::Observation__Pointer&  Domain_Observation__to__Problem_Observation
::generate__null_observation(double local_probability) 
{
    NEW_referenced_WRAPPED(runtime_Thread
                           , Planning::Observation_Name
                           , new__observation_name
                           , "no-op");
    
    NEW_referenced_WRAPPED(runtime_Thread
                           , Formula::Observational_Proposition
                           , new__observational_proposition
                           , new__observation_name
                           , Planning::Constant_Arguments());
    
    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread
         , Planning::Observation
         , _observation
         , new__observational_proposition
         , true_cnf
         , true_action_cnf
         , Formula::List__Perceptual_Propositions()
         , false
         , false
         , local_probability
         , 0);
            
    auto problem_observation = problem__observations
        .find(Observation__Pointer(_observation));
    
    if(problem__observations.end() == problem_observation){
        problem__observations.insert(Observation__Pointer(_observation));
        problem_observation = problem__observations
            .find(Observation__Pointer(_observation));
    } else {
        INTERACTIVE_VERBOSER(true, 3510, "Re-using null observation."<<std::endl)
    }
    
    return *problem_observation;
}

/*DONE*/
Planning::Observation__Pointer Domain_Observation__to__Problem_Observation::get__answer() const 
{
    assert(problem__observations.find(result)
           != problem__observations.end() ||
           problem__observations_without_preconditions.find(result)
           != problem__observations_without_preconditions.end());
    
    return result;
}

void  Domain_Observation__to__Problem_Observation::
interpret__as_double_valued_ground_state_function(ID_TYPE function_symbol_id)
{
    //assert(function_symbol_id != -1);
    
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
    
    INTERACTIVE_VERBOSER(true, 18000, "Reading double value for :: "<<function_symbol);
    
    if(problem_Data.has_static_value(function_symbol)){
        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 3510, "Reading double value :: "
                                 <<last_double_traversed);
        }
    } else {
        /*FIX --  this, my test for staticness of functions is not working....*/
        WARNING("Hope that wasn't supposed to be a referenced read to a dynamic"<<std::endl
                <<"double-valued number. If it was, we're stuffed. Function is :: "
                <<function_symbol.get__name()<<std::endl);
        if(domain_Data.is_type__double(function_symbol.get__name())){
            last_double_traversed = problem_Data
                .read__static_value<double>(function_symbol);
            
            INTERACTIVE_VERBOSER(true, 3510, "Reading double value :: "
                                 <<last_double_traversed);
        }    
    }
}

void Domain_Observation__to__Problem_Observation::operator()(const Formula::Subformula& input)
{
    switch(input->get__type_name()){
        case enum_types::number:/*DONE*/
        {
            assert(input.test_cast<Formula::Number>());
            last_double_traversed = input.cxx_get<Formula::Number>()->get__value();
            
            INTERACTIVE_VERBOSER(true, 4110, "Reading a number :: "<<input);
        }
        break;/*DONE*/
        case enum_types::negation:/*DONE*/
        {
            assert(input.test_cast<Planning::Formula::Negation>());
            processing_negative = true;
            deref_VISITATION(Planning::Formula::Negation, input, get__subformula());
            processing_negative = false;
        }
        break;/*DONE*/
        case enum_types::conjunction:/*DONE*/
        {
            assert(input.test_cast<Formula::Conjunction>());
            
            double local_probability = probability;
            
            INTERACTIVE_VERBOSER(true, 5000, "LEVEL ::"<<level
                                 <<" conjunctive formula :: "<<input);
            
            assert(input.test_cast<Planning::Formula::Conjunction>());            

            /* PUSH LISTENERS */
            list__Listeners.push(State_Formula::List__Listeners());
            listeners.push(State_Formula::Listeners());
            
            /* PUSH LITERALS */
            effects_lists.push(Planning::Formula::List__Perceptual_Propositions());
            
            /* PUSH PRECONDITIONS */
            preconditions.push(true_cnf);
            execution_preconditions.push(true_action_cnf);
            level++;
            deref_VISITATIONS(Planning::Formula::Conjunction, input, get__subformulae());
            level--;
            
            /* POP PRECONDITIONS */
            preconditions.pop();
            execution_preconditions.pop();
            
            /* Action effects. Either an add or delete effect. */
            auto& conjunction = effects_lists.top();

            /* Any of he subactions that this might trigger. */
            auto& _list__Listeners = list__Listeners.top();
            
            assert(preconditions.size());
            assert(execution_preconditions.size());
            
            State_Formula::Conjunctive_Normal_Form_Formula__Pointer
                _precondition = preconditions.top();

            Action_Conjunctive_Normal_Form_Formula__Pointer
                _execution_precondition = execution_preconditions.top();

            
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Planning::Observation_Name
                                   , new__observation_name
                                   , "leaf-observation");
            
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Formula::Observational_Proposition
                                   , _new__observational_proposition
                                   , new__observation_name
                                   , Planning::Constant_Arguments());
            
            
            auto new__observational_proposition =
                ((level == 0)/*Primary action?*/
                 ?process__generate_name()
                 :((_list__Listeners.size() == 0)/*No children?*/
                   ?(_new__observational_proposition)
                   :process__generate_name()));
            
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Planning::Observation
                 , _observation
                 , new__observational_proposition
                 , _precondition
                 , _execution_precondition
                 , conjunction
                 , ((0==level)?true:false)
                 , false
                 , local_probability//probability
                 , 0);

            
            auto problem_observation = problem__observations
                .find(Observation__Pointer(_observation));
            
            if(problem__observations.end() == problem_observation){
                problem__observations.insert(Observation__Pointer(_observation));
                problem_observation = problem__observations
                    .find(Observation__Pointer(_observation));
            }
            
            auto observation = *problem_observation;
            
            INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE OBSERVATION:: "<<observation<<std::endl
                                 <<"with  "<<_list__Listeners.size()
                                 <<" listeners to wake on execution"<<std::endl);
            
            result = observation;
            
            for(auto listener = _list__Listeners.begin()
                    ; listener != _list__Listeners.end()
                    ; listener++){

                /* An added listener could still be rejected at this
                 * point, if it was added on a previous run.*/
                if(observation
                   ->add__sleeper(*listener)// add__listener(*listener)
                   ){
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<*listener<<std::endl
                            <<"to transformation :: "<<observation<<std::endl)
                }   
            }

            /*If the precondition is interesting.*/
            if(_precondition->get__disjunctive_clauses().size()){
                auto deref__st = observation.cxx_deref_get<basic_type>();
                if(_precondition->add__listener(deref__st)){
                    INTERACTIVE_VERBOSER(true, 9091, "successfully added listener "<<*deref__st.get()
                                         <<" to :: "
                                         <<_precondition<<std::endl);
                    INTERACTIVE_VERBOSER(true, 3110, "successfully added listener."<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<deref__st<<std::endl
                            <<"to formula :: "<<_precondition<<std::endl)
                }
            }
            /*If the execution precondition is interesting.*/
            if(_execution_precondition->get__disjunctive_clauses().size()){
                auto deref__st = observation.cxx_deref_get<basic_type>();
                if(_execution_precondition->add__listener(deref__st)){
                    INTERACTIVE_VERBOSER(true, 9090, "successfully added listener "<<*deref__st.get()
                                         <<" to :: "
                                         <<_execution_precondition<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<deref__st<<std::endl
                            <<"to formula :: "<<_execution_precondition<<std::endl)
                }
            }

            /*If the preconditions are both uninteresting.*/
            if((level == 0)
               && ( _precondition->get__disjunctive_clauses().size() &&
                    _execution_precondition->get__disjunctive_clauses().size()) ) {
                INTERACTIVE_VERBOSER(true, 6000, " :: zero precondition conjunction :: "
                                     <<input);
                
                problem__observations_without_preconditions.insert(observation);
                assert(problem__observations_without_preconditions.size());
            }
            
            /* POP LITERALS */
            effects_lists.pop();
            
            /* POP LISTENERS */
            list__Listeners.pop();
            listeners.pop();
            
            assert(preconditions.size());
            assert(execution_preconditions.size());

            if(list__Listeners.size()){
                assert(listeners.size());
                auto& set_of_listeners = listeners.top();

                
                
                auto deref__st = observation.cxx_deref_get<basic_type>();
                if(set_of_listeners.find(deref__st) == set_of_listeners.end()){
                    auto& list_of_listeners = list__Listeners.top();

                    auto old_size = list__Listeners.top().size();
                    
                    list_of_listeners.push_back(deref__st);
                    assert(list__Listeners.top().size() != old_size);
                    
                    set_of_listeners.insert(deref__st);
                    
                    INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "
                                         <<observation<<std::endl
                                         <<"requesting to be woken..."<<std::endl); 
                } else {
                    INTERACTIVE_VERBOSER(true, 3110, "::INTERMEDIATE ACTION:: "
                                         <<observation<<std::endl
                                         <<"ALREADY requested to be woken..."<<std::endl);
                }
                
            }
            
            return;
        }
        break;/*DONE*/
        case enum_types::state_ground_function:/*DONE*/
        {
            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
            
            assert(effects_lists.size());
            assert(input.test_cast<Planning::Formula::State_Ground_Function>());
            auto _symbol = input.cxx_get<Planning::Formula::State_Ground_Function>();

            INTERACTIVE_VERBOSER(true, 18000, "Reading value of symbol :: "<<_symbol<<std::endl
                                 <<" with id :: "<<_symbol->get__id());
    
            /* Assert that the function has a double type. */
            assert(domain_Data.is_type__double(_symbol->get__name()));
            
            /*In case the symbol is a reference to a number.*/
            interpret__as_double_valued_ground_state_function(_symbol->get__id());/*DONE*/
            
            /*And more generally, if the symbol is a number that characterises states.*/
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Formula::State_Ground_Function,
                 symbol,
                 _symbol->get__name(),
                 _symbol->get__arguments());


            if(problem__state_Functions.find(*symbol.cxx_get<Planning::Formula::State_Ground_Function>())
               == problem__state_Functions.end()){/*FIX : There should not need to be a test here.*/
                problem__state_Functions
                    .insert(*symbol.cxx_get<Planning::Formula::State_Ground_Function>());
            }

            last_function_symbol_id = symbol->get__id();            
        }
        break;/*DONE*/
        case enum_types::state_function:/*DONE*/
        {
            assert(effects_lists.size());
            
            assert(input.test_cast<Planning::Formula::State_Function>());
            
            auto symbol = input.cxx_get<Formula::State_Function>();

            auto argument_List = symbol->get__arguments();
            auto predicate_Name = symbol->get__name();
            
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
                    auto constant = (argument_List[index].cxx_get<Planning::Constant>());

                    auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
                    if(problem_thread != constant->get__runtime_Thread()){
                        NEW_referenced_WRAPPED_deref_visitable_POINTER
                            (problem_thread
                             , Planning::Constant
                             , _constant
                             , constant->get__name());
                        constant = _constant.cxx_get<Planning::Constant>();
                    }
                    
                    constant_Arguments[index] = *constant;
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
        break;/*DONE*/
        case enum_types::perceptual_proposition:/*DONE*/
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
            
            assert(input.test_cast<Planning::Formula::Perceptual_Proposition>());
            
            auto _proposition = input.cxx_get<Planning::Formula::Perceptual_Proposition>();


            Constant_Arguments constant_Arguments;
            auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
            bool no_spurious_constants = true;
            auto arguments = _proposition->get__arguments();
            for(auto argument = arguments.begin()
                    ; argument != arguments.end()
                    ; argument++ ){
                if(argument->get__runtime_Thread() != problem_thread) {
                    no_spurious_constants = false;
                    break;
                }
            }

            if(!no_spurious_constants){
                constant_Arguments = Constant_Arguments(arguments.size());
                assert(arguments.size() == constant_Arguments.size());
                for(uint index = 0
                        ; index < constant_Arguments.size()
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
            Formula::Perceptual_Proposition__Pointer local_proposition;
            
            if(!no_spurious_constants){
                assert(problem_thread == reinterpret_cast<basic_type::Runtime_Thread>
                       (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
        
//                 NEW_referenced_WRAPPED
//                     (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
//                      , Planning::Formula::Perceptual_Proposition
//                      , proposition
//                      , _proposition->get__name()
//                      , constant_Arguments);

                NEW_referenced_WRAPPED_deref_POINTER
                    (runtime_Thread
                     , Formula::Perceptual_Proposition
                     , proposition
                     , _proposition->get__name()
                     , constant_Arguments);
                
                problem__perceptual_Propositions
                    .insert(Formula::Perceptual_Proposition__Pointer(proposition));//.cxx_get<Formula::Perceptual_Proposition>());
                id = proposition->get__id();
                local_proposition = Formula::Perceptual_Proposition__Pointer(proposition);


        
#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 10000
                /*GARDED*/if(id >= problem__perceptual_Propositions.size()){
                /*GARDED*/    for(auto prop = problem__perceptual_Propositions.begin()
                /*GARDED*/            ; prop != problem__perceptual_Propositions.end()
                /*GARDED*/            ; prop++){
                /*GARDED*/        std::cerr<<*prop<<std::endl;
                /*GARDED*/    }
                /*GARDED*/
                /*GARDED*/        std::cerr<<std::endl<<std::endl;
                /*GARDED*/    for(auto i = 0; ; i++){
                    /*GARDED*/
                    /*GARDED*/        if(!Formula::Perceptual_Proposition::
                /*GARDED*/           ith_exists(runtime_Thread, i)){
                /*GARDED*/            break;
                /*GARDED*/        }
                        
                /*GARDED*/        auto symbol = Formula::Perceptual_Proposition::
                /*GARDED*/            make_ith<Formula::Perceptual_Proposition>
                /*GARDED*/            (runtime_Thread,
                /*GARDED*/             i);
                /*GARDED*/        std::cerr<<symbol<<"; "<<std::endl;
                /*GARDED*/    }
                    
                /*GARDED*/}
#endif 
#endif 
#endif 
                
                QUERY_WARNING(id >= problem__perceptual_Propositions.size(),
                                          proposition<<" with ID :: "<<id<<std::endl
                                          <<"Was not registered with the solver.");

                
            } else {
                
//                 NEW_referenced_WRAPPED
//                     (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
//                      , Planning::Formula::Perceptual_Proposition
//                      , proposition
//                      , _proposition->get__name()
//                      , arguments);

                NEW_referenced_WRAPPED_deref_POINTER
                    (runtime_Thread
                     , Formula::Perceptual_Proposition
                     , proposition
                     , _proposition->get__name()
                     , arguments);
                
                problem__perceptual_Propositions
                    .insert(Formula::Perceptual_Proposition__Pointer(proposition));
                
                id = proposition->get__id();
                local_proposition = Formula::Perceptual_Proposition__Pointer(proposition);


             
//                 if(id >= problem__perceptual_Propositions.size()){
//                         std::cerr<<std::endl<<std::endl;
//                     for(auto prop = problem__perceptual_Propositions.begin()
//                             ; prop != problem__perceptual_Propositions.end()
//                             ; prop++){
//                         std::cerr<<*prop<<std::endl;
//                     }

//                         std::cerr<<std::endl<<std::endl;
//                     for(auto i = 0; ; i++){

//                         if(!Formula::Perceptual_Proposition::
//                            ith_exists(runtime_Thread, i)){
//                             break;
//                         }
                        
//                         auto symbol = Formula::Perceptual_Proposition::
//                             make_ith<Formula::Perceptual_Proposition>
//                             (runtime_Thread,
//                              i);
//                         std::cerr<<symbol<<"; "<<std::endl;
//                     }
                    
//                 }   

#ifndef NDEBUG 
#ifdef  DEBUG_LEVEL
#if DEBUG_LEVEL < 10000
                /*GARDED*/if(id >= problem__perceptual_Propositions.size()){
                /*GARDED*/    for(auto prop = problem__perceptual_Propositions.begin()
                /*GARDED*/            ; prop != problem__perceptual_Propositions.end()
                /*GARDED*/            ; prop++){
                /*GARDED*/        std::cerr<<*prop<<std::endl;
                /*GARDED*/    }
                /*GARDED*/
                /*GARDED*/        std::cerr<<std::endl<<std::endl;
                /*GARDED*/    for(auto i = 0; ; i++){
                    /*GARDED*/
                    /*GARDED*/        if(!Formula::Perceptual_Proposition::
                /*GARDED*/           ith_exists(runtime_Thread, i)){
                /*GARDED*/            break;
                /*GARDED*/        }
                        
                /*GARDED*/        auto symbol = Formula::Perceptual_Proposition::
                /*GARDED*/            make_ith<Formula::Perceptual_Proposition>
                /*GARDED*/            (runtime_Thread,
                /*GARDED*/             i);
                /*GARDED*/        std::cerr<<symbol<<"; "<<std::endl;
                /*GARDED*/    }
                    
                /*GARDED*/}
#endif 
#endif 
#endif 
                
                QUERY_WARNING(id >= problem__perceptual_Propositions.size(),
                              proposition<<" with ID :: "<<id<<std::endl
                              <<"Was not registered with the solver.");
            }
            
//             NEW_referenced_WRAPPED_deref_POINTER
//                 (runtime_Thread,
//                  Formula::Perceptual_Proposition,
//                  proposition,
//                  _proposition->get__name(),
//                  _proposition->get__arguments());

//             problem__perceptual_Propositions
//                 .insert(Formula::Perceptual_Proposition__Pointer(proposition));
            
//             auto id = proposition->get__id();

//             assert(id < problem__perceptual_Propositions.size());
            
            effects_lists.top().push_back(local_proposition);//Formula::Perceptual_Proposition__Pointer(proposition));
            
            return;
        }
        break;/*DONE*/
        case enum_types::perceptual_predicate:/*DONE*/
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
            assert(input.test_cast<Formula::Perceptual_Predicate>());
            auto predicate = input.cxx_get<Formula::Perceptual_Predicate>();

            auto argument_List = predicate->get__arguments();
            auto predicate_Name = predicate->get__name();
            
            Constant_Arguments constant_Arguments(argument_List.size());
            for(auto index = 0; index < argument_List.size(); index++){
                if(argument_List[index].test_cast<Planning::Variable>()){
                    auto variable = *(argument_List[index].cxx_get<Planning::Variable>());

                    assert(assignment.find(variable) != assignment.end());
                    auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
                    assert(assignment.find(variable)->second.get__runtime_Thread() == problem_thread);
                    
                    constant_Arguments[index] = assignment.find(variable)->second;
                } else {
                    assert(argument_List[index].test_cast<Planning::Constant>());
                    auto constant = (argument_List[index].cxx_get<Planning::Constant>());

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
                 Planning::Formula::Perceptual_Proposition,
                 proposition,
                 predicate->get__name(),
                 constant_Arguments);

            (*this)(Formula::Subformula(proposition));
            
            return;
        }
        break;/*DONE*/
        case enum_types::probabilistic_effect:/*DONE*/
        {
            assert(input.test_cast<Formula::Probabilistic>());

            if(deal_with_a_missing_conjunctive_parent(input)){
                return;
            }
//             if(level == 0){
//                 assert(!list__Listeners.size());
                
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
            
            
            assert(level != 0);
            auto new__observational_proposition = process__generate_name(":probabilistic:");

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
                    
                    
                    INTERACTIVE_VERBOSER(true, 4110, "Reading a number :: "<<*probability<<" "
                                         <<((*probability)->get__type_name() == enum_types::number)<<std::endl);
            
                    VISIT(*probability);
                    
                    assert(last_double_traversed >= 0.0);
                    assert(last_double_traversed <= 1.0);
                    
                    probabilities[index] = last_double_traversed;
                    sum_of_effect_probabilities += probabilities[index];
                    
                    INTERACTIVE_VERBOSER(true, 9051, "Adding probability :: "
                                         <<last_double_traversed<<std::endl);
                    
                    last_double_traversed = -1.0;
                    last_function_symbol_id = -1;
                }
            }

            double null_effect__probability = 1.0 - sum_of_effect_probabilities;

            assert(are_Doubles_Close(0.0, null_effect__probability) ||
                   null_effect__probability > 0.0);
            assert(null_effect__probability <= 1.0);
            
            bool has_null_effect = !are_Doubles_Close(0.0, null_effect__probability);

            
            
            if(has_null_effect){
                INTERACTIVE_VERBOSER(true, 9051, "Got a null effect with probability :: "
                                     <<null_effect__probability<<std::endl);
            } else {
                INTERACTIVE_VERBOSER(true, 9051, "Got probabilistic action with no null effect.");
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


                INTERACTIVE_VERBOSER(true, 9052, "Visiting :: "<<*conjunctive_naturex_choice<<std::endl
                                     <<"With probability :: "<<probabilities[index]<<std::endl);
                
                
                probability = probabilities[index];
                VISIT(*conjunctive_naturex_choice);
            }
            probability = 1.0;
            level--;
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , Planning::Probabilistic_Observation
                 , probabilistic_Observation
                 , new__observational_proposition);

            
            /* Any of he subactions that this might trigger. */
            auto& _list__Listeners = list__Listeners.top();
            for(auto listener = _list__Listeners.begin()
                    ; listener != _list__Listeners.end()
                    ; listener++){

                /*An added listener could still be rejected at this
                 * point, if it was added on a previous run.*/
                if(probabilistic_Observation.
                   cxx_get<Planning::Probabilistic_Observation>()
                   ->add__sleeper(*listener)// add__listener(*listener)
                   ){
                    INTERACTIVE_VERBOSER(true, 9053, "successfully added listener :: "<<*listener<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<*listener<<std::endl
                            <<"to transformation :: "<<probabilistic_Observation<<std::endl)
                }
            }
            
            /* POP LISTENERS */
            list__Listeners.pop();
            listeners.pop();

            /* Adding  NULL ACTION :i.e., ("no-op"): as required. */
            if(has_null_effect){
                auto observation = generate__null_observation(null_effect__probability);
                
                auto deref__st = observation.cxx_deref_get<basic_type>();
                if(probabilistic_Observation.
                   cxx_get<Planning::Probabilistic_Observation>()
                   ->add__sleeper(deref__st)// add__listener(deref__st)
                   ){
                    INTERACTIVE_VERBOSER(true, 9053, "successfully added listener :: "<<observation<<std::endl)
                } else {
                    WARNING("Failed adding listener :: "<<observation<<std::endl
                            <<"to transformation :: "<<probabilistic_Observation<<std::endl)
                }
            }

            /* Notify parent to wake me up when I should be executed...*/
            assert(list__Listeners.size());
            assert(listeners.size());
            
            auto& set_of_listeners = listeners.top();    
            auto deref__st = probabilistic_Observation
                .cxx_deref_get<basic_type>();
            
            if(set_of_listeners.find(deref__st) == set_of_listeners.end()){
                auto& list_of_listeners = list__Listeners.top();
                auto old_size = list__Listeners.top().size();
                list_of_listeners.push_back(deref__st);
                assert(list__Listeners.top().size() != old_size);
                    
                set_of_listeners.insert(deref__st);
                    
                INTERACTIVE_VERBOSER(true, 9054, "::INTERMEDIATE OBSERVATION:: "<<deref__st<<std::endl
                                     <<"requesting to be woken..."<<std::endl); 
            } else {
                INTERACTIVE_VERBOSER(true, 9054, "::INTERMEDIATE OBSERVATION:: "<<probabilistic_Observation<<std::endl
                                     <<"ALREADY requested to be woken..."<<std::endl);
            }
            
            return;
        }
        break;/*DONE*/
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
                WARNING("Got case "<<observational_Proposition
                        <<" where conditional effect is statically impossible :: "<<input);
                return;
            }
            auto condition = std::tr1::get<0>(_condition);
            if(enum_types::formula_false == condition->get__type_name()){
                WARNING("Got case "<<observational_Proposition
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
                INTERACTIVE_VERBOSER(true, 3110, "Got case "<<observational_Proposition
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
                
                INTERACTIVE_VERBOSER(true, 3120, "Conditional state transformation with precondition :: ");
                INTERACTIVE_VERBOSER(true, 3120, ""<<cnf_condition);
            }

            level++;
            preconditions.push(cnf_condition);
            execution_preconditions.push(true_action_cnf);
            (*this)(effect);
            level--;
            
            preconditions.pop();
            execution_preconditions.pop();
            return;
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Unable to generate observations from effect formula :: "<<input);
        }
        break;
        
    }
    
    
}

/*DONE*/
Formula::Observational_Proposition
Domain_Observation__to__Problem_Observation::
process__generate_name(std::string annotation) 
{
    std::ostringstream __new_observation_name;
    Planning::Observation_Name old__observation_name = observational_Proposition.get__name();
    Planning::Constant_Arguments arguments = observational_Proposition.get__arguments();
    if(level == 0){
        __new_observation_name<<old__observation_name;
    } else {
        __new_observation_name<<old__observation_name
                         <<"+"<<annotation
                         <<count_of_observations_posted++;
    }
    
    auto _new_observation_name = __new_observation_name.str();
    NEW_referenced_WRAPPED(runtime_Thread
                           , Planning::Observation_Name
                           , new__observation_name
                           , _new_observation_name);
            
    NEW_referenced_WRAPPED(runtime_Thread
                           , Formula::Observational_Proposition
                           , new__observational_proposition
                           , new__observation_name
                           , arguments);

    return std::move<>(new__observational_proposition);
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




Formula::Subformula Domain_Observation__to__Problem_Observation::
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

            assert(index > static_cast<uint>(0));
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
