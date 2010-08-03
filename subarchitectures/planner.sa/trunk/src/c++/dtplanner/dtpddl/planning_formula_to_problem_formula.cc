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


#include "planning_formula_to_problem_formula.hh"
#include "dtp_pddl_parsing_data_problem.hh"


using namespace Planning;

typedef Planning::Formula::Subformulae Subformulae;
typedef Planning::Formula::Subformula Subformula;

Planning_Formula__to__Problem_Formula::
Planning_Formula__to__Problem_Formula
(basic_type::Runtime_Thread runtime_Thread,
 Planning::Parsing::Problem_Data& _problem_Data)
    :runtime_Thread(runtime_Thread),
     problem_Data(_problem_Data),
     constants_Description(_problem_Data.get__constants_Description())
{
    
}

#define APPLY_TO_GROUP(GROUP_TYPE)                      \
    {                                                   \
        auto tmp = input.cxx_get<GROUP_TYPE>();         \
        auto sub_fs = tmp->get__subformulae();          \
        Subformulae _result;                            \
        for(auto _sub_f = sub_fs.begin()                \
                ; _sub_f != sub_fs.end()                \
                ; _sub_f++){                            \
            auto sub_f = (*this)(*_sub_f);              \
            _result.push_back(sub_f);                   \
        }                                               \
                                                        \
        NEW_referenced_WRAPPED_deref_visitable_POINTER  \
            (runtime_Thread                             \
             , GROUP_TYPE                               \
             , result                                   \
             , _result);                                \
                                                        \
        return result;                                  \
    }                                                   \
        
#define APPLY_TO_SUBFORMULA(GROUP_TYPE)                 \
    {                                                   \
        auto tmp = input.cxx_get<GROUP_TYPE>();         \
        auto _sub_f = tmp->get__subformula();           \
        auto sub_f = (*this)(_sub_f);                   \
                                                        \
        NEW_referenced_WRAPPED_deref_visitable_POINTER  \
            (runtime_Thread                             \
             , GROUP_TYPE                               \
             , result                                   \
             , sub_f);                                  \
                                                        \
        return result;                                  \
    }                                                   \
        
#define APPLY_TO_QUANTIFIED_SUBFORMULA(GROUP_TYPE)      \
    {                                                   \
        auto tmp = input.cxx_get<GROUP_TYPE>();         \
        auto _sub_f = tmp->get__subformula();           \
        auto sub_f = (*this)(_sub_f);                   \
                                                        \
        NEW_referenced_WRAPPED_deref_visitable_POINTER  \
            (runtime_Thread                             \
             , GROUP_TYPE                               \
             , result                                   \
             , tmp->get__variable()                     \
             , tmp->get__variable_type()                \
             , sub_f);                                  \
                                                        \
        return result;                                  \
    }                                                   \
        

Subformula Planning_Formula__to__Problem_Formula::operator()(Subformula input)
{
    switch(input->get__type_name()){
        case enum_types::state_proposition:
        {
            assert(input.test_cast<State_Proposition>());
            return (*this)(Ground_Fact(input));
        }
        break;
        case enum_types::state_predicate:
        {
            assert(input.test_cast<State_Predicate>());
            return (*this)(Fact(input));
        }
        break;
        case enum_types::observational_predicate:
        {
            assert(input.test_cast<Observational_Predicate>());
            return (*this)(Observation(input));
        }
        break;
        case enum_types::observational_proposition:
        {
            assert(input.test_cast<Observational_Proposition>());
            return (*this)(Ground_Observation(input));
        }
        break;
        /* HERE */
        case enum_types::equality_test:
        {
            assert(input.test_cast<Equality_Test>());
            return (*this)(Equality(input));
        }
        case enum_types::increase:
        {
            assert(input.test_cast<Increase>());
            return (*this)(Addition (input));
        }
        break;
        case enum_types::decrease:
        {
            assert(input.test_cast<Decrease>());
            return (*this)( Subtraction(input));
        }
        break;
        case enum_types::assign:
        {
            assert(input.test_cast<Assign>());
            return (*this)(Assignment (input));
        }
        break;
        case enum_types::action_proposition:
        {
            assert(input.test_cast<Action_Proposition>());
            return (*this)(Action_Ground_Fact (input));
        }
        break;
        case enum_types::action_predicate:
        {
            assert(input.test_cast<Action_Predicate>());
            return (*this)(Action_Fact (input));
        }
        break;
        case enum_types::state_ground_function:
        {
            assert(input.test_cast<State_Ground_Function>());
            return (*this)(Ground_Map (input));
        }
        break;
        case enum_types::state_function:
        {
            assert(input.test_cast<State_Function>());
            return (*this)(Map (input));
        }
        break;
        case enum_types::perceptual_ground_function:
        {
            assert(input.test_cast<Perceptual_Ground_Function>());
            return (*this)(Ground_Perceptual_Map (input));
        }
        break;
        case enum_types::perceptual_function:
        {
            assert(input.test_cast<Perceptual_Function>());
            return (*this)(Perceptual_Map (input));
        }
        break;
        case enum_types::constant:
        {
            assert(input.test_cast<Planning::Constant>());
            auto constant = input.cxx_get<Planning::Constant>();
            if(constants_Description.find(*constant) == constants_Description.end()){
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , new_constant
                     , constant->get__name());

                QUERY_UNRECOVERABLE_ERROR(constants_Description.find(*new_constant.cxx_get<Planning::Constant>())
                                          == constants_Description.end(),
                                          "Could not find constant :: "<<new_constant<<std::endl
                                          <<"in problem definition."<<std::endl);
                
                return new_constant;
            } else {
                return input;
            }
        }
        break;
//         case enum_types::variable:
//         {
//         }
//         break;
        case enum_types::negation:
        {
            APPLY_TO_SUBFORMULA(Planning::Formula::Negation);
        }
        break;
        case enum_types::conjunction:
        {
            APPLY_TO_GROUP(Planning::Formula::Conjunction);
        }
        break;
        case enum_types::disjunction:
        {
            APPLY_TO_GROUP(Planning::Formula::Disjunction);
        }
        break;
        case exists:
        {
            APPLY_TO_QUANTIFIED_SUBFORMULA(Planning::Formula::Exists);
        }
        break;
        case forall:
        {
            APPLY_TO_QUANTIFIED_SUBFORMULA(Planning::Formula::Forall);
        }
        break;
        case enum_types::conditional_effect:
        {
            assert(input.test_cast<Planning::Formula::Conditional_Effect>());
            auto conditional_effect = input.cxx_get<Planning::Formula::Conditional_Effect>();
            auto _condition = conditional_effect->get__condition();
            auto _effect = conditional_effect->get__effect();
            auto condition = (*this)(_condition);
            auto effect = (*this)(_effect);

            NEW_referenced_WRAPPED_deref_visitable_POINTER
                (runtime_Thread
                 , Planning::Formula::Conditional_Effect
                 , new_conditional_effect
                 , condition
                 , effect);

            return new_conditional_effect;
        }
        break;
        case enum_types::probabilistic_effect:
        {
            assert(input.test_cast<Planning::Formula::Probabilistic>());
            auto probabilistic_effect = input.cxx_get<Planning::Formula::Probabilistic>();
            auto _effects = probabilistic_effect->get__formulae();
            auto numbers = probabilistic_effect->get__probabilities();

            Subformulae _result;
                         
            for(auto _sub_f = _effects.begin()              
                    ; _sub_f != _effects.end()               
                    ; _sub_f++){                            
                auto sub_f = (*this)(*_sub_f);             
                _result.push_back(sub_f);                  
            }                                              
                              
            
            NEW_referenced_WRAPPED_deref_visitable_POINTER
                (runtime_Thread
                 , Planning::Formula::Probabilistic
                 , new_probabilistic_effect
                 , _result
                 , numbers);

            return new_probabilistic_effect;
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Given strange formula to convert to problem formula.");
        }
        break;
    }
    
    return input;
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Fact fact)
{
    auto arguments = fact->get__arguments();
    
    Argument_List argument_List;
    bool got_constant_that_needed_changing = false;
    
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument ++) {
        if(argument->test_cast<Constant>()){
            auto constant =  argument->cxx_get<Constant>();
            
            if(constants_Description.find(*constant) == constants_Description.end()){
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , new_constant
                     , constant->get__name());

                QUERY_UNRECOVERABLE_ERROR(constants_Description.find(*new_constant.cxx_get<Constant>())
                                          == constants_Description.end(),
                                          "Domain predicate :: "<<fact<<std::endl
                                          <<"Talks about a constant :: "<<new_constant<<std::endl
                                          <<" ** Talks about a constant **  :: "<<*constant<<std::endl
                                          <<"But the problem description and domain constants don't mention that symbol.");

                got_constant_that_needed_changing = true;
                argument_List.push_back(new_constant);
                
            } else {
                argument_List.push_back(*argument);
            }
        } else if (argument->test_cast<Variable>()) {
            argument_List.push_back(*argument);
        } else {
            UNRECOVERABLE_ERROR("Got non-constant and non-variable argument in predicate symbol.");
        }
    }

    if(!got_constant_that_needed_changing){
        return Subformula(fact);
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread
             , Planning::Formula::State_Predicate
             , new_predicate
             , fact->get__name()
             ,  argument_List);

        return new_predicate;
    }
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Ground_Fact ground_Fact)
{
    auto arguments = ground_Fact->get__arguments();
    
    Constant_Arguments argument_List;
    bool got_constant_that_needed_changing = false;
    
    for(auto constant = arguments.begin()
            ; constant != arguments.end()
            ; constant ++) {
        
        if(constants_Description.find(*constant) == constants_Description.end()){
            NEW_referenced_WRAPPED
                (runtime_Thread
                 , Planning::Constant
                 , new_constant
                 , constant->get__name());

            QUERY_UNRECOVERABLE_ERROR(constants_Description.find(new_constant)
                                      == constants_Description.end(),
                                      "Domain proposition :: "<<*ground_Fact<<std::endl
                                      <<"Talks about a constant :: "<<new_constant<<std::endl
                                      <<" ** Talks about a constant **  :: "<<*constant<<std::endl
                                      <<"But the problem description and domain constants don't mention that symbol.");

            got_constant_that_needed_changing = true;
            argument_List.push_back(new_constant);
                
        } else {
            argument_List.push_back(*constant);
        }
    }

    if(!got_constant_that_needed_changing){
        return Subformula(ground_Fact);
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread
             , Planning::Formula::State_Proposition
             , new_proposition
             , ground_Fact->get__name()
             ,  argument_List);

        return new_proposition;
    }
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Ground_Observation ground_Observation)
{
    
    auto arguments = ground_Observation->get__arguments();
    
    Constant_Arguments argument_List;
    bool got_constant_that_needed_changing = false;
    
    for(auto constant = arguments.begin()
            ; constant != arguments.end()
            ; constant ++) {
        
        if(constants_Description.find(*constant) == constants_Description.end()){
            NEW_referenced_WRAPPED
                (runtime_Thread
                 , Planning::Constant
                 , new_constant
                 , constant->get__name());

            QUERY_UNRECOVERABLE_ERROR(constants_Description.find(new_constant)
                                      == constants_Description.end(),
                                      "Domain ground obserrvation :: "<<*ground_Observation<<std::endl
                                      <<"Talks about a constant :: "<<new_constant<<std::endl
                                      <<" ** Talks about a constant **  :: "<<*constant<<std::endl
                                      <<"But the problem description and domain constants don't mention that symbol.");

            got_constant_that_needed_changing = true;
            argument_List.push_back(new_constant);
                
        } else {
            argument_List.push_back(*constant);
        }
    }

    if(!got_constant_that_needed_changing){
        return Subformula(ground_Observation);
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread
             , Planning::Formula::Observational_Proposition
             , new_ground_proposition
             , ground_Observation->get__name()
             ,  argument_List);

        return new_ground_proposition;
    }
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Observation observation)
{
   
    auto arguments = observation->get__arguments();
    
    Argument_List argument_List;
    bool got_constant_that_needed_changing = false;
    
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument ++) {
        if(argument->test_cast<Constant>()){
            auto constant =  argument->cxx_get<Constant>();
            
            if(constants_Description.find(*constant) == constants_Description.end()){
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , new_constant
                     , constant->get__name());

                QUERY_UNRECOVERABLE_ERROR(constants_Description.find(*new_constant.cxx_get<Constant>())
                                          == constants_Description.end(),
                                          "Domain observational predicate :: "<<*observation<<std::endl
                                          <<"Talks about a constant :: "<<new_constant<<std::endl
                                          <<" ** Talks about a constant **  :: "<<*constant<<std::endl
                                          <<"But the problem description and domain constants don't mention that symbol.");

                got_constant_that_needed_changing = true;
                argument_List.push_back(new_constant);
                
            } else {
                argument_List.push_back(*argument);
            }
        } else if (argument->test_cast<Variable>()) {
            argument_List.push_back(*argument);
        } else {
            UNRECOVERABLE_ERROR("Got non-constant and non-variable argument in observational predicate symbol.");
        }
    }

    if(!got_constant_that_needed_changing){
        return Subformula(observation);
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread
             , Planning::Formula::Observational_Predicate
             , new_observation
             , observation->get__name()
             ,  argument_List);

        return new_observation;
    } 
}


#define APPLY_TO_FUNCTIONAL(GROUP_TYPE)                         \
    auto _subject = input->get__subject();                      \
    auto _modification = input->get__modification();            \
                                                                \
    auto subject  = this->operator()(_subject);                 \
    auto modification  = this->operator()(_modification);       \
                                                                \
    NEW_referenced_WRAPPED_deref_visitable_POINTER              \
    (runtime_Thread                                             \
     , GROUP_TYPE                                               \
     , new_thing                                                \
     , subject                                                  \
     , modification);                                           \
                                                                \
                                                                \
    return new_thing;                                           \
        

Subformula Planning_Formula__to__Problem_Formula::operator()(Addition input)
{
    APPLY_TO_FUNCTIONAL(Planning::Formula::Increase);
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Subtraction input)
{
    APPLY_TO_FUNCTIONAL(Planning::Formula::Decrease);
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Assignment input)
{
    APPLY_TO_FUNCTIONAL(Planning::Formula::Assign);
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Equality input)
{
    APPLY_TO_FUNCTIONAL(Planning::Formula::Equality_Test);
}




Subformula Planning_Formula__to__Problem_Formula::operator()(Ground_Map)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Map)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Ground_Perceptual_Map)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Perceptual_Map)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}


Subformula Planning_Formula__to__Problem_Formula::operator()(Action_Ground_Fact)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}

Subformula Planning_Formula__to__Problem_Formula::operator()(Action_Fact)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Subformula();
}
