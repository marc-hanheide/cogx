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


#include "planning_formula_to_variable_ordering.hh"

#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning;
using namespace Planning::Formula;

// IMPLEMENTATION__UNARY_VISITOR(Planning_Formula__to__Variable_Ordering,
//                               Subformula);

Planning_Formula__to__Variable_Ordering::
Planning_Formula__to__Variable_Ordering(const Planning::Parsing::Domain_Data& domain_Data)
    :domain_Data(domain_Data),
     processing_negative(false),
     processing_hard_constraint(true)
{
}

IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Planning_Formula__to__Variable_Ordering,
                                            basic_type)


std::vector<Variable> Planning_Formula__to__Variable_Ordering::get__answer()
{
    std::vector<Variable> answer(scores.size());
    
    auto index = 0;
    for(auto score_element = scores_to_variables.begin()
            ; score_element != scores_to_variables.end()
            ; score_element++ ){
        for(auto variable = score_element->second.begin()
                ; variable != score_element->second.end()
                ; variable++){
            answer[index++] = *variable;
        }
    }
    
    return std::move(answer);
}

#define IMPLEMENTATION__Function_Modifier(INPUT)        \
    (*this)(INPUT.get__subject());                      \
    (*this)(INPUT.get__modification());                 \

#define CAST_IMPLEMENTATION__Function_Modifier(NUMBER)                  \
    case NUMBER:                                                        \
    {                                                                   \
        typedef Planning::Formula::Function_Modifier<NUMBER> TYPE;      \
                                                                        \
        assert(input.test_cast<TYPE>());                                \
        IMPLEMENTATION__Function_Modifier(                              \
            (*input.cxx_get<TYPE>()));                                  \
    }                                                                   \
    break                                                               \
    

void Planning_Formula__to__Variable_Ordering::operator()(const Formula::Subformula& input)
{
    switch(input->get__type_name()){
        case enum_types::variable:
        {
            assert(input.test_cast<Planning::Variable>());
            Argument_List argument_List(1);
            argument_List[0] = input;
            (*this)(argument_List, 1);
        }
        break;
        case enum_types::state_predicate:
        {
            assert(input.test_cast<Planning::Formula::State_Predicate>());
            (*this)(*input.cxx_get<Planning::Formula::State_Predicate>());
        }
        break;
        case enum_types::state_function:
        {
            assert(input.test_cast<Planning::Formula::State_Function>());
            (*this)(*input.cxx_get<Planning::Formula::State_Function>());
        }
        break;
        CAST_IMPLEMENTATION__Function_Modifier(enum_types::decrease);
        CAST_IMPLEMENTATION__Function_Modifier(enum_types::increase);
        CAST_IMPLEMENTATION__Function_Modifier(enum_types::assign);
        CAST_IMPLEMENTATION__Function_Modifier(enum_types::equality_test);
        /* ----- disjunction ----- *//* ----- disjunction ----- *//* ----- disjunction ----- */
        case enum_types::disjunction:
        {
            assert(input.test_cast<Planning::Formula::Disjunction>());
            deref_VISITATIONS(Planning::Formula::Disjunction, input, get__subformulae());
        }
        break;
        /* ----- disjunction ----- *//* ----- disjunction ----- *//* ----- disjunction ----- */
        /* ----- conjunction ----- *//* ----- conjunction ----- *//* ----- conjunction ----- */
        case enum_types::conjunction:
        {
            assert(input.test_cast<Planning::Formula::Conjunction>());
            (*this)(*input.cxx_get<Planning::Formula::Conjunction>());
        }
        break;
        /* ----- conjunction ----- *//* ----- conjunction ----- *//* ----- conjunction ----- */
        case enum_types::negation:
        {
            assert(input.test_cast<Planning::Formula::Negation>());
            processing_negative = true;
            deref_VISITATION(Planning::Formula::Negation, input, get__subformula());
            processing_negative = false;
        }
        break;
        default: break;
    }
}


void Planning_Formula__to__Variable_Ordering::operator()(const Planning::Argument_List& arguments_List, uint score)
{
    for(auto argument = arguments_List.begin()
            ; argument != arguments_List.end()
            ; argument++){
        if(argument->test_cast<Variable>()){
            auto variable = argument->cxx_get<Variable>();
            auto _old_score = scores.find(*variable);
            if(_old_score != scores.end()){
                auto old_score = _old_score->second;
                if(old_score <= score) continue;
                
                auto variables_with_old_score = scores_to_variables.find(old_score);

                assert(variables_with_old_score != scores_to_variables.end());
                
                variables_with_old_score->second.erase(*variable);
                if(0 == variables_with_old_score->second.size()){
                    scores_to_variables.erase(old_score);
                }
            }

            
            auto variables_with_score = scores_to_variables.find(score);
            if(variables_with_score == scores_to_variables.end()){
                scores_to_variables[score] = Variables();
                variables_with_score = scores_to_variables.find(score);
            }
            
            scores[*variable] = score;
            scores_to_variables[score].insert(*variable);
        }
    }
}

void Planning_Formula__to__Variable_Ordering::operator()(const Planning::Formula::State_Predicate& state_Predicate)
{
    auto predicate_Name = state_Predicate.get__name();
    auto arguments_List = state_Predicate.get__arguments();

    if(!arguments_List.size()) return;

    
    uint score = 100;
    if(((!processing_negative && !domain_Data.in_add_effect(predicate_Name))) ||
       (processing_negative && !domain_Data.in_delete_effect(predicate_Name))) {
        
        if(processing_hard_constraint &&
           (1 == state_Predicate.get__arity())){
            score = 0;
        } else {
            score = state_Predicate.get__arity();
        }

        
//         assert(score >= 0);
        assert(score < 500);/*Won't support symbols with arity hight than ~400 for the moment.*/
        (*this)(arguments_List, score);
    } else {
//         assert(score >= 0);
        assert(score < 500);/*Won't support symbols with arity hight than ~400 for the moment.*/
        (*this)(arguments_List, score + state_Predicate.get__arity());
    }
}


void Planning_Formula__to__Variable_Ordering::operator()(const Planning::Formula::State_Function& state_Function)
{
    auto function_Name = state_Function.get__name();
    auto arguments_List = state_Function.get__arguments();

    if(!arguments_List.size()) return;

    QUERY_UNRECOVERABLE_ERROR(processing_negative, "At this stage we are not expecting function symbols in the context of Boolean negation.");

    uint score = 100;
    if(domain_Data.is_static_fluent(function_Name)){
        score =  state_Function.get__arity();
    } else {
        score += state_Function.get__arity();
        QUERY_UNRECOVERABLE_ERROR((function_Name.get__name() != "total-cost") &&
                                  (function_Name.get__name() != "total-reward") &&
                                  (function_Name.get__name() != "reward")  &&
                                  (function_Name.get__name() != "cost") ,
                                  "Function symbol :: "<<function_Name<<std::endl
                                  <<"Is reported to be dynamic. We do not suppor this at this stage.");
    }

//     assert(score >= static_cast<uint>(0));
    
    (*this)(arguments_List, score);
}
            
void Planning_Formula__to__Variable_Ordering::operator()(const Planning::Formula::Conjunction& conjunction)
{
    auto subformulae = conjunction.get__subformulae();

//     if(!subformulae.size()) return;
    assert(subformulae.size());
    
    if(subformulae.size() == 1){
        auto disjunct = subformulae.back();
        assert(disjunct.test_cast<Planning::Formula::Disjunction>());

        if(disjunct.cxx_get<Planning::Formula::Disjunction>()->get__subformulae().size() == 1){
            processing_hard_constraint = true;
        }
    }
    
    VISITATIONS(conjunction, get__subformulae());

    processing_hard_constraint = false;
}
