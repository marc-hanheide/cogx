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

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning::Parsing;


const Planning::Assignment Problem_Data::EMPTY_ASSIGNMENT = Planning::Assignment();


const Planning::Formula::Subformula& Problem_Data::get__objective_function() const
{
    return objective_function;
}




Planning::Formula::Subformula Problem_Data::get__starting_state() const
{
    return starting_state;
}




bool Problem_Data::
statically_true__starting_always_true
(CXX__deref__shared_ptr<Planning::Formula::State_Proposition>& ground_Fact) const
{
    if(starting_state_propositions.find(ground_Fact->get__id())
       != starting_state_propositions.end()){
        INTERACTIVE_VERBOSER(true, 3502, "Statically true fact :: "<<ground_Fact<<".");
       return true; 
    } else {
        INTERACTIVE_VERBOSER(true, 3502, "Non-Statically true fact :: "<<ground_Fact<<".");
    }
    
    
    return false;
}

bool Problem_Data::statically_true__starting_always_true(CXX__deref__shared_ptr<Planning::Formula::State_Predicate>& fact) const
{
    return false;
//     auto occurrence_indices = state_propositions__parsed.find(fact->get__name());
//     if(occurrence_indices == state_propositions__parsed.end()) {
//         INTERACTIVE_VERBOSER(true, 3101, "No ground instances of symbols "<<fact->get__name()<<std::endl);
        
//         return false;
//     }

//     auto predicate_index = fact->get__name().get__id();
//     auto _arguments = fact->get__arguments();

    
    
//     Argument_List arguments(_arguments.size());
        
//     for(auto i = 0; i < arguments.size(); i++){
//         if(_arguments[i].test_cast<Planning::Variable>()){
//             arguments[i] = X_constant;
//         } else {
//             arguments[i] = _arguments[i];
//         }
//     }
    
//     auto cached_Partial_Assignment_Satisfiability
//         = cached__statically_true__starting_always_true.find(predicate_index);
        

//     if(cached_Partial_Assignment_Satisfiability == cached__statically_true__starting_always_true.end()){
//         INTERACTIVE_VERBOSER(true, 3101, "Have not cached ground form of first-order fact :: "
//                              <<fact->get__name()<<" "<<arguments<<std::endl);
        
//         cached__statically_true__starting_always_true[predicate_index]
//             = std::tr1::tuple<Cached_Partial_Assignment_Satisfiability
//             , Cached_Partial_Assignment_Unsatisfiability>();
//         cached_Partial_Assignment_Satisfiability = cached__statically_true__starting_always_true.find(predicate_index);
//     }
    
//     Cached_Partial_Assignment_Satisfiability& satisfiable__cached =
//         std::tr1::get<0>(cached_Partial_Assignment_Satisfiability->second);
//     Cached_Partial_Assignment_Unsatisfiability& unsatisfiable__cached =
//         std::tr1::get<1>(cached_Partial_Assignment_Satisfiability->second);

    
//     auto find_in__satisfiable__cached = satisfiable__cached.find(arguments);
//     auto find_in__unsatisfiable__cached = unsatisfiable__cached.find(arguments);

    
//     if( ( find_in__satisfiable__cached !=  satisfiable__cached.end() ) ||
//         ( find_in__unsatisfiable__cached != unsatisfiable__cached.end() ) ){
//         if(find_in__satisfiable__cached == satisfiable__cached.end()){
//             assert(!(unsatisfiable__cached.find(arguments) == unsatisfiable__cached.end()));
            
            
//             return false;
//         } else if (find_in__unsatisfiable__cached == unsatisfiable__cached.end()) {
            
//             return true;
//         }
//     }

//     if(necessarily_satisfiable(*fact)){
        
//     }
    
}



bool Problem_Data::statically_false__starting_always_false(CXX__deref__shared_ptr<Planning::Formula::State_Proposition>& ground_Fact) const
{
    INTERACTIVE_VERBOSER(true, 11000, "TESTING statically_false__starting_always_false :: "<<ground_Fact);
        
    auto predicate_Name = ground_Fact->get__name();

    if(!get__state_propositions__parsed().size()){
        /*It cannot be made true, and now we find it is not true in the starting state.*/
        return true;
    }
    

#ifndef NDEBUG
    bool debug_answer = false;
    for(auto p = get__state_propositions__parsed().begin()
            ; p != get__state_propositions__parsed().end()
            ; p++){
        std::ostringstream oss1;
        std::ostringstream oss2;
        
        oss1<<predicate_Name;
        oss2<<p->first;
        
        std::string pred_name = oss1.str();
        std::string other_pred_name = oss2.str();

        if(pred_name == other_pred_name){
            INTERACTIVE_VERBOSER(true, 11000, "In starting state :: "<<p->first<<" "<<p->first.get__runtime_Thread()<<std::endl
                                 <<"Required :: "<<predicate_Name<<" "<<predicate_Name.get__runtime_Thread()<<std::endl);
            debug_answer = true;

            assert(get__state_propositions__parsed().find(predicate_Name) != get__state_propositions__parsed().end());
            
            break;
        }
    }
#endif
    
    auto _prop_indices = get__state_propositions__parsed().find(predicate_Name);
        
    /* If we parsed no ground fact instances in the starting
     * state, and we require it to be true.*/
    if(_prop_indices ==
       get__state_propositions__parsed().end()){

        
#ifndef NDEBUG
        assert(!debug_answer);
#endif
        
        INTERACTIVE_VERBOSER(true, 11000, "NO symbol "<<predicate_Name<<" mentioned in starting state. \n"
                             <<"1 - RESULT fact :: "<<ground_Fact<<" can never be satisfied.");
        
        WARNING("getting no occurences of static predicate :: "<<predicate_Name<<std::endl
                <<"in the starting state."<<std::endl);
//         {char ch; std::cin>>ch;}
        return true;
    }

    
    auto prop_indices = _prop_indices->second;
    
    if(ground_Fact->get__runtime_Thread()
       == reinterpret_cast<basic_type::Runtime_Thread>(dynamic_cast<const Formula_Data*>(this))){
        if(prop_indices.end() == prop_indices.find(ground_Fact->get__id())){/*Not in starting state.*/
            INTERACTIVE_VERBOSER(true, 11000, "2 - RESULT fact :: "<<ground_Fact<<" can never be satisfied.");
    
            return true;
        }
    } else {
        
        NEW_referenced_WRAPPED(dynamic_cast<const Formula_Data*>(this)
                               , Planning::Formula::State_Proposition
                               , proposition
                               , predicate_Name
                               , ground_Fact->get__arguments());

        if(prop_indices.end() == prop_indices.find(proposition.get__id())){/*Not in starting state.*/
            INTERACTIVE_VERBOSER(true, 11000, "3 - RESULT fact :: "<<ground_Fact<<" can never be satisfied.");
            return true;
        }
    }
    
    INTERACTIVE_VERBOSER(true, 11000, "RESULT fact :: "<<ground_Fact<<" MIGHT be satisfied.");
    return false;
}

bool Problem_Data::statically_false__starting_always_false
(CXX__deref__shared_ptr<Planning::Formula::State_Predicate>& fact) const
{

    auto occurrence_indices = state_propositions__parsed.find(fact->get__name());
    if(occurrence_indices == state_propositions__parsed.end()) {
        INTERACTIVE_VERBOSER(true, 8000, "No ground instances of symbols "<<fact->get__name()<<std::endl);
        
        return true;
    }
    
    auto predicate_index = fact->get__name().get__id();
    auto _arguments = fact->get__arguments();

    
    
    Argument_List arguments(_arguments.size());
        
    for(uint i = 0; i < arguments.size(); i++){
        if(_arguments[i].test_cast<Planning::Variable>()){
            arguments[i] = X_constant;
        } else {
            arguments[i] = _arguments[i];
        }
    }
    
    INTERACTIVE_VERBOSER(true, 8000, "Testing "<<fact->get__name()<<std::endl
                         <<"With arguments :: "<<arguments<<std::endl);
        
    auto cached_Partial_Assignment_Satisfiability
        = cached__statically_false__starting_always_false.find(predicate_index);
        
    if(cached_Partial_Assignment_Satisfiability == cached__statically_false__starting_always_false.end()){
        INTERACTIVE_VERBOSER(true, 8000, "Have not cached ground form of first-order fact :: "
                             <<fact->get__name()<<" "<<arguments<<std::endl);
        
        cached__statically_false__starting_always_false[predicate_index]
            = std::tr1::tuple<Cached_Partial_Assignment_Satisfiability
            , Cached_Partial_Assignment_Unsatisfiability>();
        cached_Partial_Assignment_Satisfiability = cached__statically_false__starting_always_false.find(predicate_index);
    }
        
//     Cached_Partial_Assignment_Satisfiability& satisfiable__cached =
//         std::tr1::get<0>(cached_Partial_Assignment_Satisfiability->second);
//     Cached_Partial_Assignment_Unsatisfiability& unsatisfiable__cached =
//         std::tr1::get<1>(cached_Partial_Assignment_Satisfiability->second);

    auto& satisfiable__cached =
        std::tr1::get<0>(cached_Partial_Assignment_Satisfiability->second);
    auto& unsatisfiable__cached =
        std::tr1::get<1>(cached_Partial_Assignment_Satisfiability->second);

    auto find_in__satisfiable__cached = satisfiable__cached.find(arguments);
    auto find_in__unsatisfiable__cached = unsatisfiable__cached.find(arguments);

    if( ( find_in__satisfiable__cached !=  satisfiable__cached.end() ) ||
        ( find_in__unsatisfiable__cached != unsatisfiable__cached.end() ) ){
        if(find_in__satisfiable__cached != satisfiable__cached.end()){
            assert(unsatisfiable__cached.find(arguments) == unsatisfiable__cached.end());
            
            INTERACTIVE_VERBOSER(true, 8000, "TRUE CASE :: Cached GROUNDABLE ::  of first-order fact :: "<<fact->get__name()<<" "<<arguments<<std::endl);
            
            
            return false;
        } else if (find_in__unsatisfiable__cached != unsatisfiable__cached.end()) {
            INTERACTIVE_VERBOSER(true, 8000, "FALSE CASE :: Cached UN-GROUNDABLE ::  of first-order fact :: "<<fact->get__name()<<" "<<arguments<<std::endl);
            
            return true;
        }
    }
    
    if(statically_satisfiable(*fact)){
        INTERACTIVE_VERBOSER(true, 3101, "FALSE CASE :: Cache-ING POSSIBLY GROUNDABLE ::  of first-order fact :: "<<fact->get__name()<<" "<<arguments<<std::endl);
        satisfiable__cached.insert(arguments);
        
        return false;
    } else {
        INTERACTIVE_VERBOSER(true, 3101, "FALSE CASE :: Cache-ING UN-GROUNDABLE ::  of first-order fact :: "<<fact->get__name()<<" "<<arguments<<std::endl);
        
        unsatisfiable__cached.insert(arguments);
        return true;
    }
}



bool Problem_Data::translate_to_problem_arguments(const Planning::Argument_List& _arguments, Planning::Argument_List& arguments) const
{
    bool result = false;
    
    for(uint i = 0; i < arguments.size(); i++){
        if(_arguments[i].test_cast<Planning::Variable>()){
            arguments[i] = _arguments[i];
        } else if (_arguments[i].test_cast<Planning::Constant>()) {
            auto constant_symbol = _arguments[i].cxx_get<Planning::Constant>();
                
            if(constant_symbol->get__runtime_Thread()
               != reinterpret_cast<basic_type::Runtime_Thread>
               (dynamic_cast<const Planning::Parsing::Constants_Data*>(this))){

                result = true;
                
                INTERACTIVE_VERBOSER(true, 3101, "Constant symbol :: "<<constant_symbol
                                     <<"was *NOT* from the problem, "<<std::endl
                                     <<"rather probably from the domain."<<std::endl);

                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (reinterpret_cast<basic_type::Runtime_Thread>
                     (dynamic_cast<const  Planning::Parsing::Constants_Data*>(this))
                     , Planning::Constant
                     , constant
                     , constant_symbol->get__name());
                    
                arguments[i] = constant;
            } else {
                INTERACTIVE_VERBOSER(true, 3101, "Constant symbol :: "<<constant_symbol<<"was from the problem, "<<std::endl);
                arguments[i] = _arguments[i];
            }
        } else {
            UNRECOVERABLE_ERROR("Rubbish argument in :: "<<_arguments<<std::endl);
        }
    }

    return result;
}

bool Problem_Data::translate_to_problem_arguments(const Planning::Constant_Arguments& _arguments,
                                                  Planning::Constant_Arguments& arguments) const
{
    bool result = false;
    
    for(uint i = 0; i < arguments.size(); i++){
        auto constant_symbol = _arguments[i];
                
        if(constant_symbol.get__runtime_Thread()
           != reinterpret_cast<basic_type::Runtime_Thread>
           (dynamic_cast<const Planning::Parsing::Constants_Data*>(this))){

            result = true;
                
            INTERACTIVE_VERBOSER(true, 3101, "Constant symbol :: "<<constant_symbol
                                 <<"was *NOT* from the problem, "<<std::endl
                                 <<"rather probably from the domain."<<std::endl);

            NEW_referenced_WRAPPED//_deref_visitable_POINTER
                (reinterpret_cast<basic_type::Runtime_Thread>
                 (dynamic_cast<const Planning::Parsing::Constants_Data*>(this))
                 , Planning::Constant
                 , constant
                 , constant_symbol.get__name());
                    
            arguments[i] = constant;
        } else {
            INTERACTIVE_VERBOSER(true, 3101, "Constant symbol :: "<<constant_symbol<<" was from the problem, "<<std::endl);
            arguments[i] = _arguments[i];
        }
    }
    
    return result;   
}



void Problem_Data::report__observations(const std::vector<std::string>& observationSeq)
{
}

Planning::Formula::Action_Proposition Problem_Data::get__prescribed_action()
{
    
    VERBOSER(1000, "Trying for prescribed action.");
    auto action_Schemas = domain_Data->get__action_Schemas();

    assert(action_Schemas.size());

    auto action_index = (random() % action_Schemas.size());

    auto _action_Schema = action_Schemas.begin();
    for(uint i = 0; i != action_index; i++,_action_Schema++);
    auto action_Schema = *_action_Schema;
    
    auto action_name = action_Schema.get__name();

    auto action_header = action_Schema.get__header();

    auto header_contents = action_header.contents();
    
    auto arguments = std::tr1::get<1>(header_contents);
    
    auto variables = get__symbols(arguments);
    auto types = get__types(arguments);
    
    Planning::Constant_Arguments constants;

    VERBOSER(1000, "Looking into :: "<<action_name<<std::endl);
    
    for(auto argument = types.begin()
            ; argument != types.end()
            ; argument ++){

        assert(argument->size());
        auto _type = argument->begin();
        
//         NEW_object_referenced_WRAPPED(Planning::Type, type, _type->get__name());

        QUERY_WARNING(!constants_Description.size(),
                      "No problem constants parsed.");
        
        Planning::Constant_Arguments potential;
        for(auto constant = constants_Description.begin()
                ; constant != constants_Description.end()
                ; constant++){
            INTERACTIVE_VERBOSER(true, 3510, "Got a constant :: "<<constant->first<<std::endl
                                 <<"of type :: "<<constant->second);
            if(constant->second.find(*_type) != constant->second.end()){
                potential.push_back(constant->first);
            }
        }

        QUERY_WARNING(!domain_Data->get__constants_Description().size(),
                      "No domain constants parsed.");
        
//         NEW_referenced_WRAPPED(domain_Data.get(), Planning::Type, other_type, _type->get__name());
        auto other_constants = domain_Data->get__constants_Description();
        for(auto constant = other_constants.begin()
                ; constant != other_constants.end()
                ; constant++){
            INTERACTIVE_VERBOSER(true, 3510, "Got a constant :: "<<constant->first<<std::endl
                                 <<"of type :: "<<constant->second);
            if(constant->second.find(*_type) != constant->second.end()){
                potential.push_back(constant->first);
            }
        }

        
        QUERY_UNRECOVERABLE_ERROR(!potential.size(),
                                  "Could not find any objects of type :: "<<type<<std::endl);
        
        
        
        auto index = (random() % potential.size());
        constants.push_back(potential[index]);
    }

    
    NEW_object_referenced_WRAPPED(Planning::Formula::Action_Proposition, answer, action_name, constants);

    return answer;
}


void Problem_Data::report__starting_state()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing starting state."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty starting state."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting starting state."<<std::endl);
    
    this->starting_state = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}

void Problem_Data::report__objective_function()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing objective."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty objective."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting objective."<<std::endl);
    
    this->objective_function = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}


void Problem_Data::report__goal_formula()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing goal."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty goal."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting goal."<<std::endl);
    
    this->goal_formula = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}


const CXX__PTR_ANNOTATION(Domain_Data)& Problem_Data::get__domain_Data() const
{return domain_Data;}


CXX__PTR_ANNOTATION(Domain_Data)  Problem_Data::get__domain_Data()
{return domain_Data;}


Problem_Data::Problem_Data(CXX__PTR_ANNOTATION(Domain_Data)& domain_Data)
    :domain_Data(domain_Data)
{
    NEW_object_referenced_WRAPPED_deref_visitable_POINTER
        (Planning::Constant
         , _constant
         , "X");
    X_constant = _constant;
}

void Problem_Data::reset__domain_Data(CXX__PTR_ANNOTATION(Domain_Data)& in__domain_Data)
{
    domain_Data = in__domain_Data;
}

void Problem_Data::reset__domain_Data(const Planning::Domain_Name& domain_Name)
{
    if(domain_Name == domain_Data->domain_Name){
        /*(see \module{Formula_Data})*/
        report__symbol_name_reference(domain_Data.get());
        
        WARNING("No change occurred when re-specifying problem associated to a domain.");
        return;
    }
                
    auto domains_iterator = Planning::Parsing::domains.find(domain_Name);
    if(domains_iterator != Planning::Parsing::domains.end()){
        reset__domain_Data(domains_iterator->second);
        
        /*(see \module{Formula_Data})*/
        report__symbol_name_reference(domains_iterator->second.get());
    } else {
        UNRECOVERABLE_ERROR("Unable to find domain :: "<<domain_Name<<std::endl);
    }
    

}

void Problem_Data::reset__domain_Data(const std::string& str)
{
    for(auto domain = Planning::Parsing::domains.begin()
            ; domain != Planning::Parsing::domains.end()
            ; domain++){

        /* If we have parsed a domain description that is suitable for
         * this problem...*/
        if(domain->first.get__name() == str){
            reset__domain_Data(domain->first);
            return;
        }
    }
    
    WARNING("Problem :: "<<problem_Name<<std::endl
            <<"Is supposed to be associated with a domain identified as :: "<<str<<std::endl
            <<"Yet as far as I can tell, we have not parsed a domain file"<<std::endl
            <<"That that defines such a domain.");
}

void Problem_Data::add__problem_Name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Problem_Name, prob_nam, str);
    problem_Name = prob_nam;
}

const Planning::Problem_Name& Problem_Data::get__problem_Name() const
{
    return problem_Name;
}

void Problem_Data::report__minimisation_objective()
{
    objective = minimise;
}

void Problem_Data::report__maximisation_objective()
{
    objective = maximise;
}



bool Problem_Data::has_static_value(Formula::State_Ground_Function& in) const
{
    if(static_ground_double_function.find(in) != static_ground_double_function.end()){
        return true;
    }

    if(static_ground_int_function.find(in) != static_ground_int_function.end()){
        return true;
    }

    return false;
}


bool Problem_Data::has_static_value(Formula::Subformula input
                                    , const Planning::Assignment& assignment) const
{
    switch(input->get__type_name()){
        case enum_types::number:
        {
            return true;
        }
        break;
        case enum_types::state_ground_function:
        {
            return has_static_value(*input.cxx_get<Formula::State_Ground_Function>());
        }
        break;
        case enum_types::state_function:
        {
            
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
                (dynamic_cast<const Planning::Parsing::Formula_Data*>(this),
                 Formula::State_Ground_Function,
                 ground_function,
                 symbol->get__name(),
                 constant_Arguments);
            
            return has_static_value(*ground_function.cxx_get<Formula::State_Ground_Function>());
        }
        break;
        default:
            break;
    }

    return false;
}

namespace Planning
{
    namespace Parsing
    {
        template<>
        double Problem_Data::read__static_value(const Planning::Formula::State_Ground_Function& in) const
        {
//             NEW_object_referenced_WRAPPED(Formula::State_Ground_Function, in, _in.get__name());
            
            if(static_ground_double_function.find(in)
               == static_ground_double_function.end()){
                
                for(auto t = static_ground_double_function.begin()
                   ; t != static_ground_double_function.end()
                   ; t++){
                    std::cerr<<t->first<<" "<<t->second<<std::endl;
                }
            }
            
            
            
            assert(static_ground_double_function.find(in)
                   != static_ground_double_function.end());

            
            
            return static_ground_double_function.find(in)->second;
        }
        
        template<>
        int Problem_Data::read__static_value(const Planning::Formula::State_Ground_Function& in) const
        {
//             NEW_object_referenced_WRAPPED(Formula::State_Ground_Function, in, _in.get__name());
            
            assert(static_ground_int_function.find(in)
                   != static_ground_int_function.end());

            return static_ground_int_function.find(in)->second;
        }
        
        template<>
        double Problem_Data::read__static_value<double>(
            Formula::Subformula input,
            const Planning::Assignment& assignment) const
        {
    
            switch(input->get__type_name()){
                case enum_types::number:
                {
                    return input.cxx_get<Formula::Number>()->get__value();
                }
                break;
                case enum_types::state_ground_function:
                {
                    assert(static_ground_double_function.find(*input.cxx_get<Formula::State_Ground_Function>())
                           != static_ground_double_function.end());
            
                    return static_ground_double_function.find(*input.cxx_get<Formula::State_Ground_Function>())->second;
                }
                break;
                case enum_types::state_function:
                {
            
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

            
//                     NEW_referenced_WRAPPED_deref_POINTER
                    NEW_referenced_WRAPPED_deref_visitable_POINTER
                        (dynamic_cast<const Planning::Parsing::Formula_Data*>(this),
                         Formula::State_Ground_Function,
                         ground_function,
                         symbol->get__name(),
                         constant_Arguments);
            
                    return read__static_value<double>(ground_function);
                }
                break;
                default:
                    UNRECOVERABLE_ERROR("Asked for static double value for symbol that does not have a double value.");
                    break;
            }
        }

        template<>
        int Problem_Data::read__static_value<int>(
            Formula::Subformula input,
            const std::map<Planning::Variable, Planning::Constant>& assignment) const
        {
    
            switch(input->get__type_name()){
                case enum_types::number:
                {
                    return static_cast<int>(input.cxx_get<Formula::Number>()->get__value());
                }
                break;
                case enum_types::state_ground_function:
                {
                    assert(static_ground_int_function.find(*input.cxx_get<Formula::State_Ground_Function>())
                           != static_ground_int_function.end());
            
                    return static_ground_int_function.find(*input.cxx_get<Formula::State_Ground_Function>())->second;
                }
                break;
                case enum_types::state_function:
                {
            
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

            
//                     NEW_referenced_WRAPPED_deref_POINTER
                    NEW_referenced_WRAPPED_deref_visitable_POINTER
                        (dynamic_cast<const Planning::Parsing::Formula_Data*>(this),
                         Formula::State_Ground_Function,
                         ground_function,
                         symbol->get__name(),
                         constant_Arguments);
            
                    return read__static_value<int>(ground_function);
                }
                break;
                default:
                    UNRECOVERABLE_ERROR("Asked for static int value for symbol that does not have a int value.");
                    break;
            }
        }


    }
}
