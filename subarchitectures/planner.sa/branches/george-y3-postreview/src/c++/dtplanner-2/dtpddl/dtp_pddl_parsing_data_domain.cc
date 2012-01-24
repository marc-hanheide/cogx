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
#include "dtp_pddl_parsing_data_domain.hh"

// #include "stl__map_transitive_closure.hh"
// #include "stl__anti_reflexive.hh"


using namespace Planning::Parsing;

Domain_Data::Domain_Data()
    :got__action_precondition(false),
     got__action_effect(false),
     got__observation_precondition(false),
     got__observation_execution_precondition(false),
     got__observation_effect(false)
{
    symbol_theory = this;

    
    NEW_object_referenced_WRAPPED(Planning::Type, _double__constant, "double");
    NEW_object_referenced_WRAPPED(Planning::Type, _int__constant, "int");
    NEW_object_referenced_WRAPPED(Planning::Type, _number__constant, "number");
    double__constant = _double__constant;
    int__constant = _int__constant;
    number__constant = _number__constant;
}


const Planning::Action_Schemas& Domain_Data::get__action_Schemas() const
{
    return action_Schemas;
}

Planning::Action_Schemas& Domain_Data::get__action_Schemas()
{
    return action_Schemas;
}


const Planning::Derived_Predicates&  Domain_Data::get__derived_Predicates() const
{
    return derived_Predicates;
}

Planning::Derived_Predicates&  Domain_Data::get__derived_Predicates()
{
    return derived_Predicates;
}



const Planning::Derived_Percepts& Domain_Data::get__derived_Percepts() const
{
    VERBOSER(3110, "No domain parse will generate derived observation predicates.");
    return derived_Percepts;
}

Planning::Derived_Percepts& Domain_Data::get__derived_Percepts()
{
    VERBOSER(3110, "No domain parse will generate derived observation predicates.");
    return derived_Percepts;
}



void Domain_Data::add__derived_predicate_header()
{
    NEW_object_referenced_WRAPPED(Planning::Derived_Predicate_Header
                                  , predicate 
                                  , predicate_Name
                                  , typed_Arguments);
    
    derived_Predicate_Header = predicate;

    typed_Arguments = Typed_Arguments();
}


void Domain_Data::commit__derived_predicate()
{
    
    /*Make sure we have a coherent basis on which to build a derived predicate.*/
    QUERY_UNRECOVERABLE_ERROR(subformulae.find(1) != subformulae.end()
                              , "Asked to commit a derived predicate, yet no formula was parsed.");
    QUERY_UNRECOVERABLE_ERROR(!subformulae[1].size()
                              , "Asked to commit a derived predicate, yet the basis is an empty formula.");
    
//     assert(subformulae.find(1) != subformulae.end());
//     assert(subformulae[1].size());

    /*If we have parsed a formula, then we have finished doing so.*/
    QUERY_UNRECOVERABLE_ERROR(formula_parsing_level != 0
                              , "Asked to commit a derived predicate, yet we \n"
                              <<"are still parsing the formula that described that predicate.");
    
    auto last_formula_parsed = *subformulae[1].begin();
    subformulae[1] = Planning::Formula::Subformulae();

    auto new_predicate__name = derived_Predicate_Header.get__name();
    auto new_predicate__arguments = derived_Predicate_Header.get__arguments();
    
    /* If the last formula parsed was a derived predicate, then copy
     * that as a human engineered predicate, and remove the computer
     * engineered version of it.*/
    if(last_formula_parsed.test_cast<Planning::Derived_Predicate>()){
        auto derived_predicate = last_formula_parsed.do_cast_and_copy<Planning::Derived_Predicate>();
        
        NEW_object_referenced_WRAPPED
            (Planning::Derived_Predicate
             , new_derived_predicate
             , new_predicate__name
             , new_predicate__arguments
             , derived_predicate.get__quantified_symbols()
             , derived_predicate.get__variables()
             , derived_predicate.get__formula()
             , derived_predicate.get__type()
             );

        VERBOSER(42, "new predicate :: "<<new_derived_predicate.get__description());
//         {char ch; std::cin>>ch;}
        Formula_Data::derived_Predicates__artificial
            .erase(derived_predicate);
        
        derived_Predicates
            .insert(new_derived_predicate);

        if(derived__state_predicates__parsed.end() == derived__state_predicates__parsed.find(new_predicate__name)){
            derived__state_predicates__parsed[new_predicate__name] = std::set<ID_TYPE>();
        }
        derived__state_predicates__parsed[new_predicate__name].insert(new_derived_predicate.get__id());
    }
    /* Actually make a derived predicate, and then commit to that.*/
    else {
        
        NEW_object_referenced_WRAPPED
            (Planning::Derived_Predicate
             , new_derived_predicate
             , new_predicate__name
             , new_predicate__arguments
             , Planning::Typed_Arguments()
             , Planning::Variables()
             , last_formula_parsed
             , last_formula_parsed->get__type_name()
             );

        VERBOSER(42, "new predicate :: "<<new_derived_predicate.get__description());
//         {char ch; std::cin>>ch;}
        
        
        derived_Predicates
            .insert(new_derived_predicate);
        
        if(derived__state_predicates__parsed.end() == derived__state_predicates__parsed.find(new_predicate__name)){
            derived__state_predicates__parsed[new_predicate__name] = std::set<ID_TYPE>();
        }
        derived__state_predicates__parsed[new_predicate__name].insert(new_derived_predicate.get__id());
    }

    /* Re-set/initialise the \member{derived_Predicate_Header}, so it
     * is ready to store the information from the next parse of a
     * derived predicate.*/
    derived_Predicate_Header = Planning::Derived_Predicate_Header();
}



Planning::Types Domain_Data::find__type_of_variable(const Planning::Variable& in) const
{
    auto answer = find__action_Header(in);

    if(answer.size()) return answer;

    answer = find__stack_of__Typed_Arguments(in);
    
    if(answer.size()) return answer;
    
    answer = find__derived_Predicate_Header(in);

    return answer;
}

Planning::Types Domain_Data::find__action_Header(const Planning::Variable& in) const
{
    auto typed_variables = std::tr1::get<0>(std::tr1::get<1>(action_Header.contents()));
    auto variablex_types = std::tr1::get<1>(std::tr1::get<1>(action_Header.contents()));
    
    for(auto i = 0
            ; i < typed_variables.size()
            ; i++){
        auto var = typed_variables[i];
        
        assert(var.test_cast<Variable>());
        
        if(in == var.do_cast_and_copy<Variable>()){
            return variablex_types[i];
        }
    }

    return Types();
}

Planning::Types Domain_Data::find__derived_Predicate_Header(const Planning::Variable& in) const
{
    auto typed_variables = std::tr1::get<0>(std::tr1::get<1>(derived_Predicate_Header.contents()));
    auto variablex_types = std::tr1::get<1>(std::tr1::get<1>(derived_Predicate_Header.contents()));
    
    for(uint i = 0
            ; i < typed_variables.size()
            ; i++){
        auto var = typed_variables[i];
        
        assert(var.test_cast<Variable>());
        
        if(in == var.do_cast_and_copy<Variable>()){
            return variablex_types[i];
        }
    }

    return Types();
}

void Domain_Data::report__state_function_domain()
{
    state_function_domain_specification = typed_Arguments;
    typed_Arguments = Typed_Arguments();
}

void Domain_Data::report__perceptual_function_domain()
{
    perceptual_function_domain_specification = typed_Arguments;
    typed_Arguments = Typed_Arguments();
}

void Domain_Data::add__state_function()
{
    assert(types_of_types.size());
    NEW_object_referenced_WRAPPED(Planning::State_Function_Description
                                  , state_function
                                  , state_Function_Name
                                  , state_function_domain_specification
                                  , types_of_types);
    
    state_Function_Descriptions.insert(state_function);

    range_of_state_function[state_Function_Name] = types_of_types;

    INTERACTIVE_VERBOSER(true, 18000, "type of function :: "<<state_Function_Name<<std::endl
                         <<"with arguemnts :: "<<state_function_domain_specification<<std::endl
                         <<"is -- "<<types_of_types<<std::endl);
    
    state_function_domain_specification = Typed_Arguments();
    types_of_types = Types();
}

void Domain_Data::add__perceptual_function()
{
    assert(types_of_types.size());
    
    NEW_object_referenced_WRAPPED(Planning::Perceptual_Function_Description
                                  , perceptual_function 
                                  , perceptual_Function_Name
                                  , perceptual_function_domain_specification
                                  , types_of_types);
    
    perceptual_Function_Descriptions.insert(perceptual_function);
    range_of_perceptual_function[perceptual_Function_Name] = types_of_types;
    
    perceptual_function_domain_specification = Typed_Arguments();
    types_of_types = Types();
}

void Domain_Data::add__percept()
{
    NEW_object_referenced_WRAPPED(Planning::Percept_Description
                                  , percept 
                                  , percept_Name
                                  , typed_Arguments);
    
    percept_Descriptions.insert(percept);

    typed_Arguments = Typed_Arguments();
}

void Domain_Data::add__predicate()
{
    NEW_object_referenced_WRAPPED(Planning::Predicate_Description
                                  , predicate 
                                  , predicate_Name
                                  , typed_Arguments);
    
    predicate_Descriptions.insert(predicate);

    typed_Arguments = Typed_Arguments();
}






void Domain_Data::set_function_range__number()
{
    exit(0);
//     function_Range = t_number;
    function_Range = t_int;
}

void Domain_Data::set_function_range__int()
{
    exit(0);
    function_Range = t_int;
}

void Domain_Data::set_function_range__double()
{
    exit(0);
    function_Range = t_double;
}

void Domain_Data::set_function_range__type(const std::string& str)
{
    exit(0);
    assert("number" != str);
    assert("int" != str);
    assert("double" != str);
    
    function_Range = t_type;
    
    NEW_object_referenced_WRAPPED(Planning::Type, function_range, str);
    range_of_last_function_parsed = function_range;
}




void Domain_Data::add__domain_Name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Domain_Name, dom_nam, str);
    domain_Name = dom_nam;
}

Planning::Domain_Name  Domain_Data::get__domain_Name() const {
    return domain_Name;
};



const Planning::Observation_Schemas& Domain_Data::get__observation_Schemas() const
{
    return observation_Schemas;
}

Planning::Observation_Schemas& Domain_Data::get__observation_Schemas()
{
    return observation_Schemas;
}

          
void Domain_Data::add__requirement(const std::string& str){
    NEW_object_referenced_WRAPPED(Requirement, requirement, str);
    domain_requirements.insert(requirement);
}



#define IMPLEMENTATION____read__type(RESULT_TYPE, CONSTANT, QUERY_TYPE, DATA_BASE) \
    namespace Planning                                                  \
    {                                                                   \
        namespace Parsing                                               \
        {                                                               \
                                                                        \
            bool Domain_Data::                                          \
            RESULT_TYPE(const QUERY_TYPE& _name) const                  \
            {                                                           \
                NEW_object_referenced_WRAPPED(QUERY_TYPE, name, _name.get__name()); \
                                                                        \
                assert(DATA_BASE.find(name) != DATA_BASE.end());        \
                                                                        \
                                                                        \
                auto _range_types = DATA_BASE.find(name);               \
                if(_range_types != DATA_BASE.end()){                    \
                    auto range_types = _range_types->second;            \
                    auto type_index =                                   \
                        range_types.find(CONSTANT);                     \
                    if(type_index != range_types.end()){                \
                        return true;                                    \
                    }                                                   \
                }                                                       \
                                                                        \
                return false;                                           \
            }                                                           \
        }                                                               \
    }                                                                   \
    


IMPLEMENTATION____read__type(is_type__double, double__constant,
                             Planning::State_Function_Name, range_of_state_function);
IMPLEMENTATION____read__type(is_type__int, int__constant,
                             Planning::State_Function_Name, range_of_state_function);
IMPLEMENTATION____read__type(is_type__double, double__constant,
                             Planning::Perceptual_Function_Name, range_of_perceptual_function);
IMPLEMENTATION____read__type(is_type__int, int__constant,
                             Planning::Perceptual_Function_Name, range_of_perceptual_function);
IMPLEMENTATION____read__type(is_type__number, number__constant,
                             Planning::State_Function_Name, range_of_state_function);
IMPLEMENTATION____read__type(is_type__number, number__constant,
                             Planning::Perceptual_Function_Name, range_of_perceptual_function);
//         template<>
//         bool Domain_Data::read__type<double>(const Planning::State_Function_Name& name) const
//         {
//             assert(range_of_state_function.find(name) != range_of_state_function.end());

//             auto _range_types = range_of_state_function.find(name);
//             if(_range_types != range_of_state_function.end()){
//                 auto range_types = _range_types->second;
//                 auto type_index = range_types.find(double__constant);
//                 if(type_index != range_types.end()){
//                     return true;
//                 }
//             }

//             return false;
//         }

//         template<>
//         bool Domain_Data::read__type<int>(const Planning::State_Function_Name& name) const
//         {
//             assert(range_of_state_function.find(name) != range_of_state_function.end());
//             return (range_of_state_function[name] == int__constant);
//         }

//         template<>
//         bool Domain_Data::read__type<double>(const Planning::Perceptual_Function_Name& name) const
//         {
//             assert(range_of_perceptual_function.find(name) != range_of_perceptual_function.end());
//             return (range_of_perceptual_function[name] == double__constant);
//         }

//         template<>
//         bool Domain_Data::read__type<int>(const Planning::Perceptual_Function_Name& name) const
//         {
//             assert(range_of_perceptual_function.find(name) != range_of_perceptual_function.end());
//             return (range_of_perceptual_function[name] == int__constant);
//         }
