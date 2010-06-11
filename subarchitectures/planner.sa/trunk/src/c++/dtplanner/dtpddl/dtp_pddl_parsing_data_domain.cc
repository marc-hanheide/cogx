/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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


const Planning::Action_Schemas& Domain_Data::get__action_Schemas() const
{
    return action_Schemas;
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
    assert(subformulae.find(1) != subformulae.end());
    assert(subformulae[1].size());
    
    /*If we have parsed a formula, then we have finished doing so.*/
    assert(formula_parsing_level == 0);
    
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
        {char ch; std::cin>>ch;}
        Formula_Data::derived_Predicates__artificial
            .erase(derived_predicate);
        
        derived_Predicates
            .insert(new_derived_predicate);
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
             , last_formula_parsed.get()->get__type_name()
             );

        VERBOSER(42, "new predicate :: "<<new_derived_predicate.get__description());
        {char ch; std::cin>>ch;}
        
        
        derived_Predicates
            .insert(new_derived_predicate);
    }

    
    derived_Predicate_Header = Planning::Derived_Predicate_Header();
}



void Domain_Data::stack__typed_Arguments()
{
    VERBOSER(41, "Pushing back quantified arguments :: "<<typed_Arguments);
    stack_of__Typed_Arguments.push(typed_Arguments);
    typed_Arguments = Typed_Arguments();
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
    function_Range = t_number;
}

void Domain_Data::set_function_range__int()
{
    function_Range = t_int;
}

void Domain_Data::set_function_range__double()
{
    function_Range = t_double;
}

void Domain_Data::set_function_range__type(const std::string& str)
{
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



          
void Domain_Data::add__requirement(const std::string& str){
    NEW_object_referenced_WRAPPED(Requirement, requirement, str);
    domain_requirements.insert(requirement);
}

