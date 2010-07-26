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

#include "assignment_applicator.hh"



using namespace Planning;



Assignment_Applicator::Assignment_Applicator(basic_type::Runtime_Thread runtime_Thread,
                                             Planning::Parsing::Domain_Data& domain_Data,
                                             Planning::Parsing::Problem_Data& problem_Data,
                                             std::map<Variable,  Constants&>& assignment_possibilities)
    :runtime_Thread(runtime_Thread),
     domain_Data(domain_Data),
     problem_Data(problem_Data),
     assignment_possibilities(assignment_possibilities),
     made_assignment_to__runtime_Thread(false),
     processing_negative(false)
{
}


Result Assignment_Applicator::operator()(Conjunct conjunct, const Assignment& assignment)
{
    Subformulae result;
    
    for(auto atom = conjunct.begin()
            ; atom != conjunct.end()
            ; atom++){
        auto temporary = (*this)(*atom, assignment);
        
        /* If the result is not satisfiable.*/
        if(!std::tr1::get<1>(temporary)){
            return Result(conjunct, false);
        }
        
        result.push_back(temporary);
    }

    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread
         , Planning::Formula::Conjunction
         , conjunctive_formula
         , result);
    
    return Result(conjunctive_formula, true);
}

Result Assignment_Applicator::operator()(Disjunct disjunct, const Assignment& assignment)
{
    Subformulae result;
    
    for(auto atom = disjunct.begin()
            ; atom != disjunct.end()
            ; atom++){
        
        auto temporary = (*this)(*atom, assignment);
        if(!std::tr1::get<1>(temporary)) continue;
        
        result.push_back(temporary);
    }

    if(!result.size()) Result(disjunct, false);
    
    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread
         , Planning::Formula::Disjunction
         , disjunctive_formula
         , result);
    
    return Result(disjunctive_formula, true);
}

Result Assignment_Applicator::operator()(Nagative negative, const Assignment& assignment)
{
    processing_negative = true;

    auto result = (*this)(negative->get__subformula(), assignment);
    
    processing_negative = false;
    
    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread
         , Planning::Formula::Negation
         , negated_atom
         , std::tr1::get<0>(result));
    
    
    return Result(negated_atom, std::tr1::get<1>(result));
    
}



bool Assignment_Applicator::satisfiable(Fact fact)
{
    if(!processing_negative && !domain_Data.in_add_effect(fact->get__name())){

        auto predicate_index = fact->get__name().get__id();
        auto _arguments = fact->get__arguments();

        Argument_List arguments(_arguments.size());
        for(auto i = 0; i < arguments.size(); i++){

            if(_arguments[i]->try_cast<Planning::Constant>()){
                NEW_referenced_WRAPPED_deref_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , constant
                     , X);
                arguments[i] = X;
            } else {
                arguments[i] = _arguments[i];
            }
        }
           
        auto cached_Partial_Assignment_Satisfiability = cached_satisfiable.find(predicate_index);

        if(cached_Partial_Assignment_Satisfiability == cached_satisfiable.end()){
            cached_Partial_Assignment_Satisfiability[predicate_index] = Cached_Partial_Assignment_Satisfiability();
            cached_Partial_Assignment_Satisfiability = cached_satisfiable.find(predicate_index);
        }
           
        Cached_Partial_Assignment_Satisfiability& satisfiable__cached =
            std::tr1::get<0>(cached_Partial_Assignment_Satisfiability->second);
        Cached_Partial_Assignment_Unsatisfiability& unsatisfiable__cached =
            std::tr1::get<1>(cached_Partial_Assignment_Satisfiability->second);

        if(satisfiable__cached.find(arguments) != satisfiable__cached.end()){
            return true;
        } else if (unsatisfiable__cached.find(arguments) != unsatisfiable__cached.end()) {
            return false;
        }
           
        if(problem_Data.possibly_statically_satisfiable(*fact)){
            satisfiable__cached.insert(arguments);
            return true;
        } else {
            unsatisfiable__cached.insert(arguments);
            return false;
        }
    } else if(processing_negative && !domain_Data.in_delete_effect(fact->get__name())){

        
    }
    
    return true;
}

bool Assignment_Applicator::satisfiable(Ground_Fact ground_Fact)
{
    auto predicate_Name = ground_Fact->get__name();

    
    if((!processing_negative && !domain_Data.in_add_effect(predicate_Name))){
        auto prop_indices = problem_Data.state_propositions__parsed[predicate_Name];
        
        /*This must already be possible in a starting state, if it is ever reachable.*/
        return (prop_indices.end() != prop_indices.find(ground_Fact->get__id()));
    } else if (processing_negative && !domain_Data.in_delete_effect(predicate_Name)) {
        auto prop_indices = problem_Data.state_propositions__parsed[predicate_Name];
        
        /*This must NOT already be possible in a starting state, if the negation is reachable.*/
        return (prop_indices.end() == prop_indices.find(ground_Fact->get__id()));
    }
    
    return true;
}


Result Assignment_Applicator::operator()(Fact fact, const Assignment& assignment)
{
    auto arguments = fact->get__arguments();

    bool variable_remains_unassigned = false;
    
    Constant_Arguments constant_Arguments;
    Argument_List argument_List;
    
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument++ ){
        if((*argument)->test_cast<Planning::Variable>()){
            auto variable = (*argument)->get<Planning::Variable>();

            auto maplet = assignment.find(*variable);

            if(maplet == assignment.end()){
                
              variable_remains_unassigned = true;
              argument_List.push_back(variable);
              
            } else {
                NEW_referenced_WRAPPED_deref_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , constant
                     , maplet.second.get__name());
                
                if(!variable_remains_unassigned){
                    constant_Arguments.push_back(maplet.second);
                }
                argument_List.push_back(constant);
            }

        } else {
            QUERY_UNRECOVERABLE_ERROR((*argument)->test_cast<Planning::Constant>()
                                      , " Could not understand predicate argument symbol :: "<<*argument<<std::endl);

            auto constant = (*argument)->get<Planning::Constant>();

            if(!variable_remains_unassigned){
                constant_Arguments.push_back(*constant);
            }
            
            argument_List.push_back(constant);
        }   
    }

    /*We have a proposition.*/
    if(!variable_remains_unassigned){

        assert(constant_Arguments.size() == arguments.size());
        
        NEW_referenced_WRAPPED_deref_POINTER
            (runtime_Thread
             , Planning::Formula::State_Proposition
             , proposition
             , constant_Arguments);

        return Result(proposition, satisfiable(proposition));
    } else {
        NEW_referenced_WRAPPED_deref_POINTER
            (runtime_Thread
             , Planning::Formula::State_Predicate
             , predicate
             , argument_List);
        
        return Result(predicate, satisfiable(predicate));
    }
}

Result Assignment_Applicator::operator()(Observation observation, const Assignment& assignment)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
}

Result Assignment_Applicator::operator()(Ground_Fact ground_Fact, const Assignment& assignment)
{
    return Result(ground_Fact, true);
}

Result Assignment_Applicator::operator()(Ground_Observation ground_Observation, const Assignment& assignment)
{
    return Result(ground_Observation, true);
}

Result Assignment_Applicator::operator()(Equality equality, const Assignment& assignment)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    
}


Subformula Assignment_Applicator::operator()(Subformula input,
                                             const Assignment& assignment)
{
    if(!made_assignment_to__runtime_Thread){
        runtime_Thread = input->get__runtime_Thread();
        made_assignment_to__runtime_Thread = true;
    }
    
    
    switch(input->get__type_name()){//get__id()){
        case vacuous:
        {
            return input;
        }
        break;
        case negation:
        {
            assert(input.test_cast<Negation>());
            return (*this)(Nagative(input.cxx_get<Nagative>()), assignment);
        }
        break;
        case state_predicate:
        {
            assert(input.test_cast<State_Predicate>());
            return (*this)(Fact(input.cxx_get<Fact>()), assignment);
        }
        break;
        case state_proposition:
        {
            assert(input.test_cast<State_Proposition>());
            return (*this)(Ground_Fact(input.cxx_get<Ground_Fact>()), assignment);
        }
        break;
        case observational_predicate:
        {
            assert(input.test_cast<Observational_Predicate>());
            return (*this)(Observation(input.cxx_get<Observation>()), assignment);
        }
        break;
        case observational_proposition:
        {
            assert(input.test_cast<Observational_Proposition>());
            return (*this)(Ground_Observation(input.cxx_get<Ground_Observation>()), assignment);
        }
        break;
        case equality_test:
        {
            assert(input.test_cast<Equality_Test>());
            return (*this)(Equality(input.cxx_get<Equality>()), assignment);
        }
        break;
        case conjunction:
        {
            assert(input.test_cast<Conjunction>());
            return (*this)(Conjunct(input.cxx_get<Conjunction>()), assignment);
        }
        break;
        case disjunction:
        {
            assert(input.test_cast<Disjunction>());
            return (*this)(Disjunct(input.cxx_get<Disjunction>()), assignment);
        }
        break;
        case exists:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<_input<<std::endl);
        }
        break;
        case forall:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<_input<<std::endl);
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Cannot ground formula ::"<<input<<std::endl);
        }
            break;
    }

}

void reset__runtime_Thread()
{
    made_assignment_to__runtime_Thread = false;
}

void Assignment_Applicator::reset__assignment_possibilities(std::map<Variable,  Constants&>& _assignment_possibilities)
{
    assignment_possibilities = _assignment_possibilities;
}
