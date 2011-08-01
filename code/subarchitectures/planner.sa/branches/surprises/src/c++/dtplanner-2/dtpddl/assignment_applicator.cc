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

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"


using namespace Planning;

typedef  CNF_Assignment_Applicator::Result Result;

typedef CNF_Assignment_Applicator::Subformulae Subformulae;
typedef CNF_Assignment_Applicator::Subformula Subformula;
typedef CNF_Assignment_Applicator::Negation Negation;
typedef CNF_Assignment_Applicator::Conjunction Conjunction;
typedef CNF_Assignment_Applicator::Disjunction Disjunction;
typedef CNF_Assignment_Applicator::Exists Exists;
typedef CNF_Assignment_Applicator::Forall Forall;
typedef CNF_Assignment_Applicator::State_Proposition State_Proposition;
typedef CNF_Assignment_Applicator::Perceptual_Predicate Perceptual_Predicate;
typedef CNF_Assignment_Applicator::Perceptual_Proposition Perceptual_Proposition;
typedef CNF_Assignment_Applicator::Equality_Test Equality_Test;
typedef CNF_Assignment_Applicator::State_Predicate State_Predicate;


CNF_Assignment_Applicator::
CNF_Assignment_Applicator(basic_type::Runtime_Thread _runtime_Thread,
                          const Planning::Parsing::Domain_Data& domain_Data,
                          const Planning::Parsing::Problem_Data& problem_Data,
                          std::pair<basic_type::Runtime_Thread, ID_TYPE>& actions_validator
                          )
    
    :runtime_Thread(_runtime_Thread),
     domain_Data(domain_Data),
     problem_Data(problem_Data),
     actions_validator(actions_validator),
     processing_negative(false)
{
    
    NEW_referenced_WRAPPED_deref_visitable_POINTER
        (_runtime_Thread
         , Planning::Formula::True
         , _formula__true
         , 0);

    
    NEW_referenced_WRAPPED_deref_visitable_POINTER
        (_runtime_Thread
         , Planning::Formula::False
         , _formula__false
         , 0);

    
    formula__true = _formula__true;
    formula__false = _formula__false;
}


/* PREDICATE -- Some of the arguments are not ground.*/
Result CNF_Assignment_Applicator::satisfiable(Fact fact)
{
    if(!domain_Data.in_add_effect(fact->get__name())){
        INTERACTIVE_VERBOSER(true, 11000, "Got UN-MAKEABLE first-order fact :: "<<fact);

        if(problem_Data.statically_false__starting_always_false(fact)){
            if(!processing_negative){
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial FALSE) UN-MAKEABLE needed to be TRUE in STARTING STATES but wasn't :: "<<fact);
                return Result(formula__false, false);
            } else {
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial TRUE) UN-MAKEABLE always false in STARTING STATE :: "<<fact);
                return Result(formula__true, true);
            }
        }
    }
    
    
    if(!domain_Data.in_delete_effect(fact->get__name())){
        INTERACTIVE_VERBOSER(true, 11000, " Got UN-BREAKABLE first-order fact :: "<<fact);

        if(problem_Data.statically_true__starting_always_true(fact)){
            if(!processing_negative){
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial TRUE)  UN-BREAKABLE always true in STARTING STATE  :: "<<fact);
                return Result(formula__true, true);
            } else {
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial FALSE) UN-BREAKABLE needed to be false in STARTING STATES but wasn't :: "<<fact);
                return Result(formula__false, false);
            }
        }
    }

    
    INTERACTIVE_VERBOSER(true, 11000, "Got satisfiable first-order fact :: "<<fact);
    
    return Result(Subformula(fact), true);

}


/* PROPOSITION -- All of the arguments are ground.*/
Result CNF_Assignment_Applicator::satisfiable(Ground_Fact ground_Fact)
{
    if(!domain_Data.in_add_effect(ground_Fact->get__name())){
        INTERACTIVE_VERBOSER(true, 3102, "Got UN-MAKEABLE ground fact :: "<<ground_Fact);

        if(problem_Data.statically_false__starting_always_false(ground_Fact)){
            if(!processing_negative){
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial FALSE) UN-MAKEABLE needed to be TRUE in STARTING STATES but wasn't :: "<<ground_Fact);
                return Result(formula__false, false);
            } else {
                INTERACTIVE_VERBOSER(true, 11000, " -- (trivial TRUE) UN-MAKEABLE always false in STARTING STATE :: "<<ground_Fact);
                return Result(formula__true, true);
            }
            
//             INTERACTIVE_VERBOSER(true, 11000, " --  (trivial FALSE) UN-MAKEABLE and requested truth value in STARTING STATES :: "<<ground_Fact);

//             return Result(formula__false, false);
        }
    }
    
    
    if(!domain_Data.in_delete_effect(ground_Fact->get__name())){
        INTERACTIVE_VERBOSER(true, 11000, " Got UN-BREAKABLE ground fact :: "<<ground_Fact);

        if(problem_Data.statically_true__starting_always_true(ground_Fact)){
            if(!processing_negative){
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial TRUE)  UN-BREAKABLE always true in STARTING STATE  :: "<<ground_Fact);
                return Result(formula__true, true);
            } else {
                INTERACTIVE_VERBOSER(true, 11000, " --  (trivial FALSE) UN-BREAKABLE needed to be false in STARTING STATES but wasn't :: "<<ground_Fact);
                return Result(formula__false, false);
            }

            
//             INTERACTIVE_VERBOSER(true, 11000, " --  (trivial TRUE)  UN-BREAKABLE and requested truth value in STARTING STATES  :: "<<ground_Fact);
//             return Result(formula__true, true);
        }
    }
    
    INTERACTIVE_VERBOSER(true, 11000, "Got possible ground ground_Fact :: "<<ground_Fact);
    
    return Result(Subformula(ground_Fact), true);
}


Result CNF_Assignment_Applicator::operator()(Conjunct conjunct, const Planning::Assignment& assignment)
{
    Subformulae result;
    
    auto conjunctive_elements = conjunct->get__subformulae();
    
    QUERY_UNRECOVERABLE_ERROR(!conjunctive_elements.size(),
                              "Got an empty conjunction.");
    
    for(auto atom = conjunctive_elements.begin()
            ; atom != conjunctive_elements.end()
            ; atom++){
        auto temporary = (*this)(*atom, assignment);
        
        /* If one element of a conjunct is false, then the whole
         * conjunct is false.*/
        if(!std::tr1::get<1>(temporary)){
            return Result(formula__false, false);
        }

        if(std::tr1::get<0>(temporary) == formula__true) continue;
        
        result.push_back(std::tr1::get<0>(temporary));
    }

    /*All the conjunctive elements were statically satisfied.*/
    if(result.size() == 0){
        INTERACTIVE_VERBOSER(true, 3101, "Conjunction "<<conjunct<<" "
                             <<" is unsatisfiable  given assignment"<<std::endl);
        
        return Result(formula__true, true);
    }

    NEW_referenced_WRAPPED_deref_visitable_POINTER
        (runtime_Thread
         , Planning::Formula::Conjunction
         , conjunctive_formula
         , result);
    
    return Result(conjunctive_formula, true);
}

Result CNF_Assignment_Applicator::operator()(Disjunct disjunct, const Planning::Assignment& assignment)
{
    Subformulae result;

    auto disjunctive_elements = disjunct->get__subformulae();
    
    QUERY_UNRECOVERABLE_ERROR(!disjunctive_elements.size(),
                              "Got an empty disjunct.");
    
    for(auto atom = disjunctive_elements.begin()
            ; atom != disjunctive_elements.end()
            ; atom++){
        
        auto temporary = (*this)(*atom, assignment);
        
        /* If one element of a conjunct is false, then the whole
         * conjunct is false.*/
        if(!std::tr1::get<1>(temporary)) continue;

        
        if(std::tr1::get<0>(temporary) == formula__true) {
            return Result(formula__true, true);
        }
        
        result.push_back(std::tr1::get<0>(temporary));
    }

    if(!result.size()) {
        INTERACTIVE_VERBOSER(true, 3101, "Disjunction "<<disjunct<<" "
                             <<" is unsatisfiable  given assignment."<<std::endl);
        
        return Result(formula__false, false);
    }
    
    
    NEW_referenced_WRAPPED_deref_visitable_POINTER
        (runtime_Thread
         , Planning::Formula::Disjunction
         , disjunctive_formula
         , result);
    
    return Result(disjunctive_formula, true);
}

Result CNF_Assignment_Applicator::operator()(Negative negative, const Planning::Assignment& assignment)
{
    processing_negative = true;

    auto result = (*this)(negative->get__subformula(), assignment);
    
    processing_negative = false;
    
    if(std::tr1::get<0>(result) == formula__true){
        return result;//Result(formula__false, false);
    } else if (std::tr1::get<0>(result) == formula__false) {
        return result;//Result(formula__true, true);
    }
    
    NEW_referenced_WRAPPED_deref_visitable_POINTER
        (runtime_Thread
         , Planning::Formula::Negation
         , negated_atom
         , std::tr1::get<0>(result));
    
    return Result(negated_atom, std::tr1::get<1>(result));
    
}

Result CNF_Assignment_Applicator::operator()(Action_Fact fact, const Planning::Assignment& assignment)
{
    auto arguments = fact->get__arguments();

    bool variable_remains_unassigned = false;
    
    Constant_Arguments constant_Arguments;
    Argument_List argument_List;

    bool no_assignment_made = true;
    
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument++ ){
        if((*argument).test_cast<Planning::Variable>()){
            auto variable = (*argument).cxx_get<Planning::Variable>();

            auto maplet = assignment.find(*variable);

            if(maplet == assignment.end()){
                
              variable_remains_unassigned = true;
              argument_List.push_back(*argument);
              
            } else {
                assert(runtime_Thread == reinterpret_cast<basic_type::Runtime_Thread>
                 (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
                
                no_assignment_made = false;
                
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , constant
                     , maplet->second.get__name());
                
                INTERACTIVE_VERBOSER(true, 3101, "New constant :: "<<constant<<" for thread :: "<<runtime_Thread<<std::endl);
                if(!variable_remains_unassigned){
                    if(maplet->second.get__runtime_Thread() != runtime_Thread){
                        constant_Arguments.push_back(*constant.cxx_get<Planning::Constant>());
                    } else {
                        constant_Arguments.push_back(maplet->second);
                    }
//                     constant_Arguments.push_back(maplet->second);
                }
                argument_List.push_back(constant);
            }

        } else {
            QUERY_UNRECOVERABLE_ERROR(!(*argument).test_cast<Planning::Constant>()
                                      , " Could not understand predicate argument symbol :: "<<*argument<<std::endl
                                      <<"It is not a variable, and now we find it not to be a constant either."<<std::endl);

            auto constant = (*argument).cxx_get<Planning::Constant>();

            
            if(constant->get__runtime_Thread() != runtime_Thread){   
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , _constant
                     , constant->get__name());

                constant = _constant.cxx_get<Planning::Constant>();
            }
            
            if(!variable_remains_unassigned){
                constant_Arguments.push_back(*constant);
            }
            
            argument_List.push_back(Formula::Subformula(constant));
//             argument_List.push_back(*argument);
        }   
    }

    if(no_assignment_made){
        return Result(Subformula(fact), true);
    }
    
    /* We have a proposition. */
    if(!variable_remains_unassigned){

        assert(constant_Arguments.size() == arguments.size());
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (actions_validator.first//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
             , Planning::Formula::Action_Proposition
             , proposition
             , fact->get__name()
             , constant_Arguments);

        return (*this)(proposition, assignment);//satisfiable(Ground_Fact(proposition));
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (actions_validator.first
             , Planning::Formula::Action_Predicate
             , predicate
             , fact->get__name()
             , argument_List);
        
        return Result(predicate, true);
    }
}

Result CNF_Assignment_Applicator::operator()(Fact fact, const Planning::Assignment& assignment)
{
    auto arguments = fact->get__arguments();

    bool variable_remains_unassigned = false;
    
    Constant_Arguments constant_Arguments;
    Argument_List argument_List;

    bool no_assignment_made = true;
    
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument++ ){
        if((*argument).test_cast<Planning::Variable>()){
            auto variable = (*argument).cxx_get<Planning::Variable>();

            auto maplet = assignment.find(*variable);

            if(maplet == assignment.end()){
                
              variable_remains_unassigned = true;
              argument_List.push_back(*argument);
              
            } else {
                assert(runtime_Thread == reinterpret_cast<basic_type::Runtime_Thread>
                 (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
                
                no_assignment_made = false;
                
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , constant
                     , maplet->second.get__name());
                
                INTERACTIVE_VERBOSER(true, 3101, "New constant :: "<<constant<<" for thread :: "<<runtime_Thread<<std::endl);
                if(!variable_remains_unassigned){
                    if(maplet->second.get__runtime_Thread() != runtime_Thread){
                        constant_Arguments.push_back(*constant.cxx_get<Planning::Constant>());
                    } else {
                        constant_Arguments.push_back(maplet->second);
                    }
                    
                }
                argument_List.push_back(constant);
            }

        } else {
            QUERY_UNRECOVERABLE_ERROR(!(*argument).test_cast<Planning::Constant>()
                                      , " Could not understand predicate argument symbol :: "<<*argument<<std::endl
                                      <<"It is not a variable, and now we find it not to be a constant either."<<std::endl);

            auto constant = (*argument).cxx_get<Planning::Constant>();

            
            if(constant->get__runtime_Thread() != runtime_Thread){   
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread
                     , Planning::Constant
                     , _constant
                     , constant->get__name());

                constant = _constant.cxx_get<Planning::Constant>();
            }
            
            
            if(!variable_remains_unassigned){
                constant_Arguments.push_back(*constant);
                INTERACTIVE_VERBOSER(true, 10010, "New constant :: "<<constant_Arguments.back()<<std::endl);
            }
            
            argument_List.push_back(Formula::Subformula(constant));
            INTERACTIVE_VERBOSER(true, 10010, "New constant :: "<<argument_List.back()<<std::endl);
//             argument_List.push_back(*argument);
        }   
    }

    if(no_assignment_made){
        return Result(Subformula(fact), true);
    }
    
    /* We have a proposition. */
    if(!variable_remains_unassigned){
        
        assert(constant_Arguments.size() == arguments.size());
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
             , Planning::Formula::State_Proposition
             , proposition
             , fact->get__name()
             , constant_Arguments);

        INTERACTIVE_VERBOSER(true, 10004, "Testing satisfiability of :: "<<proposition<<std::endl);
        
        
        return satisfiable(Ground_Fact(proposition));
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (runtime_Thread
             , Planning::Formula::State_Predicate
             , predicate
             , fact->get__name()
             , argument_List);
        
        INTERACTIVE_VERBOSER(true, 11000, "Testing satisfiability of :: "<<predicate<<std::endl);
        
        return satisfiable(Fact(predicate));
    }
}

Result CNF_Assignment_Applicator::operator()(Perception perception, const Planning::Assignment& assignment)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Result(Subformula(perception), true);
}

Result CNF_Assignment_Applicator::operator()(Ground_Fact ground_Fact, const Planning::Assignment& assignment)
{
    
    auto arguments = ground_Fact->get__arguments();

    Constant_Arguments constant_Arguments;

    bool no_spurious_constants = true;
    for(auto argument = arguments.begin()
            ; argument != arguments.end()
            ; argument++ ){
        if(argument->get__runtime_Thread() != runtime_Thread) {
            no_spurious_constants = false;
            INTERACTIVE_VERBOSER(true, 10012, "Spurious constant :: "<<*argument<<std::endl);
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
                    (runtime_Thread
                     , Planning::Constant
                     , constant
                     , arguments[index].get__name());
                constant_Arguments[index] = constant;
            } else {
                constant_Arguments[index] = arguments[index];
            }
            
            INTERACTIVE_VERBOSER(true, 10010, "New constant :: "<<constant_Arguments[index]<<std::endl);
        }
    }

//     assert(no_spurious_constants);
    assert(runtime_Thread == reinterpret_cast<basic_type::Runtime_Thread>
           (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
    assert(runtime_Thread == reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data));
    
    if(!no_spurious_constants){
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
             , Planning::Formula::State_Proposition
             , proposition
             , ground_Fact->get__name()
             , constant_Arguments);

        INTERACTIVE_VERBOSER(true, 10004, "Testing satisfiability of :: "<<proposition<<std::endl);
        
        return satisfiable(Ground_Fact(proposition));
    } else {
        return satisfiable(ground_Fact);
    }
}

Result CNF_Assignment_Applicator::operator()(Ground_Perception ground_Perception, const Planning::Assignment& assignment)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Result(Subformula(ground_Perception), true);
}

Result CNF_Assignment_Applicator::operator()(Equality equality, const Planning::Assignment& assignment)
{
    UNRECOVERABLE_ERROR("UNIMPLEMENTED");
    return Result(Subformula(equality), true);
}


Result CNF_Assignment_Applicator::operator()(Subformula input,
                                             const Planning::Assignment& assignment)
{
    
    switch(input->get__type_name()){//get__id()){
        case enum_types::vacuous:
        {
            return Result(formula__true, true);
        }
        break;
        case enum_types::formula_true:
        {
            return Result(formula__true, true);
        }
        break;
        case enum_types::formula_false:
        {
            return Result(formula__false, false);
        }
        break;
        case enum_types::negation:
        {
            assert(input.test_cast<Negation>());
            return (*this)(Negative(input), assignment);
        }
        break;
        case enum_types::action_proposition:
        {
            assert(input.test_cast<Formula::Action_Proposition>());
            auto _action_proposition = input.cxx_get<Formula::Action_Proposition>();

            decltype(_action_proposition) action_proposition;
            if(actions_validator.first != _action_proposition->get__runtime_Thread()){
                NEW_referenced_WRAPPED_deref_POINTER
                    (actions_validator.first
                     , Formula::Action_Proposition
                     , new__action_proposition
                     , _action_proposition->get__name()
                     , _action_proposition->get__arguments());

                action_proposition = new__action_proposition.cxx_get<Formula::Action_Proposition>();
            } else {
                action_proposition = _action_proposition;
            }

            assert(actions_validator.second);
            
            if(action_proposition->get__id() < actions_validator.second){
                
                INTERACTIVE_VERBOSER(true, 10016, "Admissible action literal :: "
                                     <<*action_proposition<<" at thread :: "<<actions_validator.first<<std::endl);
                
                return Result(Formula::Subformula(action_proposition), true);
            } else {
                INTERACTIVE_VERBOSER(true, 10016, "Inadmissible action literal :: "
                                     <<*action_proposition<<" at thread :: "<<actions_validator.first<<std::endl);
                
                
                if(processing_negative){/*Truth value of negated symbol.*/
                    return Result(formula__true, true);
                } else {/*Truth value of symbol.*/
                    return Result(formula__false, false);
                }
            }
            
            assert(0);
        }
        break;
        case enum_types::action_predicate:
        {
            assert(input.test_cast<Formula::Action_Predicate>());
            return (*this)(Action_Fact(input), assignment);
            
        }
        break;
        case enum_types::state_predicate:
        {
            assert(input.test_cast<State_Predicate>());
            return (*this)(Fact(input), assignment);
        }
        break;
        case enum_types::state_proposition:
        {
            assert(input.test_cast<State_Proposition>());
            return (*this)(Ground_Fact(input), assignment);
        }
        break;
        case enum_types::perceptual_predicate:
        {
            assert(input.test_cast<Perceptual_Predicate>());
            return (*this)(Perception(input), assignment);
        }
        break;
        case enum_types::perceptual_proposition:
        {
            assert(input.test_cast<Perceptual_Proposition>());
            return (*this)(Ground_Perception(input), assignment);
        }
        break;
        case enum_types::equality_test:
        {
            assert(input.test_cast<Equality_Test>());
            return (*this)(Equality(input), assignment);
        }
        break;
        case enum_types::conjunction:
        {
            assert(input.test_cast<Conjunction>());
            return (*this)(Conjunct(input), assignment);
        }
        break;
        case enum_types::disjunction:
        {
            assert(input.test_cast<Disjunction>());
            return (*this)(Disjunct(input), assignment);
        }
        break;
        case enum_types::exists:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<input<<std::endl);
        }
        break;
        case enum_types::forall:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<input<<std::endl);
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Cannot ground formula ::"<<input<<std::endl);
        }
            break;
    }

}


// void Assignment_Applicator::reset__assignment_possibilities(std::map<Variable,  Constants&>& _assignment_possibilities)
// {
//     assignment_possibilities = _assignment_possibilities;
// }
