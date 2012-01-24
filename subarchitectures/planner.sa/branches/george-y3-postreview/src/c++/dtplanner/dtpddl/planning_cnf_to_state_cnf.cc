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


#include "planning_cnf_to_state_cnf.hh"

#include "planning_state.hh"
#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

#include "dtp_pddl_parsing_data_domain.hh"


using namespace Planning;
using namespace Planning::State_Formula;

Planning_CNF__to__State_CNF::
Planning_CNF__to__State_CNF(basic_type::Runtime_Thread runtime_Thread,
                            Formula::State_Propositions& state_Propositions,
                            State_Formula::Literals& problem__literals,
                            State_Formula::Disjunctive_Clauses& problem__clauses,
                            State_Formula::Conjunctive_Normal_Form_Formulae& problem__cnfs,
                            const Planning::Parsing::Problem_Data& _problem_Data)
    :problem__state_Propositions(state_Propositions),
     problem__literals(problem__literals),
     problem__clauses(problem__clauses),
     problem__cnfs(problem__cnfs),
     processing_negative(false),
     runtime_Thread(runtime_Thread),
     problem_Data(_problem_Data)
{
}

State_Formula::Conjunctive_Normal_Form_Formula__Pointer Planning_CNF__to__State_CNF::get__answer() const
{
    return answer;
}



bool Planning_CNF__to__State_CNF::flipped_once(const Planning::Formula::State_Proposition& fact, bool sign)
{
    auto& domain_Data = *problem_Data.get__domain_Data();

    if(!sign){/*positive -- i.e., NOT \member{processing_negative}.*/
        if(!domain_Data.in_add_effect(fact.get__name())){
            /* If it is false in the starting state, then
             * \argument{fact} should not be here. If \argument{fact}
             * is true in the starting state, and it is flipped, then
             * it is made false. At this point, it can never be made
             * true again.*/
            
            return true;
        }
    } else {
        if(!domain_Data.in_delete_effect(fact.get__name())){
            /* If it is true in the starting state, then
             * \argument{fact} should not be here. If \argument{fact}
             * is flase in the starting state, and it is flipped, then
             * it is made true. At this point, it can never be made
             * false again.*/
            return true;
        }
    }

    return false;
}



IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Planning_CNF__to__State_CNF,
                                            basic_type)


void Planning_CNF__to__State_CNF::operator()(const Formula::Subformula& input)
{
    switch(input->get__type_name()){
        case enum_types::state_proposition:
        {
            assert(input.test_cast<Planning::Formula::State_Proposition>());
            
            auto __proposition = input.cxx_get<Planning::Formula::State_Proposition>();

            Constant_Arguments constant_Arguments;
            auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
            auto arguments = __proposition->get__arguments();
            bool no_spurious_constants = true;
            for(auto argument = arguments.begin()
                    ; argument != arguments.end()
                    ; argument++ ){
                if(argument->get__runtime_Thread() != problem_thread) {
                    no_spurious_constants = false;
                    break;
                }
            }

            QUERY_WARNING(!no_spurious_constants,
                          "Was not expecting constant symbols from :: "<<*__proposition);
            
//             assert(no_spurious_constants);
            
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
                    
                    INTERACTIVE_VERBOSER(true, 10010, "New constant :: "<<constant_Arguments[index]<<std::endl);
                }
            }

            Formula::State_Proposition _proposition;
            if(!no_spurious_constants){
                assert(problem_thread == reinterpret_cast<basic_type::Runtime_Thread>
                       (dynamic_cast<const Planning::Parsing::Constants_Data*>(&problem_Data)));
        
                NEW_referenced_WRAPPED
                    (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
                     , Planning::Formula::State_Proposition
                     , proposition
                     , __proposition->get__name()
                     , constant_Arguments);

                
                problem__state_Propositions
                    .insert(proposition);
                _proposition = proposition;
            } else {
                
                NEW_referenced_WRAPPED
                    (runtime_Thread//dynamic_cast<const Planning::Parsing::Formula_Data*>(&problem_Data)
                     , Planning::Formula::State_Proposition
                     , proposition
                     , __proposition->get__name()
                     , arguments);

                
                problem__state_Propositions
                    .insert(proposition);
                
                _proposition = proposition;
            }
            
            
            
            
//             Constant_Arguments constant_Arguments;
//             auto problem_thread = reinterpret_cast<basic_type::Runtime_Thread>(&problem_Data);
//             bool no_spurious_constants = true;
//             for(auto argument = __proposition->get__arguments.begin()
//                     ; argument != __proposition->get__arguments.end()
//                     ; argument++ ){
//                 if(argument->get__runtime_Thread() != problem_thread) {
//                     no_spurious_constants = false;
//                     break;
//                 }
//             }

//             assert(no_spurious_constants);
            
            

            
            INTERACTIVE_VERBOSER(true, 10004,
                                 "New state proposition :: "<<_proposition<<std::endl);
            
            auto _problem__state_Proposition = problem__state_Propositions
                .find(_proposition);//*_proposition.cxx_get<Planning::Formula::State_Proposition>());
            if(_problem__state_Proposition == problem__state_Propositions.end()){
                problem__state_Propositions
                    .insert(_proposition);//*_proposition.cxx_get<Planning::Formula::State_Proposition>());
                _problem__state_Proposition = problem__state_Propositions
                    .find(_proposition);//*_proposition.cxx_get<Planning::Formula::State_Proposition>());
            }
            auto proposition = *_problem__state_Proposition;
            
            INTERACTIVE_VERBOSER(true, 10004,
                                 "New state proposition :: "<<proposition<<std::endl);
            
            auto id = proposition.get__id();
            
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
                /*GARDED*/        ; tmp_literal != problem__literals.end()
                /*GARDED*/        ; tmp_literal++){
                /*GARDED*/    /*GARDED*/std::cerr<<(*tmp_literal)->get__runtime_Thread()<<"::"<<*tmp_literal<<std::endl;
                /*GARDED*/}
#endif 
#endif 
#endif 
                
                INTERACTIVE_VERBOSER(true, 11000, "Adding new problem literal :: "
                                     <<literal->get__runtime_Thread()<<"::"<<literal<<std::endl);
                
                problem__literals.insert(literal);
                _literal__pointer = problem__literals.find(literal);
            }
            auto literal__pointer = *_literal__pointer;
            
            literal__pointer->configure__complement(literal__pointer, problem__literals);
            literal__pointer->set__can_only_be_flipped_once(flipped_once(proposition, processing_negative));

            INTERACTIVE_VERBOSER(literal__pointer->get__can_only_be_flipped_once()
                                 , 11000
                                 , literal__pointer<<" can only have its truth value changed once.");
            
            
            if(clause__as_set.find(literal__pointer) != clause__as_set.end()){
                INTERACTIVE_VERBOSER(true, 3124,
                                     "Already got proposition :: "<<proposition
                                     <<" as literal :: "<<literal__pointer<<std::endl
                                     <<"Registered with the current clause being built.");
                return;
            }
            
            INTERACTIVE_VERBOSER(true, 3124,
                                 "New proposition :: "<<proposition
                                 <<" as literal :: "<<literal__pointer<<std::endl
                                 <<"Now registered with the current clause being built.");
            
            clause.push_back(literal__pointer);
            clause__as_set.insert(literal__pointer);
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
            assert(input.test_cast<Planning::Formula::Conjunction>());
            deref_VISITATIONS(Planning::Formula::Conjunction, input, get__subformulae());


            
            QUERY_WARNING(
                !disjunctions.size(),
                "No disjunctions identified for formula while processing :: "
                <<input<<std::endl
                <<disjunctions__as_set);

            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 State_Formula::Conjunctive_Normal_Form_Formula,
                 _conjunct,
                 disjunctions);
            

            auto formula =
                CXX__deref__shared_ptr<State_Formula::Conjunctive_Normal_Form_Formula>
                (_conjunct);
            
            auto _problem__pointer = problem__cnfs.find(formula);
            

            /* If this CNF is already in the ground instance. */
            if(_problem__pointer != problem__cnfs.end()){
                answer = *_problem__pointer;
                disjunctions = State_Formula::List__Disjunctive_Clauses();
                disjunctions__as_set = State_Formula::Disjunctive_Clauses();
                return;
            }
            
            if(_problem__pointer == problem__cnfs.end()){
                problem__cnfs.insert(formula);
                _problem__pointer = problem__cnfs.find(formula);
            }
            auto problem__pointer = *_problem__pointer;

            
            auto clauses = problem__pointer->get__disjunctive_clauses();
            for(auto clause = clauses.begin()
                    ; clause != clauses.end()
                    ;  clause++){
                auto deref__st = problem__pointer.cxx_deref_get<basic_type>();

                
                if((*clause).cxx_get<Disjunctive_Clause>()
                   ->add__listener(deref__st)){
                    
                
                    INTERACTIVE_VERBOSER(true, 9091, "successfully added CNF listener "<<*deref__st.get()
                                         <<" to clause :: "
                                         <<(*clause)<<std::endl);
                }
                
            }

            answer = problem__pointer;
            
            disjunctions = State_Formula::List__Disjunctive_Clauses();
            disjunctions__as_set = State_Formula::Disjunctive_Clauses();   
        }
        break;
        case enum_types::disjunction:
        {
            assert(input.test_cast<Planning::Formula::Disjunction>());
            deref_VISITATIONS(Planning::Formula::Disjunction, input, get__subformulae());

            QUERY_UNRECOVERABLE_ERROR(
                !clause.size(),
                "Got an empty clause while processnig :: "<<input);
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 State_Formula::Disjunctive_Clause,
                 _disjunct,
                 clause);

            INTERACTIVE_VERBOSER(
                true, 3124,
                "Processing disjunctive clause :: "<<_disjunct<<std::endl);
            
            auto disjunct = CXX__deref__shared_ptr<State_Formula::Disjunctive_Clause>(_disjunct);
            auto _clause__pointer = problem__clauses.find(disjunct);
            
            bool is_new_planning_problem_clause = false;
            if(_clause__pointer != problem__clauses.end()){
                INTERACTIVE_VERBOSER(
                    true, 3120,
                    "Repeated problem clause :: "<<*_clause__pointer<<std::endl);
            } else {
                is_new_planning_problem_clause = true;
                problem__clauses.insert(disjunct);
                _clause__pointer = problem__clauses.find(disjunct);
                INTERACTIVE_VERBOSER(
                    true, 3120,
                    "New problem clause :: "<<*_clause__pointer<<std::endl);
            }
            
            auto clause__pointer = *_clause__pointer;


            /* If it is already in the problem. */
            if(disjunctions__as_set.find(clause__pointer) != disjunctions__as_set.end()){
                INTERACTIVE_VERBOSER(true, 3104,
                                     "Processing duplicated clause :: "<<clause__pointer<<std::endl
                                     <<"Reacted by dropping back to processing conjunct."<<std::endl );
                clause = List__Literals();
                clause__as_set = Literals();
                return;
            }

            if(is_new_planning_problem_clause){
                List__Literals& literals = clause__pointer->get__literals();
                for(auto literal = literals.begin()
                        ; literal != literals.end()
                        ; literal ++){
                    INTERACTIVE_VERBOSER(true, 3120,
                                         "Input formula is :: "<<input<<std::endl
                                         <<"Registering clause as listener :: "<<clause__pointer<<std::endl
                                         <<"With literal"<<*literal<<std::endl );
                    
                    auto deref__st = clause__pointer.cxx_deref_get<basic_type>();
                    if((*literal).cxx_get<Literal>()->add__listener(deref__st)){;
                        
                        INTERACTIVE_VERBOSER(true, 9091, "successfully added clause listener "<<*deref__st.get()
                                             <<" to literal :: "
                                             <<(*literal)<<std::endl);
                        
                    } else {
                        WARNING("Input formula is :: "<<input<<std::endl
                                <<"Could not register clause :: "<<deref__st<<std::endl
                                <<"as a listener to literal :: "<<*literal<<std::endl
                                <<"apparently because that registration has already occurred.");
                    }
                }
            }
            
            INTERACTIVE_VERBOSER(true, 3120,
                                 "Added new CNF clause :: "<<clause__pointer<<std::endl);
                
            disjunctions.push_back(clause__pointer);
            disjunctions__as_set.insert(clause__pointer);
            
            clause = List__Literals();
            clause__as_set = Literals();
        }
        break;
        case enum_types::vacuous:
        {
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
            UNRECOVERABLE_ERROR("Non-CNF passed.");
            break;
    }
}

