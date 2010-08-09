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

using namespace Planning;
using namespace Planning::State_Formula;

Planning_CNF__to__State_CNF::
Planning_CNF__to__State_CNF(basic_type::Runtime_Thread runtime_Thread,
                            Formula::State_Propositions& state_Propositions,
                            State_Formula::Literals& problem__literals,
                            State_Formula::Disjunctive_Clauses& problem__clauses,
                            State_Formula::Conjunctive_Normal_Form_Formulae& problem__cnfs)
    :problem__state_Propositions(state_Propositions),
     problem__literals(problem__literals),
     problem__clauses(problem__clauses),
     problem__cnfs(problem__cnfs),
     processing_negative(false),
     runtime_Thread(runtime_Thread)
{
}

State_Formula::Conjunctive_Normal_Form_Formula__Pointer Planning_CNF__to__State_CNF::get__answer() const
{
    return answer;
}


IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Planning_CNF__to__State_CNF,
                                            basic_type);


void Planning_CNF__to__State_CNF::operator()(Formula::Subformula input)
{
    switch(input->get__type_name()){
        case enum_types::state_proposition:
        {
            assert(input.test_cast<Planning::Formula::State_Proposition>());
            
            auto __proposition = input.cxx_get<Planning::Formula::State_Proposition>();

            
    
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Planning::Formula::State_Proposition,
                 _proposition,
                 __proposition->get__name(),
                 __proposition->get__arguments());

            
            INTERACTIVE_VERBOSER(true, 3104,
                                 "New state proposition :: "<<_proposition<<std::endl);
            
            auto _problem__state_Proposition = problem__state_Propositions
                .find(*_proposition.cxx_get<Planning::Formula::State_Proposition>());
            if(_problem__state_Proposition == problem__state_Propositions.end()){
                problem__state_Propositions
                    .insert(*_proposition.cxx_get<Planning::Formula::State_Proposition>());
                _problem__state_Proposition = problem__state_Propositions
                    .find(*_proposition.cxx_get<Planning::Formula::State_Proposition>());
            }
            auto proposition = *_problem__state_Proposition;
            
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
                problem__literals.insert(literal);
                _literal__pointer = problem__literals.find(literal);
            }
            auto literal__pointer = *_literal__pointer;
            
            if(clause__as_set.find(literal__pointer) != clause__as_set.end()){
                INTERACTIVE_VERBOSER(true, 3104,
                                     "Already got proposition :: "<<proposition
                                     <<" as literal :: "<<literal__pointer<<std::endl
                                     <<"Registered with the current clause being built.");
                return;
            }
            
            INTERACTIVE_VERBOSER(true, 3104,
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

            
            QUERY_UNRECOVERABLE_ERROR(
                !disjunctions.size(),
                "No disjunctions identified for formula while processnig :: "
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
                disjunctions = State_Formula::List__Disjunctive_Clause();
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
                (*clause).cxx_get<Disjunctive_Clause>()
                    ->add__listener(deref__st);
            }

            answer = problem__pointer;
            
            disjunctions = State_Formula::List__Disjunctive_Clause();
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
                true, 3104,
                "Processing disjunctive clause :: "<<_disjunct<<std::endl);
            
            auto disjunct = CXX__deref__shared_ptr<State_Formula::Disjunctive_Clause>(_disjunct);
            auto _clause__pointer = problem__clauses.find(disjunct);
            if(_clause__pointer != problem__clauses.end()){
            } else {
                problem__clauses.insert(disjunct);
                _clause__pointer = problem__clauses.find(disjunct);
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
            
            List__Literals& literals = clause__pointer->get__literals();
            for(auto literal = literals.begin()
                    ; literal != literals.end()
                    ; literal ++){
                INTERACTIVE_VERBOSER(true, 3104,
                                     "Registering clause as listener :: "<<clause__pointer<<std::endl
                                     <<"With literal"<<*literal<<std::endl );
                
                auto deref__st = clause__pointer.cxx_deref_get<basic_type>();
                (*literal).cxx_get<Literal>()->add__listener(deref__st);
            }
            
            
            INTERACTIVE_VERBOSER(true, 3104,
                                 "Added new CNF clause :: "<<clause__pointer<<std::endl);
                
            disjunctions.push_back(clause__pointer);
            disjunctions__as_set.insert(clause__pointer);
            
            clause = List__Literals();
            clause__as_set = Literals();
        }
        break;
        default:
            UNRECOVERABLE_ERROR("Non-CNF passed.");
            break;
    }
}

