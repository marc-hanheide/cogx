#include "planning_cnf_to_action_cnf.hh"


#include "planning_state.hh"
#include "action__literal.hh"
#include "action__disjunctive_clause.hh"
#include "action__conjunctive_normal_form_formula.hh"
#include "action__state_transformation.hh"

using namespace Planning;

Planning_CNF__to__Action_CNF::
Planning_CNF__to__Action_CNF(basic_type::Runtime_Thread runtime_Thread,
                             Action_Literals& problem__action_literals,
                             Action_Disjunctive_Clauses& problem__action_clauses,
                             Action_Conjunctive_Normal_Form_Formulae& problem__action_cnfs,
//                              CXX__PTR_ANNOTATION(List__Action_Literals)& problem__negative_literals,
                             const std::map<Formula::Action_Proposition
                             , State_Transformation__Pointer>& action_symbol__to__state_transformation)
    :problem__literals(problem__action_literals),
     problem__clauses(problem__action_clauses),
     problem__cnfs(problem__action_cnfs),
//      problem__negative_literals(problem__negative_literals),
     action_symbol__to__state_transformation(action_symbol__to__state_transformation),
     processing_negative(false),
     runtime_Thread(runtime_Thread)
{
}

Action_Conjunctive_Normal_Form_Formula__Pointer Planning_CNF__to__Action_CNF::get__answer() const
{
    return answer;
}


IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Planning_CNF__to__Action_CNF,
                                            basic_type);


void Planning_CNF__to__Action_CNF::operator()(const Formula::Subformula& input)
{
    switch(input->get__type_name()){
        case enum_types::action_proposition:
        {
            assert(input.test_cast<Planning::Formula::Action_Proposition>());
            
            auto _proposition = input.cxx_get<Planning::Formula::Action_Proposition>();

            auto proposition = *_proposition;

            if( (proposition.get__runtime_Thread() != runtime_Thread)
                && !proposition.get__arguments().size() ){
                
                NEW_referenced_WRAPPED_deref_POINTER
                    ( runtime_Thread
                      , Planning::Formula::Action_Proposition
                      , tmp
                      , proposition.get__name()
                      , proposition.get__arguments());
                proposition = *tmp.cxx_get<Planning::Formula::Action_Proposition>();
            }
            
            
            QUERY_UNRECOVERABLE_ERROR(proposition.get__runtime_Thread() != runtime_Thread
                                      , "Proposition :: "<<proposition<<std::endl
                                      <<"Has the wrong runtime thread :: "
                                      <<proposition.get__runtime_Thread()<<std::endl
                                      <<"Expecting :: "<<runtime_Thread<<std::endl);
            
            
            
            auto id = proposition.get__id();
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Action_Literal,
                 _literal,
                 id,
                 processing_negative);
            
            auto literal = CXX__deref__shared_ptr<Action_Literal>(_literal);
            
            auto _literal__pointer = problem__literals.find(literal);
            if(_literal__pointer == problem__literals.end()){
                problem__literals.insert(literal);
                
//                 if(processing_negative){
//                     problem__negative_literals->push_back(literal);
//                 }
                
                _literal__pointer = problem__literals.find(literal);
            }
            auto literal__pointer = *_literal__pointer;

            
            QUERY_UNRECOVERABLE_ERROR(!action_symbol__to__state_transformation.size(),
                                      "Got zero actions, therefore observations cannot be triggered.");
            
            QUERY_UNRECOVERABLE_ERROR(action_symbol__to__state_transformation.begin()->first.get__runtime_Thread()
                                      != proposition.get__runtime_Thread(),
                                      "Oops, asking for action literals from the wrong thread.");

//             for(auto p = action_symbol__to__state_transformation.begin()
//                     ; p != action_symbol__to__state_transformation.end()
//                     ; p++){
                
//                 std::cerr<<p->first<<std::endl;// <<"\n"<<p->first.get__runtime_Thread()<<"::"<<p->first.get__id()<<std::endl
// //                          <<proposition<<"\n"<<proposition.get__runtime_Thread()<<"::"<<proposition.get__id()
// //                          <<std::endl<<std::endl;
                
// //                 std::ostringstream oss1;
// //                 oss1<<p->first;
// //                 std::string string1 = oss1.str();
                
// //                 std::ostringstream oss2;
// //                 oss2<<proposition;
// //                 std::string string2 = oss2.str();
                
                
// //                 if(string2 == string1){
                    
// //                     assert(p->first.get__arguments() == proposition.get__arguments());

// //                     std::cerr<<p->first.get__name().get__runtime_Thread()<<" "
// //                              <<proposition.get__name().get__runtime_Thread();
                    
// //                     assert(p->first.get__name() == proposition.get__name());
// //                     assert(p->first == proposition);
// //                 }
//             }
//             std::cerr<<std::endl;


            /*If a positive version of this literal cannot ever be satisfied.*/
            if(action_symbol__to__state_transformation.find(proposition) ==
               action_symbol__to__state_transformation.end()){
                WARNING("Action :: "<<proposition<<" cannot be executed.");
                
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
                
                INTERACTIVE_VERBOSER(true, 16000, "Got an unsatisfiable action literal."<<std::endl);
                return;
            }
            
            
            QUERY_UNRECOVERABLE_ERROR
                (action_symbol__to__state_transformation.find(proposition) ==
                 action_symbol__to__state_transformation.end(),
                 "Could not find transformation for :: "<<proposition<<" "<<proposition.get__runtime_Thread()<<std::endl);
            
            
            /* Ensure the corresponding action wakes this literal on
             * execution; if it is positive.*/
            if(!processing_negative){
                auto state_Transformation = action_symbol__to__state_transformation.find(proposition)->second;
                auto deref__st = literal__pointer.cxx_deref_get<basic_type>();

                
                INTERACTIVE_VERBOSER(true, 16000, "Action :: "
                                     <<state_Transformation->get__identifier()<<std::endl
                                     <<"Is supposed to wake :: "<<literal__pointer<<std::endl);
                
                if(!state_Transformation->add__sleeper(deref__st)){
                    WARNING("Could not add :: "<<literal__pointer<<" to action labelled"
                            <<state_Transformation->get__identifier()<<std::endl);
                }
                
                
                INTERACTIVE_VERBOSER(true, 16000, "Testing if registration of sleeper worked :: "
                                     <<*state_Transformation<<std::endl);
                
//                 state_Transformation->add__listener(deref__st);
            }
            
            literal__pointer->configure__complement(literal__pointer, problem__literals);
//             literal__pointer->configure__negatives(/*literal__pointer,*/ problem__negative_literals);
            
            if(clause__as_set.find(literal__pointer) != clause__as_set.end()){
                INTERACTIVE_VERBOSER(true, 16000,
                                     "Already got proposition :: "<<proposition
                                     <<" as literal :: "<<literal__pointer<<std::endl
                                     <<"Registered with the current clause being built.");
                return;
            }
            
            INTERACTIVE_VERBOSER(true, 16000,
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
                 Action_Conjunctive_Normal_Form_Formula,
                 _conjunct,
                 disjunctions);
            

            auto formula =
                CXX__deref__shared_ptr<Action_Conjunctive_Normal_Form_Formula>
                (_conjunct);
            
            auto _problem__pointer = problem__cnfs.find(formula);
            

            /* If this CNF is already in the ground instance. */
            if(_problem__pointer != problem__cnfs.end()){
                answer = *_problem__pointer;
                disjunctions = List__Action_Disjunctive_Clauses();//Action_List__Disjunctive_Clause();
                disjunctions__as_set = Action_Disjunctive_Clauses();//Action_Disjunctive_Clauses();   
//                 disjunctions = Action_List__Disjunctive_Clause();
//                 disjunctions__as_set = Action_Disjunctive_Clauses();
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
                (*clause).cxx_get<Action_Disjunctive_Clause>()
                    ->add__listener(deref__st);
            }

            answer = problem__pointer;
            
            disjunctions = List__Action_Disjunctive_Clauses();//Action_List__Disjunctive_Clause();
            disjunctions__as_set = Action_Disjunctive_Clauses();//Action_Disjunctive_Clauses();   
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
                 Action_Disjunctive_Clause,
                 _disjunct,
                 clause);

            INTERACTIVE_VERBOSER(
                true, 3124,
                "Processing disjunctive clause :: "<<_disjunct<<std::endl);
            
            auto disjunct = CXX__deref__shared_ptr<Action_Disjunctive_Clause>(_disjunct);
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
                clause = List__Action_Literals();
                clause__as_set = Action_Literals();
//                 clause = Action_List__Literals();
//                 clause__as_set = Action_Literals();
                return;
            }

            if(is_new_planning_problem_clause){
                List__Action_Literals& literals = clause__pointer->get__literals();
                for(auto literal = literals.begin()
                        ; literal != literals.end()
                        ; literal ++){
                    INTERACTIVE_VERBOSER(true, 3120,
                                         "Input formula is :: "<<input<<std::endl
                                         <<"Registering clause as listener :: "<<clause__pointer<<std::endl
                                         <<"With literal"<<*literal<<std::endl );
                    
                    auto deref__st = clause__pointer.cxx_deref_get<basic_type>();
                    if((*literal).cxx_get<Action_Literal>()->add__listener(deref__st)){;
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
            
            clause = List__Action_Literals();
            clause__as_set = Action_Literals();
//             clause = List__Action_Literals();
//             clause__as_set = Action_Literals();
        }
        break;
        default:
            UNRECOVERABLE_ERROR("Non-CNF passed.");
            break;
    }
}

