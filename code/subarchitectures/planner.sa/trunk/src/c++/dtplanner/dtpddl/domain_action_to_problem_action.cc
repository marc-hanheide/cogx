
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

#include "domain_action_to_problem_action.hh"

#include "planning_cnf_to_state_cnf.hh"

/* Functionality for simplifying CNF formula. */
#include "turnstyle.hh"

using namespace Planning;
using namespace Planning::State_Formula;


Planning_Formula__to__CNF Domain_Action__to__Problem_Action::planning_Formula__to__CNF;


IMPLEMENTATION__STRICT_SHARED_UNARY_VISITOR(Domain_Action__to__Problem_Action,
                                            basic_type);



Domain_Action__to__Problem_Action::
Domain_Action__to__Problem_Action
(basic_type::Runtime_Thread runtime_Thread,
 Assignment& assignment,
 Formula::State_Propositions& state_Propositions,
 State_Formula::Literals& problem__literals,
 State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses,
 State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae,
 const Planning::Parsing::Domain_Data& _domain_Data,
 const Planning::Parsing::Problem_Data& _problem_Data,
 const Formula::Action_Proposition& action_Proposition,
 State_Formula::Conjunctive_Normal_Form_Formula__Pointer& precondition,
 Planning::State_Transformations& state_Transformations,
 State_Transformations& executable_actions_without_preconditions,
 Probabilistic_State_Transformations& probabilistic_actions)
    :runtime_Thread(runtime_Thread),
     assignment(assignment),
     problem__state_Propositions(state_Propositions),
     problem__literals(problem__literals),
     problem__disjunctive_Clauses(problem__disjunctive_Clauses),
     problem__conjunctive_Normal_Form_Formulae(problem__conjunctive_Normal_Form_Formulae),
     domain_Data(_domain_Data),
     problem_Data(_problem_Data),
     action_Proposition(action_Proposition),
     assignment_Applicator(reinterpret_cast<basic_type::Runtime_Thread>(&_problem_Data)
                           , _domain_Data
                           , _problem_Data),
     problem__actions(state_Transformations),
     executable_actions_without_preconditions(executable_actions_without_preconditions),
     probabilistic_actions(probabilistic_actions),
     processing_negative(false),
     count_of_actions_posted(0),
     level(0),
     probability(1.0)
{

    /* Some actions and transformations have void preconditions. Here
     * we store a single formula to represent that case. */
    List__Disjunctive_Clause list__Disjunctive_Clause;
    NEW_referenced_WRAPPED_deref_POINTER
        (runtime_Thread,
         State_Formula::Conjunctive_Normal_Form_Formula,
         _conjunct,
         list__Disjunctive_Clause);


    true_cnf = CXX__deref__shared_ptr<State_Formula::Conjunctive_Normal_Form_Formula>(_conjunct);


    /*Action precondition, for the ground action that this factory shall build.*/
    preconditions.push(precondition); /*(see \argument{precondition})*/
}


Planning::State_Transformation__Pointer Domain_Action__to__Problem_Action::get__answer() const 
{
    return result;
}

void Domain_Action__to__Problem_Action::operator()(Formula::Subformula input)
{
    switch(input->get__type_name()){
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

//             if(!input.cxx_get<Planning::Formula::Conjunction>()->get__subformulae().size()){
//                 return;
//             }
            

            /* PUSH LISTENERS */
            list__Listeners.push(State_Formula::List__Listeners());
            listeners.push(State_Formula::Listeners());
            
            /* PUSH LITERALS */
            literals_at_levels.push(State_Formula::List__Literals());
            
            /* PUSH PRECONDITIONS */
            preconditions.push(true_cnf);
            level++;
            deref_VISITATIONS(Planning::Formula::Conjunction, input, get__subformulae());
            level--;
            
            /* POP PRECONDITIONS */
            preconditions.pop();
            
            /* Action effects. Either an add or delete effect. */
            auto conjunction = literals_at_levels.top();

            /* Any of he subactions that this might trigger. */
            auto _list__Listeners = list__Listeners.top();

//             if(!conjunction.size() && !_list__Listeners.size()){
//                 literals_at_levels.pop();
//                 list__Listeners.pop();
//                 listeners.pop();
//                 assert(preconditions.size());

//                 return;
//             }
            
            assert(preconditions.size());
            State_Formula::Conjunctive_Normal_Form_Formula__Pointer
                _precondition = preconditions.top();

            std::ostringstream __new_action_name;
            Planning::Action_Name old__action_name = action_Proposition.get__name();
            Planning::Constant_Arguments arguments = action_Proposition.get__arguments();
            if(level == 0){
                __new_action_name<<old__action_name;
            } else {
                __new_action_name<<old__action_name
                              <<"+"
                              <<count_of_actions_posted++;
            }
            
            auto _new_action_name = __new_action_name.str();
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Planning::Action_Name
                                   , new__action_name
                                   , _new_action_name);
            
            NEW_referenced_WRAPPED(runtime_Thread
                                   , Formula::Action_Proposition
                                   , new__action_proposition
                                   , new__action_name
                                   , arguments);
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread
                 , State_Transformation
                 , _state_Transformation
                 , new__action_proposition
                 , _precondition
                 , conjunction
                 , (level == 0)?false:true
                 , false
                 , probability
                 , 0);

            
            
            auto problem_action = problem__actions
                .find(CXX__deref__shared_ptr<State_Transformation>(_state_Transformation));
            
            if(problem__actions.end() == problem_action){
                problem__actions.insert(CXX__deref__shared_ptr<State_Transformation>(_state_Transformation));
                problem_action = problem__actions
                    .find(CXX__deref__shared_ptr<State_Transformation>(_state_Transformation));
            }
            auto state_Transformation = *problem_action;
            
            result = state_Transformation;
            
            for(auto listener = _list__Listeners.begin()
                    ; listener != _list__Listeners.end()
                    ; listener++){

                /*An added listener could still be rejected at this
                 * point, if it was added on a previous run.*/
                state_Transformation
                    ->add__listener(*listener);
            }

            /*If the precondition is interesting.*/
            if(_precondition->get__disjunctive_clauses().size()){
                auto deref__st = state_Transformation.cxx_deref_get<basic_type>();
                _precondition->add__listener(deref__st);
            }

            executable_actions_without_preconditions.insert(state_Transformation);
            
            /* POP LITERALS */
            literals_at_levels.pop();
            
            /* POP LISTENERS */
            list__Listeners.pop();
            listeners.pop();
            
            assert(preconditions.size());

            if(list__Listeners.size()){
                assert(listeners.size());
                auto set_of_listeners = listeners.top();
                auto deref__st = state_Transformation.cxx_deref_get<basic_type>();
                if(set_of_listeners.find(deref__st) == set_of_listeners.end()){
                    auto list_of_listeners = list__Listeners.top();
                    list_of_listeners.push_back(deref__st);
                    set_of_listeners.insert(deref__st);   
                }
            }
            
            return;
        }
        break;
        case enum_types::state_proposition:
        {
            assert(input.test_cast<Planning::Formula::State_Proposition>());
            
            auto _proposition = input.cxx_get<Planning::Formula::State_Proposition>();
            
            NEW_referenced_WRAPPED_deref_POINTER
                (runtime_Thread,
                 Planning::Formula::State_Proposition,
                 proposition,
                 _proposition->get__name(),
                 _proposition->get__arguments());

            problem__state_Propositions
                .insert(*proposition.cxx_get<Planning::Formula::State_Proposition>());
            
            auto id = proposition->get__id();

            assert(id < problem__state_Propositions.size());
            
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
            
            literals_at_levels.top().push_back(literal__pointer);
            
            return;
        }
        break;
        case enum_types::state_predicate:
        {
            /* -- ground it and try again -- */
            assert(input.test_cast<Formula::State_Predicate>());
            auto predicate = input.cxx_get<Formula::State_Predicate>();

            auto argument_List = predicate->get__arguments();
            auto predicate_Name = predicate->get__name();
            
            Constant_Arguments constant_Arguments(argument_List.size());
            for(auto index = 0; index < argument_List.size(); index++){
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
                (runtime_Thread,
                 Planning::Formula::State_Proposition,
                 proposition,
                 predicate->get__name(),
                 constant_Arguments);

            (*this)(Formula::Subformula(proposition));
            
            return;
        }
        break;
        case enum_types::probabilistic_effect:
        {
            assert(level != 0);
            UNRECOVERABLE_ERROR("unimplemented.");
//             assert(input.test_cast<Formula::Probabilistic>());


//             for(){
//                 probability = ...;
//             }
            
            
//             probability = 1.0;
        }
        break;
        case enum_types::conditional_effect:
        {
            assert(level != 0);
            assert(input.test_cast<Formula::Conditional_Effect>());
            
            auto conditional_Effect = input.cxx_get<Formula::Conditional_Effect>();
            auto ___condition =  conditional_Effect->get__condition();
            auto __condition = simplify_formula(___condition, runtime_Thread);
            auto _condition = assignment_Applicator(__condition, assignment);

            /* If this condition is statically not executable.*/
            if(std::tr1::get<1>(_condition) == false){
                return;
            }
            auto condition = std::tr1::get<0>(_condition);
            if(enum_types::formula_false == condition->get__type_name()){
                return ;
            }

            

            
            auto _effect = conditional_Effect->get__effect();
            Formula::Subformula effect;
            if(enum_types::conjunction != _effect->get__type_name()){
                Formula::Subformulae elements;
                elements.push_back(_effect);
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (runtime_Thread,
                     Formula::Conjunction,
                     conjunction,
                     elements);

                effect = conjunction;
            } else {
                effect = _effect;
            }

            State_Formula::Conjunctive_Normal_Form_Formula__Pointer cnf_condition;
            if (enum_types::formula_true == condition->get__type_name()) {
                cnf_condition = true_cnf;
            } else {
                Planning_CNF__to__State_CNF
                    planning_CNF__to__State_CNF
                    (runtime_Thread
                     , problem__state_Propositions
                     , problem__literals
                     , problem__disjunctive_Clauses
                     , problem__conjunctive_Normal_Form_Formulae);
                

                planning_CNF__to__State_CNF(condition);

                cnf_condition = planning_CNF__to__State_CNF.get__answer();
            }

            level++;
//             listeners.push(Listeners());            /*searchable*/
//             list__Listeners.push(List__Listeners());/*traversable*/
            preconditions.push(cnf_condition);
            (*this)(effect);
            level--;
            
            preconditions.pop();
            return;
            
//             auto _action = problem__actions.find(result);
//             if(_action == problem__actions.end()){
//                 problem__actions.insert(result);
//                 _action = problem__actions.find(result);
//             }

//             assert(listeners.size());
//             assert(listeners.size() == list__Listeners.size());
            
//             auto listeners_iterator = listeners.top().find(_action.cxx_deref_get<basic_type>());
//             if(listeners.top().find(_action.cxx_deref_get<basic_type>()) == listeners.top.end()){
//                 listeners.top().insert(_action.cxx_deref_get<basic_type>());
//                 list__Listeners.top().push_back(_action.cxx_deref_get<basic_type>());
//             }
            
//             listeners.pop();
//             list__Listeners.pop();
            
        }
        break;
        case enum_types::increase:
        {
            UNRECOVERABLE_ERROR("unimplemented.");
        }
        break;
        case enum_types::decrease:
        {
            UNRECOVERABLE_ERROR("unimplemented.");
        }
        break;
        case enum_types::assign:
        {
            UNRECOVERABLE_ERROR("unimplemented.");
        }
        break;
        default:
        {
            UNRECOVERABLE_ERROR("Unable to generate actions from effect formula :: "<<input);
        }
        break;
        
    }
    
    
}




#define NEW_CONJUNCTION(NAME, INPUT)                                 \
    NEW_referenced_WRAPPED_deref_visitable_POINTER                   \
    (runtime_Thread                                                  \
     , Formula::Conjunction                                          \
     , NAME                                                          \
     , INPUT)                                                        \
        
#define NEW_DISJUNCTION(NAME, INPUT)                             \
    NEW_referenced_WRAPPED_deref_visitable_POINTER               \
    (runtime_Thread                                              \
     , Formula::Disjunction                                      \
     , NAME                                                      \
     , INPUT)                                                    \
        

#define NEW_NEGATION(NAME, INPUT)                       \
    NEW_referenced_WRAPPED_deref_visitable_POINTER      \
    (runtime_Thread                                     \
     , Formula::Negation                                \
     , NAME                                             \
     , INPUT)                                           \




Formula::Subformula Domain_Action__to__Problem_Action::
simplify_formula(Formula::Subformula subformula,
                 basic_type::Runtime_Thread runtime_Thread)
{
    /* \module{turnstyle} */
    using namespace Turnstyle;
    typedef CNF::Clause Clause;

    /* Atoms that formulate the \argument{action_Schema} preconditions*/
    std::vector<Formula::Subformula> atoms;

    /* For each atom, we map it to an integer index.*/
    std::map<Formula::Subformula, uint> atom_id;

    /* First step, we convert the formula into a CNF.*/
    CNF::Problem_Data problem_Data;

    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(subformula);

    if(precondition_as_cnf.test_cast<Formula::Vacuous>()){
        return subformula;
    }

    
    QUERY_UNRECOVERABLE_ERROR(!precondition_as_cnf.test_cast<Formula::Conjunction>(),
                              "Converted a formula :: "<<subformula<<std::endl
                              <<"to CNF format, but ended up with something that is not a conjunct.");
    
    /* Make sure that the conversion to CNF just undertaken has worked.*/
    auto _conjunction = precondition_as_cnf.do_cast<Formula::Conjunction>();

    /* Get the bubformulae associated with the conjunction. */
    auto conjunction = _conjunction->get__subformulae();
    assert(conjunction.size());
    for(auto __disjunction = conjunction.begin() /* The precondition is
                                                 * not in CNF format,
                                                 * therefore each
                                                 * subformulae must be
                                                 * a disjunction. */
            ; __disjunction != conjunction.end()
            ; __disjunction++){

        QUERY_UNRECOVERABLE_ERROR(
            !((*__disjunction).test_cast<Formula::Disjunction>()),
            "Formula :: "<<(*__disjunction)<<std::endl
            <<"is supposed to be an element in a CNF, however it is not a disjunction.\n");
        
//         auto _disjunction = (*__disjunction).do_cast<Disjunction>();
        
        auto disjunction = __disjunction->cxx_get<Formula::Disjunction>()->get__subformulae();//_disjunction->get__subformulae();

        Clause clause;
        for(auto _literal = disjunction.begin() /* Every element in
                                                 * the disjunction
                                                 * should be a literal
                                                 * -- positive or
                                                 * negative atom.*/
                ; _literal != disjunction.end()
                ; _literal++){

            bool negation = _literal->test_cast<Formula::Negation>();

            /* Get the atom associated with the literal.*/
            auto __atom = ((negation)
                           ?((_literal->do_cast<Formula::Negation>())->get__subformula())
                           :(*_literal));

            /* Get the integer we associate with this atom.*/
            auto _atom = atom_id.find(__atom);

            /* If that failed, then make a new integer associated with
             * that atom, and make sure the \type{Subformula}
             * associated with that atom is mapped to this new
             * integer/index in \local{atom_id}.*/
            if(_atom == atom_id.end()){
                auto index = atoms.size();
                atoms.push_back(__atom);
                atom_id[__atom] = index + 1;
                _atom = atom_id.find(__atom);
            }
            
            auto atom = _atom->second;

            assert(atom != 0);
            
            
            if(negation){
                clause.insert(-1 * atom);
            } else {
                clause.insert(atom);
            }

        }
        
        problem_Data.insert(clause);


        VERBOSER(3001, "Pushing clause :: "<<problem_Data<<std::endl);
    }
    

    VERBOSER(3001, "CNF problem data is :: "<<problem_Data<<std::endl);
    
    /* Construction of a CNF does implicit simplification of that CNF
     * data.*/
    CNF cnf(problem_Data);

    
    VERBOSER(3001, "CNF having been simplified is :: "<<cnf<<std::endl);

    
    
    Formula::Subformulae conjunctive_data;
    for(auto clause = cnf.problem_Data.begin()
            ; clause != cnf.problem_Data.end()
            ; clause++){

        
        Formula::Subformulae disjunctive_data;
        
        for(auto _literal = clause->begin()
                ; _literal != clause->end()
                ; _literal++){
            auto literal = *_literal;

            uint index = abs(literal);
            
            assert(index - 1 >= 0);
            assert(index - 1 < atoms.size());
            
            auto atom = atoms[index - 1];
            
            if(literal < 0){
                NEW_NEGATION(literal, atom);
                disjunctive_data.push_back(literal);
            } else if (literal > 0) {
                disjunctive_data.push_back(atom);
            } else {
                assert(0);
            }
        }
        NEW_DISJUNCTION(new_disjunction, disjunctive_data);
        
        conjunctive_data.push_back(new_disjunction);
    }
    
    NEW_CONJUNCTION(new_conjunction, conjunctive_data);

    
    VERBOSER(3001, "Original formula is :: "<<subformula<<std::endl);
    VERBOSER(3001, "translated formula is :: "<<precondition_as_cnf<<std::endl);
    VERBOSER(3001, "formula after simplification is :: "<<new_conjunction<<std::endl);

    return new_conjunction;
    
}
