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



#include "problem_grounding.hh"


#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"


/* Functionality for simplifying CNF formula. */
#include "turnstyle.hh"

using namespace Planning;

#define NEW_CONJUNCTION(NAME, INPUT)            \
NEW_referenced_WRAPPED_deref_POINTER            \
(runtime_Thread                                 \
 , Planning::Formula::Conjunction               \
 , NAME                                         \
 , INPUT)                                       \
        
#define NEW_DISJUNCTION(NAME, INPUT)            \
    NEW_referenced_WRAPPED_deref_POINTER        \
(runtime_Thread                                 \
 , Planning::Formula::Disjunction               \
 , NAME                                         \
 , INPUT)                                       \
        
    
#define NEW_NEGATION(NAME, INPUT)               \
    NEW_referenced_WRAPPED_deref_POINTER        \
(runtime_Thread                                 \
 , Planning::Formula::Negation                  \
 , NAME                                         \
 , INPUT)                                       \
    

Problem_Grounding::Problem_Grounding(Parsing::Problem_Data& problem_Data,
                                     CXX__PTR_ANNOTATION(Parsing::Domain_Data) domain_Data)
    :problem_Data(problem_Data),
     domain_Data(domain_Data)
{
    assert(domain_Data->get__action_Schemas().size());
    auto first_action = domain_Data->get__action_Schemas().begin();
    auto first_actionx_precondition = first_action->get__precondition();

    this->runtime_Thread = first_actionx_precondition->get__runtime_Thread();
}

void Problem_Grounding::ground_actions()
{

    Planning::Action_Schemas& schemas = domain_Data->get__action_Schemas();
    for(auto action = schemas.begin()
            ; action != schemas.end()
            ; action ++){
        ground_action_schema(const_cast<Planning::Action_Schema&>(*action));
    }
}

void Problem_Grounding::simplify_action_schema_precondition(Planning::Action_Schema& action_Schema)
{
    /* usual suspects C++*/
    using std::map;
    using std::vector;
    using std::string;

    /* \module{planning_formula}*/
    using Planning::Formula::Conjunction;
    using Planning::Formula::Disjunction;
    using Planning::Formula::Negation;
    using Planning::Formula::Subformula;
    using Planning::Formula::Subformulae;
    using Planning::Formula::Vacuous;

    /* \module{turnstyle} */
    using namespace Turnstyle;
    typedef CNF::Clause Clause;
    typedef CNF::Problem_Data Problem_Data;

    /* Atoms that formulate the \argument{action_Schema} preconditions*/
    vector<Subformula> atoms;

    /* For each atom, we map it to an integer index.*/
    map<Subformula, uint> atom_id;

    /* First step, we convert the formula into a CNF.*/
    CNF::Problem_Data problem_Data;

    /* If the action has no precondition.*/
    if(action_Schema.get__precondition().test_cast<Vacuous>()){
        return;
    }
    
    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(action_Schema.get__precondition());

    if(precondition_as_cnf.test_cast<Vacuous>()){
        return;
    }

    
    QUERY_UNRECOVERABLE_ERROR(!precondition_as_cnf.test_cast<Conjunction>(),
                              "Converted a formula :: "<<action_Schema.get__precondition()<<std::endl
                              <<"to CNF format, but ended up with something that is not a conjunct.");
    
    /* Make sure that the conversion to CNF just undertaken has worked.*/
    auto _conjunction = precondition_as_cnf.do_cast<Conjunction>();

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
            !((*__disjunction).test_cast<Disjunction>()),
            "Formula :: "<<(*__disjunction)<<std::endl
            <<"is supposed to be an element in a CNF, however it is not a disjunction.\n");
        
//         auto _disjunction = (*__disjunction).do_cast<Disjunction>();
        
        auto disjunction = __disjunction->cxx_get<Disjunction>()->get__subformulae();//_disjunction->get__subformulae();

        Clause clause;
        for(auto _literal = disjunction.begin() /* Every element in
                                                 * the disjunction
                                                 * should be a literal
                                                 * -- positive or
                                                 * negative atom.*/
                ; _literal != disjunction.end()
                ; _literal++){

            bool negation = _literal->test_cast<Negation>();

            /* Get the atom associated with the literal.*/
            auto __atom = ((negation)
                           ?((_literal->do_cast<Negation>())->get__subformula())
                           :(*_literal));

            /* Get teh integer we associate with this atom.*/
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

        
        std::cerr<<"Pushing clause :: "<<problem_Data<<std::endl;
        {char ch; std::cin>>ch;}
    }
    

    VERBOSER(3001, "CNF problem data is :: "<<problem_Data<<std::endl);
    
    /* Construction of a CNF does implicit simplification of that CNF
     * data.*/
    CNF cnf(problem_Data);

    
    VERBOSER(3001, "CNF having been simplified is :: "<<cnf<<std::endl);

    {char ch; std::cin>>ch;}
    
    
    Subformulae conjunctive_data;
    for(auto clause = cnf.problem_Data.begin()
            ; clause != cnf.problem_Data.end()
            ; clause++){

        
        Subformulae disjunctive_data;
        
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

    
    VERBOSER(3001, "Original formula is :: "<<action_Schema.get__precondition()<<std::endl);
    VERBOSER(3001, "translated formula is :: "<<precondition_as_cnf<<std::endl);
    VERBOSER(3001, "formula after simplification is :: "<<new_conjunction<<std::endl);
    {char ch; std::cin>>ch;}
    
    action_Schema.alter__precondition(new_conjunction);
}

void Problem_Grounding::ground_action_schema(Planning::Action_Schema& action_Schema)
{
    /* First step, we alter the action precondition formula so that it
     * corresponds to propositional CNF.*/
    simplify_action_schema_precondition(action_Schema);
    
}

