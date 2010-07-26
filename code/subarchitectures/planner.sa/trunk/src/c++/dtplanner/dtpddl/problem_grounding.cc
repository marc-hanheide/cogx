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

/* usual suspects C++*/
using std::map;
using std::vector;
using std::string;
using std::list;

/* \module{planning_formula}*/
using Planning::Formula::Conjunction;
using Planning::Formula::Disjunction;
using Planning::Formula::Negation;
using Planning::Formula::Subformula;
using Planning::Formula::Subformulae;
using Planning::Formula::Vacuous;
using Planning::Derived_Predicate;


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
                                     CXX__PTR_ANNOTATION(Parsing::Domain_Data) domain_Data,
                                     Planning::Parsing::Constants_Data::Constants_Description& constants_Description)
    :problem_Data(problem_Data),
     domain_Data(domain_Data),
     constants_Description(constants_Description)
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


void  Problem_Grounding::ground_derived_predicates()
{
    auto derived_Predicates = domain_Data->get__derived_Predicates();

    for(auto derived_Predicate = derived_Predicates.begin()
            ; derived_Predicate != derived_Predicates.end()
            ; derived_Predicate++){
        ground_derived_predicate_schema(const_cast<Planning::Derived_Predicate&>(*derived_Predicate));
    }   
}

void  Problem_Grounding::ground_derived_perceptions()
{
    WARNING("Grounding of derived perception predicates is being called, but is redundant.");
    
    auto derived_Percepts = domain_Data->get__derived_Percepts();

    for(auto derived_Percept = derived_Percepts.begin()
            ; derived_Percept != derived_Percepts.end()
            ; derived_Percept++){
        ground_derived_percept_schema(const_cast<Planning::Derived_Percept&>(*derived_Percept));
    }
}

Subformula Problem_Grounding::simplify_formula(Planning::Formula::Subformula subformula)
{
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

    
    /* Try to convert the precondition formula into a CNF. */
    auto precondition_as_cnf = planning_Formula__to__CNF(subformula);//action_Schema.get__precondition());

    if(precondition_as_cnf.test_cast<Vacuous>()){
        return subformula;
    }

    
    QUERY_UNRECOVERABLE_ERROR(!precondition_as_cnf.test_cast<Conjunction>(),
                              "Converted a formula :: "<<subformula<<std::endl
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

    
    VERBOSER(3001, "Original formula is :: "<<subformula<<std::endl);
    VERBOSER(3001, "translated formula is :: "<<precondition_as_cnf<<std::endl);
    VERBOSER(3001, "formula after simplification is :: "<<new_conjunction<<std::endl);
    {char ch; std::cin>>ch;}

    return new_conjunction;
}

void Problem_Grounding::simplify_action_schema_precondition(Planning::Action_Schema& action_Schema)
{
    /* If the action has no precondition.*/
    if(action_Schema.get__precondition().test_cast<Vacuous>()){
        return;
    }
    
    auto new_conjunction = simplify_formula(action_Schema.get__precondition());
    
    action_Schema.alter__precondition(new_conjunction);
}

void Problem_Grounding::
ground_action_schema(list<Constant>& ordereed_assignment,/*result, an assignment*/
                     map<Variable, Constant>& assignment_detail, /*explicit representation of results*/
                     const map<Variable, Constants&>& potential_assignments, /* constants from which the result is formed.*/
                     const Variables& action_variables, /*Gives the order in which variables assignment should be made.*/
 )
{
    complete(assignment)
    
}

void Problem_Grounding::grow__cached_constants_of_types(const Types& types)
{
    Constants constants;
    
    for(auto type = types.begin()
            ; type != types.end()
            ; type++){
        auto consts = extensions_of_types[type];
        for(auto c = consts.begin()
                ; c != consts.end()
                ; c++){
            constants.insert(*c);
        }        
    }

    cached_constants_of_types[types] = std::tr1::move(constants);
}

void Problem_Grounding::grow__cached_constants_of_types(const Argument_Types& argument_Types)
{
    for(auto arg_Types = argument_Types.begin()
            ; arg_Types != argument_Types.end()
            ; arg_Types++){
        if(arg_Types->size() == 1) continue;
        assert(0 != arg_Types->size());
        if(cached_constants_of_types.find(*arg_Types) == cached_constants_of_types.end()){
            grow__cached_constants_of_types(*arg_Types)
        }
    }
}


void Problem_Grounding::ground_action_schema(Planning::Action_Schema& action_Schema)
{
    /* First step, we alter the action precondition formula so that it
     * corresponds to propositional CNF.*/
    simplify_action_schema_precondition(action_Schema);
    
    auto action_headder = action_Schema.get__header();
    auto action_Name = action_headder.get__name();
    auto arguments = action_headder.get__arguments();
    auto variables = get__symbols(arguments);
    auto argument_Types = get__types(arguments);
    
    grow__cached_constants_of_types(argument_Types);
    
    map<Variable,  Constants&> potential_assignments;
    assert(argument_Types .size() == variables.size());
    for(uint index = 0; index < argument_Types.size(); index++){
        auto types = argument_Types[index];
        auto variable = variables[index];

        if(types.size() == 1){
            potential_assignments[variable] = *extensions_of_types.find(*types.begin());
        } else {
            assert(types.size());
            assert(cached_constants_of_types.end() != cached_constants_of_types.find(types));
            potential_assignments[variable] = *cached_constants_of_types.find(types);
        }
    }

    list<Constant> arguments;
    map<Variable, Constant> assignment_detail;
    
    ground_action_schema(arguments,
                         assignment_detail,
                         potential_assignments,
                         variables);
    
}

void Problem_Grounding::simplify_derived_predicate_trigger(Planning::Derived_Predicate& derived_Predicate)
{
    
    auto new_conjunction = simplify_formula(derived_Predicate.get__formula());
    derived_Predicate.alter__formula(new_conjunction);
}

void Problem_Grounding::ground_derived_predicate_schema(Planning::Derived_Predicate& derived_Predicate)
{
    simplify_derived_predicate_trigger(derived_Predicate);
}


void Problem_Grounding::simplify_derived_percept_trigger(Planning::Derived_Percept& derived_Percept)
{
    
    auto new_conjunction = simplify_formula(derived_Percept.get__formula());
    derived_Percept.alter__formula(new_conjunction);
}

void Problem_Grounding::ground_derived_percept_schema(Planning::Derived_Percept& derived_Percept)
{
   
    simplify_derived_percept_trigger(derived_Percept); 
}
