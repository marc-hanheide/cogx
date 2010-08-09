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

#ifndef PROBLEM_GROUNDING_HH
#define PROBLEM_GROUNDING_HH


#include "solver_basics.hh"
#include "dtp_pddl_parsing_data_constants.hh"

#include "planning_action_schema.hh"
#include "planning_derived_predicate.hh"

// #include "basic_action.hh"

#include "planning_formula_to_cnf.hh"
#include "assignment_applicator.hh"

#include "action_basics.hh"
#include "state_basics.hh"

namespace Planning
{
    class Problem_Grounding
    {
    public:
        Problem_Grounding(Parsing::Problem_Data& problem_Data,
                          CXX__PTR_ANNOTATION(Parsing::Domain_Data) domain_Data,
                          const Planning::Constants_Description& constants_Description,
                          const std::map<Type, Constants>& extensions_of_types);

        void ground_actions();
        void ground_derived_predicates();
        void ground_derived_perceptions();

    private:
        /* All the state formula associated with the ground problem
         * instance (see \member{press_ground_action}).*/
        Formula::State_Propositions state_Propositions;
        State_Formula::Literals literals;
        State_Formula::Disjunctive_Clauses disjunctive_Clauses;
        State_Formula::Conjunctive_Normal_Form_Formulae conjunctive_Normal_Form_Formulae;

        
        State_Transformations deterministic_actions;
        State_Transformations executable_actions_without_preconditions;
        Probabilistic_State_Transformations probabilistic_actions;
        
        /* A tool to apply an assignment to variables to a CNF formula. */
        CNF_Assignment_Applicator assignment_Applicator;
        
        /* Incrementally add entries to \member{cached_constants_of_types}.*/
        void grow__cached_constants_of_types(const Types&);
        /* Incrementally add entries to \member{cached_constants_of_types}.*/
        void grow__cached_constants_of_types(const Argument_Types&);
        
        /* Some variables take values from a domain definition as the
         * union of multiple type classes. Here we cache the domain
         * objects from unions that appear in the domain
         * definition.*/
        std::map<Types, Constants> cached_constants_of_types;

        /* Description of the problem constants (see \module{Solver}).*/
        const Planning::Constants_Description& constants_Description;

        /* Constants that types can take (see \module{Solver}). This is the
         * basis of data in \member{cached_constants_of_types}.*/
        const std::map<Type, Constants>& extensions_of_types;
        
        /* Object converts planning formula ---in this case formulae
         * that correspond to actions preconditions--- into
         * Conjunctive Normal Form formula.  Such a formula is a
         * conjunction over disjunctive clauses.  */
        Planning::Planning_Formula__to__CNF planning_Formula__to__CNF;

        /* The thread with which the domain actions are linked.*/
        basic_type::Runtime_Thread runtime_Thread;

        /* Basis for \member{simplify_action_schema_precondition} and
         * \member{simplify_derived_predicate_trigger}. Simply turns
         * quantifier free first-order formula into a CNF form that
         * has no redundancy.  */
        Planning::Formula::Subformula simplify_formula(Planning::Formula::Subformula);
            
        /* Simplify the description of the \argument{Action_Schema} precondition.*/
        void simplify_action_schema_precondition(Planning::Action_Schema& );
        
        /* Populate \member{}*/
        /* --1-- */ void ground_action_schema(Planning::Action_Schema& action_Schema);
        /* --3-- */ void ground_action_schema(const Action_Name& action_name,
                                  Planning::Formula::Subformula& effect_formula,
                                  std::map<Variable, Constant>& assignment_detail,
                                  const std::map<Planning::Variable, Planning::Constants/*FIX*/>& potential_assignments,
                                  const Argument_List& action_variables,
                                  Planning::Formula::Subformula precondition,
                                  const std::vector<Variable>& variables_in_order,
                                  uint variable_index);

        void 
        /* --2-- */ ground_action_schema(const Action_Name& action_Name,
                             Planning::Formula::Subformula& effect_formula,
                             std::map<Variable, Constant>& assignment_detail, /*explicit representation of results*/
                             const std::map<Variable, Constants>& potential_assignments, /* constants from which the result is formed.*/
                             const Argument_List& action_Arguments, /*Gives the order in which variables assignment should be made -- Some of these may be constant.*/
                             Planning::Formula::Subformula _precondition
                             );
        /* --4-- */ void press_ground_action(const Action_Name& action_name,
                                 Planning::Formula::Subformula precondition,  
                                 Planning::Formula::Subformula __effect_formula,/*This should be completely ground at this stage -- i.e., no variable symbols.. */
                                 std::map<Variable, Constant>& assignment_detail,
                                 const Argument_List& action_variables
                                 );
        
        /* Populate \member{}*/
        void ground_derived_predicate_schema(Planning::Derived_Predicate&);
        
        /* Populate \member{}*/
        void ground_derived_percept_schema(Planning::Derived_Percept&);
        
        
        /* Simplify the description of the
         * \argument{Derived_Predicate} trigger formula -- i.e., the
         * formula that described when the predicate is satisfied, and
         * when it is not satisfied.*/
        void simplify_derived_predicate_trigger(Planning::Derived_Predicate& );
        
        /*Percept analogue of \member{simplify_derived_predicate_trigger}.*/
        void simplify_derived_percept_trigger(Planning::Derived_Percept& );
        

        /* Ground problem actions. */
        std::vector<CXX__PTR_ANNOTATION(State_Transformation)> state_transformations;
        
        /*Problem targetted by this solver.*/
        Parsing::Problem_Data& problem_Data;
        
        /* Domain data associated with \member{problem} (see
         * \member{preprocess}).*/
        CXX__PTR_ANNOTATION(Parsing::Domain_Data) domain_Data;
    };
}


#endif


/* You stick to your trade, I'll stick to mine
 *
 *  -- Arnold Rothstein 
 */
