
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


#ifndef DOMAIN_ACTION_TO_PROBLEM_ACTION_HH
#define DOMAIN_ACTION_TO_PROBLEM_ACTION_HH

#include "dtp_pddl_parsing_data.hh"
#include "planning_formula.hh"
#include "planning_formula.hh"
#include "assignment_applicator.hh"
#include "planning_formula_to_cnf.hh"
#include "action_basics.hh"
#include "state_basics.hh"

namespace Planning
{  
    class Domain_Action__to__Problem_Action : public Visitor<basic_type>
    {
    public:
        //typedef std::map<Planning::Variable, Planning::Constant> Assignment;
        
        Domain_Action__to__Problem_Action(basic_type::Runtime_Thread,
                                          Planning::Assignment& assignment,/*Can have entries added to it if we have quantification in action effects.*/
                                          Formula::State_Propositions&,
                                          Formula::State_Ground_Functions& ,
                                          State_Formula::Literals& problem__literals,
                                          State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses,
                                          State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae,
                                          const Parsing::Domain_Data&,
                                          const Parsing::Problem_Data&,
                                          const Formula::Action_Proposition&,
                                          State_Formula::Conjunctive_Normal_Form_Formula__Pointer&,
                                          State_Transformations&/* Non-Executable, or 1-or-more preconditions. */,
                                          State_Transformations&/* Executable, and zero preconditions. */,
                                          Probabilistic_State_Transformations&,
                                          std::pair<basic_type::Runtime_Thread, ID_TYPE>& actions_validator,
                                          std::map<Formula::Action_Proposition
                                          , State_Transformation__Pointer>& action_symbol__to__state_transformation);
        
        DECLARATION__UNARY_VISITOR(basic_type);


        /* Non-compulsory ground PDDL action. */
        State_Transformation__Pointer get__answer() const ;
    private:
        
        /* Returns true if there was no conjunctive parent to traversal
         * of \argument{input}, false otherwise.*/
        bool deal_with_a_missing_conjunctive_parent(const Formula::Subformula& input);
        
        void interpret__as_double_valued_ground_state_function(ID_TYPE);
        void interpret__as_double_valued_ground_state_function(Formula::State_Ground_Function&);
        
        const Planning::State_Transformation__Pointer& generate__null_action(double local_probability) ;
        
        void process__Function_Modifier(Formula::Subformula& modification,
                                        ID_TYPE);
        
        Formula::Action_Proposition process__generate_name(std::string annotation = "") ;

        
        static Formula::Subformula simplify_formula(Formula::Subformula,
                                                    basic_type::Runtime_Thread);

        
        basic_type::Runtime_Thread runtime_Thread;
        
        
        Planning::Assignment& assignment;
        
        
        
        Formula::State_Propositions& problem__state_Propositions;
        Formula::State_Ground_Functions& problem__state_Functions;
        
        State_Formula::Literals& problem__literals;
        State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses;
        State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae;

        const Parsing::Domain_Data& domain_Data;
        const Parsing::Problem_Data& problem_Data;
        
        
        const Formula::Action_Proposition& action_Proposition;
        //         State_Formula::Conjunctive_Normal_Form_Formula& precondition;/* (see \member{preconditions})*/

        /* A tool to apply an assignment to variables to a CNF formula. */
        CNF_Assignment_Applicator assignment_Applicator;
        
        
        Planning::State_Transformations& problem__actions;
        Planning::State_Transformations& executable_actions_without_preconditions;
        Planning::Probabilistic_State_Transformations& probabilistic_actions;
        
        std::map<Formula::Action_Proposition
                       , State_Transformation__Pointer>& action_symbol__to__state_transformation;
        
        
        bool processing_negative;/* false -- initialise*/
        uint count_of_actions_posted;/* 0 initialise*/
        uint level;/* 0 initialise*/
        double probability;/* 1.0  initialise*/
        
        /* Object converts planning formula ---in this case formulae
         * that correspond to actions preconditions--- into
         * Conjunctive Normal Form formula.  Such a formula is a
         * conjunction over disjunctive clauses.  */
        static Planning_Formula__to__CNF planning_Formula__to__CNF;
        
        
        State_Transformation__Pointer result;

        /* CNF that is, as considered by future processing, always satisfied. */
        State_Formula::Conjunctive_Normal_Form_Formula__Pointer true_cnf;
        

        
        std::stack<State_Formula::List__Literals> literals_at_levels;
        std::stack<State_Formula::List__Listeners> list__Listeners;
        std::stack<State_Formula::Listeners> listeners;
        std::stack<State_Formula::Conjunctive_Normal_Form_Formula__Pointer> preconditions;
        
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
        
        
        
        int last_function_symbol_id;
        
        /* Last double valued entry traversed while grounding action.*/
        double last_double_traversed;

        
    };
}

#endif


/* CogX email interaction from Wednesday, September 29, 2010 at 10:35
 * AMI.
 *
 * Hendrik Zender writes a detailed question: "...performed a full svn
 * up just a minute ago. Any hints what I should do to make it all
 * compile?"
 *
 *
 * Nick Hawes writes concise and expertly response: "Yup, update CAST
 * to the latest tag."
 * 
 */
