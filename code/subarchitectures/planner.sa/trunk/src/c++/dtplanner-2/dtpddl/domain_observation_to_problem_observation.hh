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


#ifndef DOMAIN_OBSERVATION_TO_PROBLEM_OBSERVATION_HH
#define DOMAIN_OBSERVATION_TO_PROBLEM_OBSERVATION_HH


#include "dtp_pddl_parsing_data.hh"
#include "planning_formula.hh"
#include "planning_formula.hh"
#include "assignment_applicator.hh"
#include "planning_formula_to_cnf.hh"


#include "action_basics.hh"
#include "state_basics.hh"
#include "observation_basics.hh"


namespace Planning
{  
    class Domain_Observation__to__Problem_Observation : public Visitor<basic_type>
    {
    public:
        Domain_Observation__to__Problem_Observation
        (basic_type::Runtime_Thread runtime_Thread,
         Planning::Assignment& assignment,/*Can have entries added to it if we have quantification in action effects.*/
         Formula::State_Propositions& state_Propositions,
         Formula::Perceptual_Propositions& problem__perceptual_Propositions,
         Formula::State_Ground_Functions& state_Ground_Functions,
         State_Formula::Literals& problem__literals,
         State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses,
         State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae,
//          Action_Literals& action_Literals,
//          Action_Disjunctive_Clauses& action_Disjunctive_Clauses,
//          Action_Conjunctive_Normal_Form_Formulae& action_Conjunctive_Normal_Form_Formulae,
         const Parsing::Domain_Data& _domain_Data,
         const Parsing::Problem_Data& _problem_Data,
         const Formula::Observational_Proposition& observational_Proposition,
         State_Formula::Conjunctive_Normal_Form_Formula__Pointer& precondition,
         Action_Conjunctive_Normal_Form_Formula__Pointer& execution_precondition,
         Observations& problem__observations,
         Observations& problem__observations_without_preconditions,
         std::pair<basic_type::Runtime_Thread, ID_TYPE>& actions_validator);
        
        DECLARATION__UNARY_VISITOR(basic_type);


        /* compulsory ground PDDL observation schema. */
        Observation__Pointer get__answer() const ;
    private:
        bool deal_with_a_missing_conjunctive_parent(const Formula::Subformula&);
        
        void interpret__as_double_valued_ground_state_function(ID_TYPE);
        
        const Planning::Observation__Pointer& generate__null_observation(double local_probability) ;
        
        
        Formula::Observational_Proposition process__generate_name(std::string annotation = "") ;

        static Formula::Subformula simplify_formula(Formula::Subformula,
                                                    basic_type::Runtime_Thread);

        
        
        basic_type::Runtime_Thread runtime_Thread;
        Planning::Assignment& assignment;
        
        Formula::State_Ground_Functions& problem__state_Functions;
        Formula::State_Propositions& problem__state_Propositions;
        Formula::Perceptual_Propositions& problem__perceptual_Propositions;
        
        State_Formula::Literals& problem__literals;
        State_Formula::Disjunctive_Clauses& problem__disjunctive_Clauses;
        State_Formula::Conjunctive_Normal_Form_Formulae& problem__conjunctive_Normal_Form_Formulae;
        
//         Action_Literals& problem__action_literals;
//         Action_Disjunctive_Clauses& problem__action_clauses;
//         Action_Conjunctive_Normal_Form_Formulae& problem__action_cnfs;
        
        
        const Parsing::Domain_Data& domain_Data;
        const Parsing::Problem_Data& problem_Data;
        
        const Formula::Observational_Proposition& observational_Proposition;
        
        /* A tool to apply an assignment to variables to a CNF formula. */
        CNF_Assignment_Applicator assignment_Applicator;


        Observations& problem__observations;
        Observations& problem__observations_without_preconditions;
        
        
        /* Object converts planning formula ---in this case formulae
         * that correspond to actions preconditions--- into
         * Conjunctive Normal Form formula.  Such a formula is a
         * conjunction over disjunctive clauses.  */
        static Planning_Formula__to__CNF planning_Formula__to__CNF;
        
        Planning::Observation__Pointer result;

        /* CNF that is, as considered by future processing, always satisfied. */
        State_Formula::Conjunctive_Normal_Form_Formula__Pointer true_cnf;
        Action_Conjunctive_Normal_Form_Formula__Pointer true_action_cnf;
        
        
        std::stack<Formula::List__Perceptual_Propositions> effects_lists;
        std::stack<State_Formula::List__Listeners> list__Listeners;
        std::stack<State_Formula::Listeners> listeners;
        
        std::stack<State_Formula::Conjunctive_Normal_Form_Formula__Pointer> preconditions;
        std::stack<Action_Conjunctive_Normal_Form_Formula__Pointer> execution_preconditions;
        
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
        
        bool processing_negative;/* false -- initialise*/
        uint count_of_observations_posted;/* 0 initialise*/
        uint level;/* 0 initialise*/
        
        double probability;/* 1.0  initialise*/

        int last_function_symbol_id;
        
        /* Last double valued entry traversed while grounding action.*/
        double last_double_traversed;
    };
}



#endif
