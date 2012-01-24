
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


#ifndef OBSERVATION_HH
#define OBSERVATION_HH

#include "observation_basics.hh"
#include "planning_observation.hh"
#include "state_formula.hh"
#include "planning_types_enum.hh"


namespace Planning
{
    class Observation
        : public State_Formula::
        _Satisfaction_Listener<enum_types::observation
                               , Formula::Observational_Proposition
                               , State_Formula::Conjunctive_Normal_Form_Formula__Pointer /*State precondition*/
                               , Action_Conjunctive_Normal_Form_Formula__Pointer /*Action precondition*/
                               , Formula::List__Perceptual_Propositions/*effects*/
                               , bool /* top-level -- derived from the top level effects of a PDDL observation schema */
                               , bool /* lookup probability */
                               , double /* Probability that this transformation is applied. */
                               , uint /* Probability lookup index (ignore if "lookup probability" is false). */>
    {
        PRINTING;
    public:
        /* top-level -- derived from the top level effects of a PDDL observation schema. */
        bool is_top_level() const;
        
        /* How is this perception identified in the other modules of this
         * system? This will be a PDDL-based identifier if the
         * transformation corresponds to a ground PDDL operator.*/
        const Formula::Observational_Proposition& get__identifier() const;

        /* What conditions on a state must be satisfied in order for
         * this perception to be applicable? That condition is
         * expressed as a conjunctive normal form propositional
         * formula (see \module{state_formula}). */
        const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& get__precondition() const;
        
        /* What conditions on an execution must be satisfied in order
         * for this perception to be applicable? That condition is
         * expressed as a conjunctive normal form propositional
         * formula (see
         * \module{action__conjunctive_normal_form_formula}). */
        const Action_Conjunctive_Normal_Form_Formula__Pointer& get__execution_precondition() const;
        
        /* What are the add and delete effects of this perception? */
        const Formula::List__Perceptual_Propositions& get__effects() const;

        /* FIX :: Should the probability of making a perception be read from a state?*/
        bool get__lookup_probability() const;

        void forced_wake(State&) const;
        
        /*What is the probability of successful application of this perception?*/
        double get__probability() const;

        /* If the probability of a successful perception is to be
         * read from a state, then this member reads that
         * information. */
        double get__probability(const State&) const;
        
        /**/
        Planning::Observational_State* operator()(Planning::Observational_State*, Planning::State*) const;
        
        void report__newly_satisfied(State&) const;
        void report__newly_unsatisfied(State&) const;
            
        uint get__level_of_satisfaction(State&) const;

        uint get__number_of_satisfied_conditions(State& state) const;
        
        bool is_satisfied(const State&) const;
    private:
        void increment__level_of_satisfaction(State&) const;
        void decrement__level_of_satisfaction(State&) const;
        void set__satisfied(State&) const;
        void set__unsatisfied(State&) const;
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
    };
}


#endif

