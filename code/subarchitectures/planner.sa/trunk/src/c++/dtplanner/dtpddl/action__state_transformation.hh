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

#ifndef ACTION__STATE_TRANSFORMATION_HH
#define ACTION__STATE_TRANSFORMATION_HH

#include "planning_formula.hh"
#include "planning_types_enum.hh"
#include "state_basics.hh"
#include "planning_formula.hh"
#include "state_formula.hh"

namespace Planning
{
    
    /* We do not explicitly store "transformation" preconditions
     * (i.e., a PDDL action preconditions) in the transformation, but
     * rather store those separately as CNF formulae. When a formula
     * is true at some given state, then \class{Satisfaction_Listener}
     * that are registered with that formula get notified (see
     * \module{state_formula.hh}). */
    class State_Transformation :
        public State_Formula::
        _Satisfaction_Listener<enum_types::state_transformation
                               , Formula::Action_Proposition
                               , State_Formula::Conjunctive_Normal_Form_Formula__Pointer/*TODO deal with empty conjunct.*/
                               , State_Formula::List__Literals /* effects */
                               , bool /* compulsory -- whole action */
                               , bool /* lookup probability */
                               , double /* Probability that this transformation is applied. */
                               , uint /* Probability lookup index (ignore if "lookup probability" is false). */>
    {PRINTING;
        
    public:
        /* How is this action identified in the other modules of this
         * system? This will be a PDDL-based identifier if the
         * transformation corresponds to a ground PDDL operator.*/
        const Formula::Action_Proposition& get__identifier() const;

        /* What conditions on a state must be satisfied in order for
         * this transformation to be applicable? That condition is
         * expressed as a conjunctive normal form propositional
         * formula (see \module{state_formula}). */
        const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& get__precondition() const;
        /* What are the add and delete effects of this transformation? */
        const State_Formula::List__Literals& get__effects() const;

        /* A compulsory transformation, is one that must be applied to
         * a state. That is, the agent cannot choose to execute it, or
         * choose not to execute it. */
        bool get__compulsory() const;

        /* Should the probability of application be read from a state?*/
        bool get__lookup_probability() const;

        /*What is the probability of successful application of this transformation?*/
        double get__probability() const;

        /* If the probability of a successful transformation is to be
         * read from a state, then this member reads that
         * information. */
        double get__probability(const State&) const;
        
//         virtual ~State_Transformation(){};

//         typedef std::vector<State*> Result_Type;
        
        /* Should be repeatedly executed until the result is NULL, or
         * no pending actions are required. If the \return{bool} is
         * true, then this method should be called again on the same
         * input state, etc until the \return{bool} is false.
         *
         * Sometimes the result has a different address from the
         * input. Here, the transformation has been generative.
         *
         * \argument{predecessor} is the state that transitions to
         * \argument{successor}. The latter is the state being
         * generated. \argument{SetOfStatePointers} is the set of
         * problem states thus far discovered.*/
         State* operator()(State* predecessor) const;
        
        void report__newly_satisfied(State&) const;
        void report__newly_unsatisfied(State&) const;

        /* Changes the executability status in \argument{State} of the
         * transformation object.*/
        void flip(State&) const;
        
        bool is_satisfied(const State&) const;
            
        uint get__level_of_satisfaction(State&) const;


        uint get__number_of_satisfied_conditions(State& state) const;

    private:
        void set__level_of_satisfaction(uint, State&) const;
        void increment__level_of_satisfaction(State&) const;
        void decrement__level_of_satisfaction(State&) const;
        void set__satisfied(State&) const;
        void set__unsatisfied(State&) const;
        
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
    };
    
}

#endif
