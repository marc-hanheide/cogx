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

#ifndef BASIC_ACTION_HH
#define BASIC_ACTION_HH

#include "state_basics.hh"
#include "planning_formula.hh"
#include "state_formula.hh"

namespace Planning
{
    /* We do not keep "transformation" preconditions (i.e., a PDDL
     * action preconditions) in the transformation, but rather store
     * those separately as CNF formulae. When a formula is true at
     * some given state, then \class{Satisfaction_Listener} that are
     * registered with that formula get notified (see
     * \module{state_formula.hh}). */
    class State_Transformation :
        public State_Formula::
        _Satisfaction_Listener<enum_types::state_transformation
                               , Formula::Action_Proposition
                               , State_Formula::Conjunctive_Normal_Form_Formula__Pointer
                               , State_Formula::List__Literals /* effects */
                               , bool /* compulsory -- whole action */
                               , double /* Probability that this transformation is applied. */ >
    {
    public:

        /* Should be repeatedly executed until the result is NULL, or
         * no pending actions are required.
         *
         * \argument{predecessor} is the state that transitions to
         * \argument{successor}. The latter is the state being
         * generated. \argument{SetOfStatePointers} is the set of
         * problem states thus far discovered.*/
        State& operator()(State& predecessor);



        
        void report__newly_satisfied(State&);
        void report__newly_unsatisfied(State&);

        void set__satisfied(State&);
        void set__unsatisfied(State&);
        void flip_satisfaction(State&);
        bool is_satisfied(const State&) const;
            
        void increment__level_of_satisfaction(State&);
        void decrement__level_of_satisfaction(State&);
        void set__level_of_satisfaction(uint, State&);
        uint get__level_of_satisfaction(State&) const;


        uint get__number_of_satisfied_conditions(State& state) const;





        
        const Formula::Action_Proposition& get__get_identifier() const;
        const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& get__precondition() const;
        const State_Formula::List__Literals& get__effects() const;
        bool get__compulsory() const;
        double get__probability() const;


        static Are_Doubles_Close are_Doubles_Close;//(1e-9);
        
    };
}



#endif

/* Did he fall?
 * 
 * By his body's known weight of eleven-stone-and-four-pounds in
 * avoirdupois measure, as certified by the graduated machine for
 * periodical selfweighing in the premises of Francis Froedman,
 * pharmaceutical chemist of 19 Fredrick street, north, on the last
 * feast of the Ascension, to wit, the twelfth day of May of the
 * bissextile year one-thousand-nine-hundred-and-four of the christian
 * era, (jewish era five-thousand-six-hundred-and-sixty-four,
 * mohammadan era one-thousand-three-hundred-and-twentytwo), golden
 * number 5, epact 13, solar cycle 9, dominical letters C B, Roman
 * indication 2, Julian period 6617, MXMIV.
 *
 *  -- James Joyce, Ulysses (Ed. 1922)
 */
