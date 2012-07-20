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


#ifndef STATE_FORMULA__CONJUNCTIVE_NORMAL_FORM_FORMULA_HH
#define STATE_FORMULA__CONJUNCTIVE_NORMAL_FORM_FORMULA_HH


#include "state_formula.hh"

namespace Planning
{
    namespace State_Formula
    {
        class Conjunctive_Normal_Form_Formula
            : public _Satisfaction_Listener<enum_types::conjunctive_normal_form_formula
                                            , List__Disjunctive_Clauses >
        {PRINTING;
        public:
            
            void report__newly_satisfied(State&) const;
            void report__newly_unsatisfied(State&) const;

            
            /* Changes the satisfaction status in \argument{State} of
             * the Boolean valued object.*/
            void flip(State&) const;
            bool is_satisfied(const State&) const;
            
            uint get__level_of_satisfaction(State&) const;
            
            /*How many of this formula's clauses are satisfied? (see
             * \parent{Satisfied_By_Count}::\member{get__level_of_satisfaction()}).*/
            uint get__number_of_satisfied_clauses(State&) const;

            
            /*Change the level of satisfaction of this formula.*/
            void report__newly_satisfied_clause(State&) const;
            void report__newly_unsatisfied_clause(State&) const;
            
            /*Get the \argument{i}th clause in the formula.*/
            const Disjunctive_Clause& get__disjunctive_clause(int i) const;
            
            /*Get the clauses in the formula.*/
            const List__Disjunctive_Clauses& get__disjunctive_clauses() const;
        private:
            void set__level_of_satisfaction(uint, State&) const;
            void increment__level_of_satisfaction(State&) const;
            void decrement__level_of_satisfaction(State&) const;
            void set__satisfied(State&) const;
            void set__unsatisfied(State&) const;
        };
    }
}

#endif
