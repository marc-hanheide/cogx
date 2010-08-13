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

#ifndef STATE_FORMULA__DISJUNCTIVE_CLAUSE_HH
#define STATE_FORMULA__DISJUNCTIVE_CLAUSE_HH


#include "state_formula.hh"


namespace Planning
{
    namespace State_Formula
    {
        class Disjunctive_Clause
            : public _Satisfaction_Listener<enum_types::disjunctive_clause
                                            , List__Literals>
        {PRINTING;
        public:            
            
            void report__newly_satisfied(State&) const;
            void report__newly_unsatisfied(State&) const;
            

            void set__satisfied(State&) const;
            void set__unsatisfied(State&) const;
            void flip_satisfaction(State&) const;
            bool is_satisfied(const State&) const;
            
            void increment__level_of_satisfaction(State&) const;
            void decrement__level_of_satisfaction(State&) const;
            void set__level_of_satisfaction(uint, State&) const;
            uint get__level_of_satisfaction(State&) const;
            
            /*(see \parent{Satisfied_By_Count}::\member{get__level_of_satisfaction()})*/
            uint get__number_of_satisfied_literals(State&) const;    

            /*Change the level of satisfaction of this clause.*/
            void report__newly_satisfied_literal(State&) const;
            void report__newly_unsatisfied_literal(State&) const;
            
            
            /*Get the \argument{i}th literal in the clause.*/
            const Literal& get__literal(int i) const;
            
            /*Get the literals in the clause.*/
            const List__Literals& get__literals() const;
            List__Literals& get__literals();/*_contents .. CHECK*/
        };
    }
}


#endif
