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


#ifndef STATE_FORMULA_HH
#define STATE_FORMULA_HH

#include "state_basics.hh"
#include "planning_types_enum.hh"
#include "planning_formula.hh"
#include "stl__typed_thing.hh"


namespace Planning
{
    namespace State_Formula
    {
        class Satisfaction_Listener
        {
        public:
            virtual ~Satisfaction_Listener();/*EMPTY*/
            virtual void report__newly_satisfied(State&) = 0;
            virtual void report__newly_unsatisfied(State&) = 0;
            
            void add__listener(Satisfaction_Listener__Pointer&);
            
            const List__Listeners& get__traversable__listeners() const ;
            const Listeners& get__searchable__listeners() const ;
        private:
            List__Listeners list__Listeners;
            Listeners listeners;
        };
        
        template<int type_name, typename... T>
        class _Satisfaction_Listener : public type_wrapper<type_name, T...>,
                                       public Satisfaction_Listener
        {
        public:
            typedef type_wrapper<type_name, List__Listeners, Listeners, T...> Parent;
        };


        
        class Literal
            : public _Satisfaction_Listener<enum_types::literal /* Type identifier.*/
                                            , uint /* Boolean variable identifier.*/
                                            , bool >
        {
        public:

            void report__newly_satisfied(State&);
            void report__newly_unsatisfied(State&);
            
            void set__satisfied(State&);
            void set__unsatisfied(State&);
            void flip_satisfaction(State&);
            bool is_satisfied(const State&) const;

            
            /* Set the subject variable to have the truth value "TRUE".*/
            void flip_variable_on(State&);
            
            /* Set the subject variable to have the truth value "FALSE".*/
            void flip_variable_off(State&);
            
            /* Set the subject variable to have the opposite truth
             * value to its current assignment.*/
            void flip(State&);

            /* Variable, that is the subject of this literal.*/
            uint get__variable() const;

            /* Sign of this literal (true is positive, false is negative). */
            bool get__sign() const;                   
        };
 
        
        class Disjunctive_Clause
            : public _Satisfaction_Listener<enum_types::disjunctive_clause
                                            , List__Literals>
        {
        public:            
            
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
            
            /*(see \parent{Satisfied_By_Count}::\member{get__level_of_satisfaction()})*/
            uint get__number_of_satisfied_literals(State&) const;    

            /*Change the level of satisfaction of this clause.*/
            void report__newly_satisfied_literal(State&);
            void report__newly_unsatisfied_literal(State&);
            
            
            /*Get the \argument{i}th literal in the clause.*/
            const Literal& get__literal(int i) const;
            
            /*Get the literals in the clause.*/
            const List__Literals& get__literals() const;
            List__Literals& get__literals();/*_contents .. CHECK*/
        };

        
        class Conjunctive_Normal_Form_Formula
            : public _Satisfaction_Listener<enum_types::conjunctive_normal_form_formula
                                            , List__Disjunctive_Clause >
        {
        public:
            
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
            
            /*How many of this formula's clauses are satisfied? (see
             * \parent{Satisfied_By_Count}::\member{get__level_of_satisfaction()}).*/
            uint get__number_of_satisfied_clauses(State&) const;

            
            /*Change the level of satisfaction of this formula.*/
            void report__newly_satisfied_clause(State&);
            void report__newly_unsatisfied_clause(State&);
            
            /*Get the \argument{i}th clause in the formula.*/
            const Disjunctive_Clause& get__disjunctive_clause(int i) const;
            
            /*Get the clauses in the formula.*/
            const List__Disjunctive_Clause& get__disjunctive_clauses() const;
        };
    }
}


#endif
