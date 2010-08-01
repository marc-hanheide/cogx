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

#include "planning_types_enum.hh"
#include "planning_formula.hh"
#include "stl__typed_thing.hh"


namespace Planning
{
    class State;
    
    namespace State_Formula
    {
        class Literal;
        class Disjunctive_Clause;
        class Conjunctive_Normal_Form_Formula;

        typedef Formula::Subformula Satisfaction_Listener__Pointer;
        typedef std::vector<Satisfaction_Listener__Pointer> List__Listeners;
        typedef std::set<Satisfaction_Listener__Pointer> Listeners;

        class _Satisfaction_Listener
        {
        public:
            virtual ~_Satisfaction_Listener(){};
            virtual void report__newly_satisfied(State&) = 0;
            virtual void report__newly_unsatisfied(State&) = 0;
        };
        
        template<int type_name, typename... T>
        class Satisfaction_Listener : public type_wrapper<type_name, List__Listeners, Listeners, T...>,
                                      public _Satisfaction_Listener
        {
        public:
            typedef type_wrapper<type_name, List__Listeners, Listeners, T...> Parent;
            

            const List__Listeners& get__traversable_parents() const {return std::tr1::get<0>(Parent::contents());};
            const Listeners& get__searchable_parents() const {return std::tr1::get<1>(Parent::contents());};
            
//             List__Listeners& get__traversable_parents() {return std::tr1::get<0>(Parent::contents());};
//             Listeners& get__searchable_parents() {return std::tr1::get<1>(Parent::contents());};
            
        };


        
        class Literal
            : public Satisfaction_Listener<enum_types::literal /* Type identifier.*/
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

        typedef CXX__deref__shared_ptr<State_Formula::Literal> Literal__Pointer; 
        typedef std::set<Literal__Pointer > Literals;       
        typedef std::vector<Literal__Pointer > List__Literals;       
        
        class Disjunctive_Clause
            : public Satisfaction_Listener<enum_types::disjunctive_clause
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
            List__Literals& get__literals();
        };

        
        typedef CXX__deref__shared_ptr<Disjunctive_Clause> Disjunctive_Clause__Pointer; 
        typedef std::set<Disjunctive_Clause__Pointer > Disjunctive_Clauses;       
        typedef std::vector<Disjunctive_Clause__Pointer > List__Disjunctive_Clause;     
        
        class Conjunctive_Normal_Form_Formula
            : public Satisfaction_Listener<enum_types::conjunctive_normal_form_formula
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
        
        typedef CXX__deref__shared_ptr<Conjunctive_Normal_Form_Formula> Conjunctive_Normal_Form_Formula__Pointer; 
        typedef std::set<Conjunctive_Normal_Form_Formula__Pointer > Conjunctive_Normal_Form_Formulae;       
        typedef std::vector< Conjunctive_Normal_Form_Formula__Pointer> List__Conjunctive_Normal_Form_Formula;     
        
    }
}


#endif
