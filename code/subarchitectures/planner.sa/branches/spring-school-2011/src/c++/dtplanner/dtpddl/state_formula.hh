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
#include "stl__typed_thing.hh"
#include "planning_formula.hh"


namespace Planning
{
    namespace State_Formula
    {
        class Satisfaction_Listener
        {
        public:
            Satisfaction_Listener(bool = false);
            virtual ~Satisfaction_Listener();/*EMPTY*/


            void satisfy_listeners(State& state) const;
            void unsatisfy_listeners(State& state) const;
            void wake_sleepers(State& state) const;
            
            void wake_sleepers_that_require_forcing(State& state) const;
            bool does_require_explicit_forced_waking() const;
 
            /* Call if this element has been found to be statically
             * unsatisfiable in the \argument{state}. If so, then all
             * the children are also deemed statically unsatisfiable*/
            virtual void set__statically_false(State& state) const;
            
            virtual void forced_wake(State&) const {assert(0);};
            
            void wake(State&in) const {if(!requires_explicit_forced_waking){forced_wake(in);}};
            
            /* Returns false if the \argument{Listener} is already
             * registered.*/
            bool add__sleeper(Satisfaction_Listener__Pointer&);
            
            const List__Listeners& get__traversable__sleepers() const ;
            const Listeners& get__searchable__sleepers() const ;
            
            
            
            
            /* Returns false if the \argument{Listener} is already
             * registered.*/
            bool add__listener(Satisfaction_Listener__Pointer&);
            
            virtual void report__newly_satisfied(State&) const = 0;
            virtual void report__newly_unsatisfied(State&) const = 0;

            const List__Listeners& get__traversable__listeners() const ;
            const Listeners& get__searchable__listeners() const ;
        protected:
            /* Does a non-Satisfaction_Listener object have to wake this listener?*/
            bool requires_explicit_forced_waking;
            
        private:
            List__Listeners list__Listeners;
            Listeners listeners;
            
            /* A sleeper is an object whose satisfaction is not
             * altered by this, however whose relevance only applies
             * when this is satisfied. An action for example, where
             * this is an action precondition.*/
            List__Listeners list__Sleepers;
            Listeners sleepers;
         
        };
        
        template<int type_name, typename... T>
        class _Satisfaction_Listener : public type_wrapper<type_name, T...>,
                                       public Satisfaction_Listener
        {
        public:
            _Satisfaction_Listener(bool in = false)
                :Satisfaction_Listener(in) {}
            
            typedef type_wrapper<type_name, List__Listeners, Listeners, T...> Parent;
        };
        
    }
}


#endif

/* When a distinguished but elderly scientist states that something is
 * possible, he is almost certainly right. When he states that
 * something is impossible, he is probably wrong.
 *
 * Arthur C. Clarke's first law of prediction
 */
