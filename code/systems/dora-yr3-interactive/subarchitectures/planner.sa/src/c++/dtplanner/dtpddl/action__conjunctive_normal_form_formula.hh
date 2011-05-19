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

#ifndef ACTION__CONJUNCTIVE_NORMAL_FORM_FORMULA_HH
#define ACTION__CONJUNCTIVE_NORMAL_FORM_FORMULA_HH


#include "action_basics.hh"
#include "state_formula.hh"
#include "planning_types_enum.hh"

namespace Planning
{
    
    class Action_Conjunctive_Normal_Form_Formula
        : public State_Formula::
        _Satisfaction_Listener<enum_types::action_conjunctive_normal_form_formula
                               , List__Action_Disjunctive_Clauses >
    {
        PRINTING;
    public:
            
        void report__newly_satisfied(State&) const;
        void report__newly_unsatisfied(State&) const;

        bool is_satisfied(const State&) const;
            
        uint get__level_of_satisfaction(State&) const;
            
        uint get__number_of_satisfied_clauses(State&) const;

            
        void report__newly_satisfied_clause(State&) const;
        void report__newly_unsatisfied_clause(State&) const;
            
        const List__Action_Disjunctive_Clauses& get__disjunctive_clauses() const;
    private:
        void increment__level_of_satisfaction(State&) const;
        void decrement__level_of_satisfaction(State&) const;
        void set__satisfied(State&) const;
        void set__unsatisfied(State&) const;
    };
    
}



#endif
