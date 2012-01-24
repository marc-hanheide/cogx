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

#include "action__conjunctive_normal_form_formula.hh"

#include "planning_state.hh"
#include "action__literal.hh"
#include "action__disjunctive_clause.hh"

using namespace Planning;

void Action_Conjunctive_Normal_Form_Formula::report__newly_satisfied(State& state) const
{
    report__newly_satisfied_clause(state);
}

void Action_Conjunctive_Normal_Form_Formula::report__newly_unsatisfied(State& state) const
{
    report__newly_unsatisfied_clause(state);
}

void Action_Conjunctive_Normal_Form_Formula::set__satisfied(State& state) const
{
    state.get__action_cnfs__satisfaction_status().satisfy(get__id());
}

void Action_Conjunctive_Normal_Form_Formula::set__unsatisfied(State& state) const
{
    state.get__action_cnfs__satisfaction_status().unsatisfy(get__id());
}
  
bool Action_Conjunctive_Normal_Form_Formula::is_satisfied(const State& state) const
{
    return state.get__action_cnfs__satisfaction_status().satisfied(get__id());
}

void Action_Conjunctive_Normal_Form_Formula::increment__level_of_satisfaction(State&state) const
{
    state.get__action_cnfs__count_status().increment_satisfaction(get__id());
}

void Action_Conjunctive_Normal_Form_Formula::decrement__level_of_satisfaction(State&state) const
{
    state.get__action_cnfs__count_status().decrement_satisfaction(get__id());
}

uint Action_Conjunctive_Normal_Form_Formula::get__level_of_satisfaction(State&state) const
{
    return state.get__action_cnfs__count_status().get_satisfaction_level(get__id());
}

uint Action_Conjunctive_Normal_Form_Formula::get__number_of_satisfied_clauses(State& state) const
{
    return get__level_of_satisfaction(state);
}


void Action_Conjunctive_Normal_Form_Formula::report__newly_satisfied_clause(State& state) const
{
    increment__level_of_satisfaction(state);

    if(get__disjunctive_clauses().size() == get__number_of_satisfied_clauses(state)){
        set__satisfied(state);
        satisfy_listeners(state);
    }   
}

void Action_Conjunctive_Normal_Form_Formula::report__newly_unsatisfied_clause(State& state) const
{
    decrement__level_of_satisfaction(state);

    if(is_satisfied(state)){
        set__unsatisfied(state);
        unsatisfy_listeners(state);
    }
}

const List__Action_Disjunctive_Clauses&
Action_Conjunctive_Normal_Form_Formula::
get__disjunctive_clauses() const
{
    return std::tr1::get<0>(contents());
}

std::ostream& Action_Conjunctive_Normal_Form_Formula::operator<<(std::ostream&o) const
{
    o<<"{";
    for(auto clause = get__disjunctive_clauses().begin()
            ; clause != get__disjunctive_clauses().end()
            ; clause++){
        o<<*clause<<";  "<<std::endl;
    }
    
    o<<"}"<<std::endl;
    return o;
}




namespace std
{
    std::ostream& operator<<(std::ostream&o
                             , const Planning::Action_Conjunctive_Normal_Form_Formula&in)
    {
        return in.operator<<(o);
    }
    

}
