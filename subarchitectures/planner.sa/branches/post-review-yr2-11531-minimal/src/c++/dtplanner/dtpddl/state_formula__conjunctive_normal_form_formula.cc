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

#include "state_formula__conjunctive_normal_form_formula.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "planning_state.hh"
// #include "planning_types_enum.hh"

using namespace Planning;
using namespace Planning::State_Formula;
               

void Conjunctive_Normal_Form_Formula::report__newly_satisfied(State& state) const
{
    report__newly_satisfied_clause(state);
}

void Conjunctive_Normal_Form_Formula::report__newly_unsatisfied(State& state) const
{
    report__newly_unsatisfied_clause(state);
}

void Conjunctive_Normal_Form_Formula::set__satisfied(State& state) const
{
    assert(state.get__cnfs__satisfaction_status().valid_index(get__id()));
    state.get__cnfs__satisfaction_status().satisfy(get__id());
}

void Conjunctive_Normal_Form_Formula::set__unsatisfied(State& state) const
{
    assert(state.get__cnfs__satisfaction_status().valid_index(get__id()));
    state.get__cnfs__satisfaction_status().unsatisfy(get__id());
}

void Conjunctive_Normal_Form_Formula::flip(State& state) const
{
    assert(state.get__cnfs__satisfaction_status().valid_index(get__id()));
    WARNING("Something just changed the Boolean status value associated with a"<<std::endl
            <<"CNF by an unusual mechanism -- "<<std::endl
            <<"i.e., the satisfaction for a clause was not reported changed.");
    state.get__cnfs__satisfaction_status().flip_satisfaction(get__id());
}
            
bool Conjunctive_Normal_Form_Formula::is_satisfied(const State& state) const
{
    assert(state.get__cnfs__satisfaction_status().valid_index(get__id()));
    return state.get__cnfs__satisfaction_status().satisfied(get__id());
}

void Conjunctive_Normal_Form_Formula::increment__level_of_satisfaction(State&state) const
{
    QUERY_UNRECOVERABLE_ERROR(!state.get__cnfs__count_status().valid_index(get__id())
                              , "Failing index test on formula :: "<<*this<<std::endl
                              <<"with id :: "<<get__id()<<std::endl
                              <<"only :: "<<state.get__cnfs__count_status().size()<<" clauses registered."<<std::endl);
    
    state.get__cnfs__count_status().increment_satisfaction(get__id());
}

void Conjunctive_Normal_Form_Formula::decrement__level_of_satisfaction(State&state) const
{
    assert(state.get__cnfs__count_status().valid_index(get__id()));
    state.get__cnfs__count_status().decrement_satisfaction(get__id());
}

void Conjunctive_Normal_Form_Formula::set__level_of_satisfaction(uint value, State&state) const
{
    assert(state.get__cnfs__count_status().valid_index(get__id()));
    state.get__cnfs__count_status().set_satisfaction(get__id(), value);
}

uint Conjunctive_Normal_Form_Formula::get__level_of_satisfaction(State&state) const
{
    assert(state.get__cnfs__count_status().valid_index(get__id()));
    return state.get__cnfs__count_status().get_satisfaction_level(get__id());
}





uint Conjunctive_Normal_Form_Formula::get__number_of_satisfied_clauses(State& state) const
{
    return get__level_of_satisfaction(state);
}


void Conjunctive_Normal_Form_Formula::report__newly_satisfied_clause(State& state) const
{
    
    INTERACTIVE_VERBOSER(true, 10004, "Increasing satisfaction of  :: "
                         <<*this<<std::endl);
    
    increment__level_of_satisfaction(state);

    QUERY_UNRECOVERABLE_ERROR(get__level_of_satisfaction(state) > get__disjunctive_clauses().size(),
                              "Satisfaction is :: "<<get__level_of_satisfaction(state)<<std::endl
                              <<"For CNF with clause count :: "<<get__disjunctive_clauses().size());
    
    
    if(get__disjunctive_clauses().size() == get__number_of_satisfied_clauses(state)){
        set__satisfied(state);
        
        satisfy_listeners(state);
//         auto listeners = get__traversable__listeners();
//         for(auto listener = listeners.begin()
//                 ; listener != listeners.end()
//                 ; listener++){
//             INTERACTIVE_VERBOSER(true, 9061, "Just SATISFIED clause  :: "<<*this<<std::endl
//                                  <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
//             (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
//         }
    }   
}

void Conjunctive_Normal_Form_Formula::report__newly_unsatisfied_clause(State& state) const
{
    INTERACTIVE_VERBOSER(true, 9090, "Decreasing satisfaction of  :: "
                         <<*this<<std::endl);
    
    assert(get__level_of_satisfaction(state) > 0);
    
    decrement__level_of_satisfaction(state);

    if(is_satisfied(state)){
        set__unsatisfied(state);
        unsatisfy_listeners(state);
//         auto listeners = get__traversable__listeners();
//         for(auto listener = listeners.begin()
//                 ; listener != listeners.end()
//                 ; listener++){
//             INTERACTIVE_VERBOSER(true, 9061, "Just UNSATISFIED clause  :: "<<*this<<std::endl
//                                  <<"Killing listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
//             (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
//         }
    }
}



const List__Disjunctive_Clauses& Conjunctive_Normal_Form_Formula::get__disjunctive_clauses() const
{
    return std::tr1::get<0>(contents());
}

const Disjunctive_Clause&
Conjunctive_Normal_Form_Formula::get__disjunctive_clause(int i) const
{
    auto tmp = get__disjunctive_clauses();

    return *tmp[i].cxx_get<Disjunctive_Clause>();
}


std::ostream& Conjunctive_Normal_Form_Formula::operator<<(std::ostream&o) const
{
    o<<"CNF::{";
    for(auto clause = get__disjunctive_clauses().begin()
            ; clause != get__disjunctive_clauses().end()
            ; clause++){
        o<<*clause<<";  "<<std::endl;
    }
    
    o<<"}"<<std::endl;



    auto listeners = get__traversable__listeners();
    if(!listeners.size()){
        o<<"CNF has no listeners :: "<<std::endl;
    }
//     for(auto listener = listeners.begin()
//             ; listener != listeners.end()
//             ; listener ++){
//         assert((*listener).get() != this);
//         o<<"LISTENER :: "<<listener->cxx_get<>()->get__identifier()<<std::endl;
// //         INTERACTIVE_VERBOSER(true, 9092, " has listener :: "<<*listener<<std::endl);
        
//     }
    
    return o;
}


namespace std
{
    std::ostream& operator<<(std::ostream&o, const Planning::State_Formula::Conjunctive_Normal_Form_Formula&in)
    {
        return in.operator<<(o);
    }
    
}
