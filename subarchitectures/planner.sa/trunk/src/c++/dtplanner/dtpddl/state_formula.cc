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


#include "state_formula.hh"

#include "planning_state.hh"

using namespace Planning;
using namespace Planning::State_Formula;
      

void Satisfaction_Listener::add__parent(CXX__PTR_ANNOTATION(basic_type) in)
{
    Satisfaction_Listener__Pointer more(in);
    add__parent(more);
}

Satisfaction_Listener::~Satisfaction_Listener()
{
}

void Satisfaction_Listener::add__parent(Satisfaction_Listener__Pointer& satisfaction_Listener__Pointer)
{
    if(listeners.find(satisfaction_Listener__Pointer) != listeners.end()){
        return;
    }
    
    listeners.insert(satisfaction_Listener__Pointer);
    list__Listeners.push_back(satisfaction_Listener__Pointer);
}
            
const List__Listeners& Satisfaction_Listener::get__traversable_parents() const 
{
    return list__Listeners;
}

const Listeners& Satisfaction_Listener::get__searchable_parents() const 
{
    return listeners;
}



void Literal::set__satisfied(State& state)
{
    state.get__literals__satisfaction_status().satisfy(get__id());
}

void Literal::set__unsatisfied(State& state)
{
    state.get__literals__satisfaction_status().unsatisfy(get__id());
}

void Literal::flip_satisfaction(State& state)
{
    state.get__literals__satisfaction_status().flip_satisfaction(get__id());

    if(get__sign() && !state.is_true(get__variable())){
        state.flip(get__variable());
    } else if (!get__sign() && state.is_true(get__variable())) {
        state.flip(get__variable());
    }
}
            
bool Literal::is_satisfied(const State& state) const
{
    return state.get__literals__satisfaction_status().satisfied(get__id());
}

void Literal::report__newly_satisfied(State& state)
{
    set__satisfied(state);
}

void Literal::report__newly_unsatisfied(State& state)
{
    set__unsatisfied(state);
}


uint Literal::get__variable() const
{
    return std::tr1::get<0>(contents());
}

bool Literal::get__sign() const
{
    return std::tr1::get<1>(contents());
}

void Literal::flip_variable_on(State& state)
{
    if(get__sign()){
        if(!is_satisfied(state)){
            flip(state);
        } 
    } else {
        if(is_satisfied(state)){
            flip(state);
        }
    }
}

void Literal::flip_variable_off(State& state)
{
    if(get__sign()){
        if(is_satisfied(state)){
            flip(state);
        } 
    } else {
        if(!is_satisfied(state)){
            flip(state);
        }
    }
}

void Literal::flip(State& state)
{

    bool was_satisfied = is_satisfied(state);
    
    flip_satisfaction(state);

    auto parents = get__traversable_parents();
    if(was_satisfied){
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
    } else {
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
        }
    }
}




            
void Disjunctive_Clause::report__newly_satisfied(State& state)
{
    report__newly_satisfied_literal(state);
}

void Disjunctive_Clause::report__newly_unsatisfied(State& state)
{
    report__newly_unsatisfied_literal(state);
}

void Disjunctive_Clause::set__satisfied(State& state)
{
    state.get__clauses__satisfaction_status().satisfy(get__id());
}

void Disjunctive_Clause::set__unsatisfied(State& state)
{
    state.get__clauses__satisfaction_status().unsatisfy(get__id());
}

void Disjunctive_Clause::flip_satisfaction(State& state)
{
    state.get__clauses__satisfaction_status().flip_satisfaction(get__id());
}
            
bool Disjunctive_Clause::is_satisfied(const State& state) const
{
    return state.get__clauses__satisfaction_status().satisfied(get__id());
}

void Disjunctive_Clause::increment__level_of_satisfaction(State&state)
{
    state.get__clauses__count_status().increment_satisfaction(get__id());
}

void Disjunctive_Clause::decrement__level_of_satisfaction(State&state)
{
    state.get__clauses__count_status().decrement_satisfaction(get__id());
}

void Disjunctive_Clause::set__level_of_satisfaction(uint value, State&state)
{
    state.get__clauses__count_status().set_satisfaction(get__id(), value);
}

uint Disjunctive_Clause::get__level_of_satisfaction(State&state) const
{
    return state.get__clauses__count_status().get_satisfaction_level(get__id());
}



void Conjunctive_Normal_Form_Formula::report__newly_satisfied(State& state)
{
    report__newly_satisfied_clause(state);
}

void Conjunctive_Normal_Form_Formula::report__newly_unsatisfied(State& state)
{
    report__newly_unsatisfied_clause(state);
}

void Conjunctive_Normal_Form_Formula::set__satisfied(State& state)
{
    state.get__cnfs__satisfaction_status().satisfy(get__id());
}

void Conjunctive_Normal_Form_Formula::set__unsatisfied(State& state)
{
    state.get__cnfs__satisfaction_status().unsatisfy(get__id());
}

void Conjunctive_Normal_Form_Formula::flip_satisfaction(State& state)
{
    state.get__cnfs__satisfaction_status().flip_satisfaction(get__id());
}
            
bool Conjunctive_Normal_Form_Formula::is_satisfied(const State& state) const
{
    return state.get__cnfs__satisfaction_status().satisfied(get__id());
}

void Conjunctive_Normal_Form_Formula::increment__level_of_satisfaction(State&state)
{
    state.get__cnfs__count_status().increment_satisfaction(get__id());
}

void Conjunctive_Normal_Form_Formula::decrement__level_of_satisfaction(State&state)
{
    state.get__cnfs__count_status().decrement_satisfaction(get__id());
}

void Conjunctive_Normal_Form_Formula::set__level_of_satisfaction(uint value, State&state)
{
    state.get__cnfs__count_status().set_satisfaction(get__id(), value);
}

uint Conjunctive_Normal_Form_Formula::get__level_of_satisfaction(State&state) const
{
    return state.get__cnfs__count_status().get_satisfaction_level(get__id());
}





uint Disjunctive_Clause::get__number_of_satisfied_literals(State& state) const
{return get__level_of_satisfaction(state);}

void Disjunctive_Clause::report__newly_satisfied_literal(State& state)
{
    increment__level_of_satisfaction(state);

    if(!is_satisfied(state)){
        set__satisfied(state);

        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
        }
    }   
}

void Disjunctive_Clause::report__newly_unsatisfied_literal(State& state)
{
    decrement__level_of_satisfaction(state);

    if(0 == get__number_of_satisfied_literals(state)){
        set__unsatisfied(state);
        
        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
    }
}



const Literal& Disjunctive_Clause::get__literal(int i) const
{
    auto tmp = get__literals();

    return *tmp[i];
}

const List__Literals& Disjunctive_Clause::get__literals(void) const
{
    return std::tr1::get<0>(contents());
}

List__Literals& Disjunctive_Clause::get__literals(void)
{
    return std::tr1::get<0>(_contents());
}


uint Conjunctive_Normal_Form_Formula::get__number_of_satisfied_clauses(State& state) const
{
    return get__level_of_satisfaction(state);
}


void Conjunctive_Normal_Form_Formula::report__newly_satisfied_clause(State& state)
{
    increment__level_of_satisfaction(state);

    if(get__disjunctive_clauses().size() == get__number_of_satisfied_clauses(state)){
        set__satisfied(state);
        
        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
        }
    }   
}

void Conjunctive_Normal_Form_Formula::report__newly_unsatisfied_clause(State& state)
{
    decrement__level_of_satisfaction(state);

    if(is_satisfied(state)){
        set__unsatisfied(state);
        
        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
    }
}



const List__Disjunctive_Clause& Conjunctive_Normal_Form_Formula::get__disjunctive_clauses() const
{
    return std::tr1::get<0>(contents());
}

const Disjunctive_Clause&
Conjunctive_Normal_Form_Formula::get__disjunctive_clause(int i) const
{
    auto tmp = get__disjunctive_clauses();

    return *tmp[i].cxx_get<Disjunctive_Clause>();
}
