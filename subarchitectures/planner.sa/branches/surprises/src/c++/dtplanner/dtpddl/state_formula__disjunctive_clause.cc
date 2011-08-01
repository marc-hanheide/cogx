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


#include "state_formula__disjunctive_clause.hh"


#include "state_formula__literal.hh"
#include "planning_state.hh"
// #include "planning_types_enum.hh"


using namespace Planning;
using namespace Planning::State_Formula;
      

void Disjunctive_Clause::report__newly_satisfied(State& state) const
{
    report__newly_satisfied_literal(state);
}

void Disjunctive_Clause::report__newly_unsatisfied(State& state) const
{
    report__newly_unsatisfied_literal(state);
}

void Disjunctive_Clause::set__satisfied(State& state) const
{
    assert(state.get__clauses__satisfaction_status().valid_index(get__id()));
    state.get__clauses__satisfaction_status().satisfy(get__id());
}

void Disjunctive_Clause::set__unsatisfied(State& state) const
{
    assert(state.get__clauses__satisfaction_status().valid_index(get__id()));
    state.get__clauses__satisfaction_status().unsatisfy(get__id());
}

void Disjunctive_Clause::flip(State& state) const
{
    WARNING("Something just changed the Boolean status value associated with a"<<std::endl
            <<"clause by an unusual mechanism -- "<<std::endl
            <<"i.e., the satisfaction for a literal was not reported changed.");
    
    assert(state.get__clauses__satisfaction_status().valid_index(get__id()));
    state.get__clauses__satisfaction_status().flip_satisfaction(get__id());
}
            
bool Disjunctive_Clause::is_satisfied(const State& state) const
{
    assert(state.get__clauses__satisfaction_status().valid_index(get__id()));
    return state.get__clauses__satisfaction_status().satisfied(get__id());
}

void Disjunctive_Clause::increment__level_of_satisfaction(State&state) const
{
    assert(state.get__clauses__count_status().valid_index(get__id()));
    state.get__clauses__count_status().increment_satisfaction(get__id());
}

void Disjunctive_Clause::decrement__level_of_satisfaction(State&state) const
{
    assert(state.get__clauses__count_status().valid_index(get__id()));
    assert(get__level_of_satisfaction(state));
    state.get__clauses__count_status().decrement_satisfaction(get__id());
}

void Disjunctive_Clause::set__level_of_satisfaction(uint value, State&state) const
{
    assert(state.get__clauses__satisfaction_status().valid_index(get__id()));
    state.get__clauses__count_status().set_satisfaction(get__id(), value);
}

uint Disjunctive_Clause::get__level_of_satisfaction(State&state) const
{
    assert(state.get__clauses__count_status().valid_index(get__id()));
    return state.get__clauses__count_status().get_satisfaction_level(get__id());
}


uint Disjunctive_Clause::get__number_of_satisfied_literals(State& state) const
{return get__level_of_satisfaction(state);}

void Disjunctive_Clause::report__newly_satisfied_literal(State& state) const
{
    increment__level_of_satisfaction(state);

    if(!is_satisfied(state)){
        set__satisfied(state);
        satisfy_listeners(state);
    }   
}

void Disjunctive_Clause::report__newly_unsatisfied_literal(State& state) const
{
    decrement__level_of_satisfaction(state);

    assert(get__traversable__listeners().size());
    
    if(0 == get__number_of_satisfied_literals(state)){
        set__unsatisfied(state);
        unsatisfy_listeners(state);

        {/* Code for the case that the constituent literals become
          * statically false. Running this code every time is not
          * efficient. FIX :: store the result
          * \local{found_to_be_statically_false}. */    
            bool found_to_be_statically_false = true;
            auto& literals = get__literals();//get__traversable__listeners();
            for(auto literal  = literals.begin()
                    ; literal != literals.end()
                    ; literal++){
                
                if(!(*literal)->get__can_only_be_flipped_once()){
                    found_to_be_statically_false = false;
                    break;
                }
            }
            
            if(found_to_be_statically_false){
                set__statically_false(state);
            }
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



std::ostream& Disjunctive_Clause::operator<<(std::ostream&o) const
{
    o<<"CLAUSE::";
    for(auto literal = get__literals().begin()
            ; literal != get__literals().end()
            ; literal++){
        o<<*literal<<", ";
    }
    
    auto listeners = get__traversable__listeners();
    if(!listeners.size()){
        o<<"Clause has no listeners :: "<<std::endl;
    }
//     for(auto listener = listeners.begin()
//             ; listener != listeners.end()
//             ; listener ++){
//         assert((*listener).get() != this);
//         INTERACTIVE_VERBOSER(true, 9092, " has listener :: "<<*listener<<std::endl);
        
//     }
    
    return o;
}
    
namespace std
{
    std::ostream& operator<<(std::ostream&o, const Planning::State_Formula::Disjunctive_Clause&in)
    {
        return in.operator<<(o);
    }
}
