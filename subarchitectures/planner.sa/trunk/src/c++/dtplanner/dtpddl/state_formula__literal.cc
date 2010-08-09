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

#include "state_formula__literal.hh"


#include "planning_state.hh"
// #include "planning_types_enum.hh"
#include "planning_formula.hh"


using namespace Planning;
using namespace Planning::State_Formula;
      

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

    auto listeners = get__traversable__listeners();
    if(was_satisfied){
        for(auto listener = listeners.begin()
                ; listener != listeners.end()
                ; listener++){
            (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
    } else {
        for(auto listener = listeners.begin()
                ; listener != listeners.end()
                ; listener++){
            (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
        }
    }
}


std::ostream& Literal::operator<<(std::ostream&o) const
{

    if(Formula::State_Proposition::ith_exists(get__runtime_Thread(), get__variable())){
        auto proposition = Formula::State_Proposition
            ::make_ith<Formula::State_Proposition>(get__runtime_Thread()
                                          , get__variable());
        o<<((get__sign())?"¬":"")<<proposition;
    } else {
        o<<((get__sign())?"¬":"")<<get__variable();
    }
    
    
    return o;
}
    

namespace std
{
    std::ostream& operator<<(std::ostream&o, const Planning::State_Formula::Literal&in)
    {
        return in.operator<<(o);
    }
    
}
