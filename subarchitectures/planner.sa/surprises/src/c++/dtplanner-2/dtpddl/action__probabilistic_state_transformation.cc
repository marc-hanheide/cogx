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


#include "action__probabilistic_state_transformation.hh"

#include "action__state_transformation.hh"


#include "planning_state.hh"

using namespace Planning;

std::vector<State*>
Probabilistic_State_Transformation::
operator()(State* input) const
{
    INTERACTIVE_VERBOSER(true, 9075, "Executing probabilistic action :: "
                         <<*this<<std::endl);
    
    auto local_list__Listeners = get__traversable__listeners();
    auto local_list__Sleepers = get__traversable__sleepers();
    
    std::vector<State*> result(local_list__Sleepers.size());

    uint index = 0;
//     for(auto _listener = list__Listeners.begin()
//             ; _listener != list__Listeners.end()
//             ; _listener++, index++){

    assert(local_list__Sleepers.size());
    assert(!local_list__Listeners.size());
    
    
    for(auto _listener = local_list__Sleepers.begin()
            ; _listener != local_list__Sleepers.end()
            ; _listener++, index++){
        auto listener = *_listener;
        
        State* new_state = (index == (local_list__Sleepers.size() - 1))
            ?input
            :(new State(*input));
        
        listener.cxx_get<Satisfaction_Listener>()->wake(*new_state);
//         listener.cxx_get<Satisfaction_Listener>()->report__newly_satisfied(*new_state);
        
        result[index] = new_state;
    }
    
    return std::move<>(result);
}

const Formula::Action_Proposition&
Probabilistic_State_Transformation::
get__identifier() const
{
    return std::tr1::get<0>(contents());
}


void Probabilistic_State_Transformation::forced_wake(State& state) const
{
    /* Probabilistic transformations are always satisfied. */
    state.push__probabilistic_transformation
        (this);
}

void Probabilistic_State_Transformation::
report__newly_satisfied(State& state) const
{
    assert(0);
    
//     state.push__probabilistic_transformation
//         (this);
}

void Probabilistic_State_Transformation::
report__newly_unsatisfied(State& state) const
{
    assert(0);
    /* NA -- A probabilistic transformation should only be activated
     * once during the computation of successor states under operator
     * execution.*/
}

std::ostream& Probabilistic_State_Transformation::operator<<(std::ostream&o) const
{

    o<<"PROBABILISTIC :: "<<get__identifier().operator<<(o)<<std::endl;
    
    o<<std::endl;
    o<<"{";
    for(auto listener = get__traversable__listeners().begin()
            ; listener != get__traversable__listeners().end()
            ; listener++){
        if(listener->test_cast<State_Transformation>()){
            listener->cxx_get<State_Transformation>()->operator<<(o);
        } else if (listener->test_cast<Probabilistic_State_Transformation>()) {
            o<<*(listener->cxx_get<Probabilistic_State_Transformation>());//->operator<<(o);
        }
    }
    o<<"}"<<std::endl;
    
    return o;
}

namespace std
{
    
    std::ostream& operator<<(std::ostream&o
                             , const Planning::Probabilistic_State_Transformation&in)
    {
        return in.operator<<(o);
    }
    
}


