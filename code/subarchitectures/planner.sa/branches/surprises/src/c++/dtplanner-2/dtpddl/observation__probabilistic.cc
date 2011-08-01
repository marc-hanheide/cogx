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


#include "observation__probabilistic.hh"

#include "observation.hh"
#include "planning_state.hh"
#include "planning_observation.hh"

using namespace Planning;

const Formula::Observational_Proposition& Probabilistic_Observation::get__identifier() const
{
    return std::tr1::get<0>(contents());
}

std::vector<Planning::Observational_State*> Probabilistic_Observation::
operator()
    (Planning::Observational_State* input,
     Planning::State* state) const
{
    
    auto local_list__Sleepers = get__traversable__sleepers();
    auto local_list__Listeners = get__traversable__listeners();
    std::vector<Planning::Observational_State*> result(local_list__Sleepers.size());

    assert(local_list__Sleepers.size());
    assert(!local_list__Listeners.size());
    
    uint index = 0;
    for(auto _listener = local_list__Sleepers.begin()
            ; _listener != local_list__Sleepers.end()
            ; _listener++, index++){
        auto listener = *_listener;
        
        Planning::Observational_State* new_observation = (index == (local_list__Sleepers.size() - 1))
            ?input
            :(new Planning::Observational_State(*input));
        
        state->set__observational_state_during_expansion(new_observation);
        listener.cxx_get<Satisfaction_Listener>()->wake(*state);
        
        result[index] = new_observation;
    }
    
    return std::move<>(result);
}

void Probabilistic_Observation::forced_wake(State& state) const
{
    state.get__observational_state_during_expansion()
        ->push__probabilistic_observation
        (this);
}


void Probabilistic_Observation::report__newly_satisfied(State& state) const
{
    assert(0);
//     state.push__probabilistic_observation
//         (this);
}

void Probabilistic_Observation::report__newly_unsatisfied(State&) const
{
    assert(0);
    /* NA -- A probabilistic transformation should only be activated
     * once during the computation of successor states under operator
     * execution. Therefore, its truth status is not maintained in
     * planning states.*/
}

std::ostream& Probabilistic_Observation::operator<<(std::ostream&o) const
{
    o<<get__identifier()<<std::endl;
    
    auto& listeners = get__traversable__listeners();
    for(auto listener = listeners.begin()
            ; listener != listeners.end()
            ; listener++){
        o<<*listener<<std::endl;
    }
    o<<std::endl;
    o<<"{"<<get__traversable__sleepers().size()<<"-SLEEPERS :: ";
    for(auto listener = get__traversable__sleepers().begin()
            ; listener != get__traversable__sleepers().end()
            ; listener++){
        if(listener->test_cast<basic_type>()){
            listener->cxx_get<basic_type>()->operator<<(o);
        }
        
    }
    o<<"}"<<std::endl;
    
    
    return o;
}





namespace std
{
    
    std::ostream& operator<<(std::ostream&o
                             , const Planning::Probabilistic_Observation&in)
    {
        return in.operator<<(o);
    }
    
}


