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

using namespace Planning;
using namespace Planning::State_Formula;

Satisfaction_Listener::Satisfaction_Listener
(bool requires_explicit_forced_waking)
    :requires_explicit_forced_waking(requires_explicit_forced_waking)
{
    
}


void Satisfaction_Listener::satisfy_listeners(State& state) const
{
    for(auto listener = list__Listeners.begin()
            ; listener != list__Listeners.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
    }
}

void Satisfaction_Listener::unsatisfy_listeners(State& state) const
{
    for(auto listener = list__Listeners.begin()
            ; listener != list__Listeners.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
    }
}

void Satisfaction_Listener::wake_sleepers(State& state) const
{
    for(auto listener = list__Sleepers.begin()
            ; listener != list__Sleepers.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()->wake(state);
    }
}


void Satisfaction_Listener::set__statically_false(State& state) const
{
    /* Doesn't matter if it is a listener or a sleeper, it cannot be
     * satisfying if one of the clauses that constitute its
     * precondition are statically false.*/
    
    for(auto listener = list__Listeners.begin()
            ; listener != list__Listeners.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()->set__statically_false(state);
    }
    
    for(auto listener = list__Sleepers.begin()
            ; listener != list__Sleepers.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()->set__statically_false(state);
    }
}


// void Satisfaction_Listener::forced_wake_sleepers(State& state) const
// {
//     for(auto listener = list__sleepers.begin()
//             ; listener != list__sleepers.end()
//             ; listener++){
//         (*listener).cxx_get<Satisfaction_Listener>()->forced_wake(state);
//     }
// }


void  Satisfaction_Listener::
wake_sleepers_that_require_forcing(State& state) const
{
    for(auto listener = list__Sleepers.begin()
            ; listener != list__Sleepers.end()
            ; listener++){
        if((*listener).cxx_get<Satisfaction_Listener>()->does_require_explicit_forced_waking()){
            INTERACTIVE_VERBOSER(true, 9074, "Forced waking of :: "<<*listener<<std::endl);
            (*listener).cxx_get<Satisfaction_Listener>()->forced_wake(state);
        } else {
            INTERACTIVE_VERBOSER(true, 9074, "Ignoring non-forced listener :: "<<*listener<<std::endl);
        }
    }
}

bool  Satisfaction_Listener::
does_require_explicit_forced_waking() const
{
    return requires_explicit_forced_waking;
}


            
bool Satisfaction_Listener::add__sleeper(Satisfaction_Listener__Pointer& in)
{
    if(sleepers.find(in) != sleepers.end()){
        assert(sleepers.size());
        WARNING("Attempted to add "<<in
                <<" as a sleeper to a set that"<<std::endl
                <<"already contained an equivalent element.");

//         for(auto sleeper = sleepers.begin(); sleeper != sleepers.end(); sleeper++){
//             std::cerr<<in<<" "<<*sleeper<<std::endl;
//         }
//         {char ch; std::cin>>ch;};
        
        return false;
    }
    
    sleepers.insert(in);
    list__Sleepers.push_back(in);
    
    return true;
}

            
const List__Listeners& Satisfaction_Listener::get__traversable__sleepers() const 
{
    return list__Sleepers;
}

const Listeners& Satisfaction_Listener::get__searchable__sleepers() const 
{
    return sleepers;
}            

Satisfaction_Listener::~Satisfaction_Listener()
{
}

bool Satisfaction_Listener::add__listener(Satisfaction_Listener__Pointer& satisfaction_Listener__Pointer)
{
    if(listeners.find(satisfaction_Listener__Pointer) != listeners.end()){
        WARNING("Attempted to add "<<satisfaction_Listener__Pointer
                <<" as a listener to a set that"<<std::endl
                <<"already contained an equivalent element.");
        return false;
    }
    
    listeners.insert(satisfaction_Listener__Pointer);
    list__Listeners.push_back(satisfaction_Listener__Pointer);
    return true;
}
            
const List__Listeners& Satisfaction_Listener::get__traversable__listeners() const 
{
    return list__Listeners;
}

const Listeners& Satisfaction_Listener::get__searchable__listeners() const 
{
    return listeners;
}




