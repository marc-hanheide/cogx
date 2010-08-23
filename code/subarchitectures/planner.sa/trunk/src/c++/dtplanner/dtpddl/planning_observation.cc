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


#include "planning_observation.hh"


using namespace Planning;

Observational_State::Observational_State(uint number_of_perceptual_propositions)
    :Boolean_State(number_of_perceptual_propositions)
{
}

std::stack<const Observation*>& Observational_State::get__observations()
{
    return observations;
}

std::stack<const Probabilistic_Observation*>& Observational_State::get__probabilistic_observations()
{
    return probabilistic_observations;
}


void Observational_State::reset__observations()
{
    observations = std::stack<const Observation*>();
}

void Observational_State::reset__probabilistic_observations()
{
    probabilistic_observations = std::stack<const Probabilistic_Observation*>();
}

void Observational_State::take__observations__from( Observational_State* in)
{
    replace__observations(in->get__observations());
    in->reset__observations();
//     std::stack<const Observation*>& tmp = const_cast<std::stack<const Observation*>&>(in->observations);
//     replace__observations(tmp);
//     in->reset__observations();
}

void Observational_State::take__probabilistic_observations__from( Observational_State* in)
{
    replace__probabilistic_observations(in->get__probabilistic_observations());
    in->reset__probabilistic_observations();
//     std::stack<const Probabilistic_Observation*>& tmp
//         = const_cast<std::stack<const Probabilistic_Observation*>&>(in->probabilistic_observations);
//     replace__probabilistic_observations(tmp);
//     in->reset__probabilistic_observations();
}

void Observational_State::replace__observations(std::stack<const Observation*>& in)
{
    observations = std::move<>(in);
}

void Observational_State::replace__probabilistic_observations(std::stack<const Probabilistic_Observation*>& in)
{
    probabilistic_observations = std::move<>(in);
}


uint Observational_State::count__probabilistic_observations() const
{
    return probabilistic_observations.size();
}

const Probabilistic_Observation* Observational_State::pop__probabilistic_observation()
{
    auto result = probabilistic_observations.top();
    probabilistic_observations.pop();
    return result;
}

void Observational_State::push__probabilistic_observation(const Probabilistic_Observation* in)
{
   probabilistic_observations.push(in); 
}


uint Observational_State::count__observations() const
{
    return observations.size();
}

const Observation* Observational_State::pop__observation()
{
    auto result = observations.top();
    observations.pop();
    return result;
}

void Observational_State::push__observation(const Observation* in)
{
    observations.push(in);
}

// uint Observational_State::get__id() const
// {
//     return id;
// }

// void Observational_State::set__id(uint _id)
// {
//     id = _id;
// }


namespace Planning
{

    std::size_t hash_value(const Observational_State& observational_State)
    {
        return observational_State.hash_value();
    }  
}

namespace std
{
    std::size_t hash_value(const Planning::Observational_State& observational_State)
    {
        return observational_State.hash_value();
    }   
}
