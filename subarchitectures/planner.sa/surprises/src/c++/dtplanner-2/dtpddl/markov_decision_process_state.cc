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

#include "markov_decision_process_state.hh"
#include "planning_observation.hh"

#ifndef NDEBUG
#include "planning_state.hh"
#include "action__state_transformation.hh"
#endif

using namespace Planning;



double Markov_Decision_Process_State::get__value() const
{
    return this->value;
}

void Markov_Decision_Process_State::set__value(double in)
{
    this->value = in;
}


Markov_Decision_Process_State::
Markov_Decision_Process_State
(uint propositions_count,
 uint function_count)
    :boolean_State(propositions_count),
     integer_State(function_count),/*FIX:: Floats and ints need to be dealt with separately. */
//      float_State(function_count),/*FIX:: Floats and ints need to be dealt with separately. */
     reward(0.0),
     value(0.0)
{
    INTERACTIVE_VERBOSER(true, 7000, "Made an MDP state with  :: "
                         <<propositions_count
                         <<" propositions.");
    
}

Markov_Decision_Process_State::
Markov_Decision_Process_State(const Markov_Decision_Process_State& markov_Decision_Process_State)
    :boolean_State(markov_Decision_Process_State.boolean_State),
     integer_State(markov_Decision_Process_State.integer_State),
     float_State(markov_Decision_Process_State.float_State),
     reward(markov_Decision_Process_State.reward),
     value(markov_Decision_Process_State.value)
{
}

const Markov_Decision_Process_State::Successor_Driver&
Markov_Decision_Process_State::
get__sorted__successor_Driver() const
{
    if(sorted__successor_Driver.size() != successor_Driver.size()){
        sorted__successor_Driver = Successor_Driver(successor_Driver.begin(), successor_Driver.end());
        sort(sorted__successor_Driver.begin(), sorted__successor_Driver.end());
    }

    return sorted__successor_Driver;
}


void Markov_Decision_Process_State::
push__successor(uint _operator_index,
                 Markov_Decision_Process_State* successor_state,
                double probability_of_transition)
{

#ifndef NDEBUG
    State* tmp = dynamic_cast<State*>(this);
    auto executable_action_indices = tmp->get__optional_transformations();

    bool found = false;
    for(auto driver = executable_action_indices.begin()
            ; driver != executable_action_indices.end()
            ; driver++){
        if((*driver)->get__id() == _operator_index){
            found = true; break;
        }
    }
    INTERACTIVE_VERBOSER(!found,
                         10014, 
                         "Action :: "<<_operator_index<<std::endl
                         <<"Is not available at state unless it is always executable :: "<<*tmp<<std::endl);

    INTERACTIVE_VERBOSER(true, 10014,
                         "Pushing action  :: "<<_operator_index<<std::endl
                         <<"At state :: "<<*dynamic_cast<State*>(this));
#endif
    
    
    if(successor_Driver.size()){    
        if(successor_Driver.back() != _operator_index){
            successor_Driver.push_back(_operator_index);
        }
    } else {
        successor_Driver.push_back(_operator_index);
    }    

//     auto operator_index = successor_Driver.back();

    if(successors.size() != successor_Driver.size()){// operator_index){
        successors
            .push_back(std::vector< Markov_Decision_Process_State*>());

        assert(successor_Probabilities.size() < successor_Driver.size());
        successor_Probabilities
            .push_back(std::vector<double>());
    }

    assert(successors.size() == successor_Driver.size());
    assert(successors.size() == successor_Probabilities.size());
       
    assert(successor_Driver.size() - 1 < successors.size());
    assert(successor_Driver.size() - 1 < successor_Probabilities.size());
    
    successors[successor_Driver.size() - 1].push_back(successor_state);
    successor_Probabilities[successor_Driver.size() - 1].push_back(probability_of_transition);
}


const std::vector<std::vector<Observational_State*> >& Markov_Decision_Process_State::get__observations() const
{
    return observation__given_action;
}

const std::vector<std::vector<double> >& Markov_Decision_Process_State::get__observation_Probabilities() const
{
    return observation_probability__given_action;
}

const std::vector<uint>& Markov_Decision_Process_State::get__observation_Driver() const
{
    return action_to_observation;
}


const Markov_Decision_Process_State::Successors&
Markov_Decision_Process_State::
get__successors() const
{
    return successors;
}

const Markov_Decision_Process_State::Successor_Driver&
Markov_Decision_Process_State::
get__successor_Driver() const
{
    return  successor_Driver;    
}

const Markov_Decision_Process_State::Successor_Probabilities&
Markov_Decision_Process_State::
get__successor_Probabilities() const
{
    return successor_Probabilities;
}


uint Markov_Decision_Process_State::get__number_of_atoms() const
{
    INTERACTIVE_VERBOSER(true, 7000, "Getting atom count associated with MDP state.");
    return boolean_State.get__number_of_atoms(); 
}

uint Markov_Decision_Process_State::get__number_of_int_fluents() const
{
    return integer_State.size();
}


uint Markov_Decision_Process_State::get__number_of_double_fluents() const
{
    return float_State.size();
}


bool Markov_Decision_Process_State::operator==(const Markov_Decision_Process_State& state) const
{
    return ((boolean_State == state.boolean_State) &&
            (integer_State == state.integer_State) &&
            (float_State == state.float_State));
}

bool Markov_Decision_Process_State::operator<(const Markov_Decision_Process_State& state) const
{
    if(boolean_State < state.boolean_State){
        return true;
    } else if(boolean_State == state.boolean_State){
        if(integer_State < state.integer_State){
            return true;
        } else if (integer_State == state.integer_State) {
            if(float_State < state.float_State){
                return true;
            }
        }
    }

    return false;   
}

std::size_t Markov_Decision_Process_State::hash_value() const
{
    std::size_t seed = boolean_State.hash_value();
    boost::hash_combine(seed, integer_State.hash_value());
    boost::hash_combine(seed, float_State.hash_value());

    return seed;
}

std::size_t std::hash_value(const Planning::Markov_Decision_Process_State& markov_Decision_Process_State)
{
    return markov_Decision_Process_State.hash_value();
}



void Markov_Decision_Process_State::flip(uint in)
{
    boolean_State.flip(in);
}

bool Markov_Decision_Process_State::is_true(uint in) const
{
    return boolean_State.is_true(in);
}

double Markov_Decision_Process_State::get__float(uint index) const
{
    return float_State.read(index);
}

void Markov_Decision_Process_State::set__float(uint index, double value)
{
    return float_State.write(index, value);
}

        
int Markov_Decision_Process_State::get__int(uint index) const
{
    return integer_State.read(index);
}

void Markov_Decision_Process_State::set__int(uint index, int value)
{
    
    integer_State.write(index, value);

    
    INTERACTIVE_VERBOSER(true, 9096, "Writing to integer state :: "
                         <<index<<" "<<value<<std::endl);
        
}


double Markov_Decision_Process_State::get__reward() const
{
    return reward;
}

void Markov_Decision_Process_State::set__reward(double reward)
{
    this->reward = reward;
}


void Markov_Decision_Process_State::
report__considered_observations_under_action(uint action_id)
{
    considered_observations_under_action.insert(action_id); 
}

bool Markov_Decision_Process_State::
has__considered_observations_under_action(uint action_id) const
{
    return (considered_observations_under_action.find(action_id)
            != considered_observations_under_action.end());
}

uint Markov_Decision_Process_State::
get__action_to_observation__index(uint action_id) const
{
    auto tmp = mirror__action_to_observation.find(action_id);
    assert(tmp != mirror__action_to_observation.end());

    return tmp->second;
}

bool Markov_Decision_Process_State::
action_to_observation__includes_index(uint action_id) const
{
    auto tmp = mirror__action_to_observation.find(action_id);

    return (tmp != mirror__action_to_observation.end());
}



void Markov_Decision_Process_State::
push__observation(uint action_id,
                  Observational_State* observation,
                  double probability)
{
   
    if(action_to_observation.size()){    
        if(action_to_observation.back() != action_id){
            mirror__action_to_observation[action_id]
                = action_to_observation.size();
            
            action_to_observation.push_back(action_id);
        }
    } else {
        mirror__action_to_observation[action_id]
            = action_to_observation.size();
        
        action_to_observation.push_back(action_id);
    }

    assert(observation__given_action.size()
           == observation_probability__given_action.size());
    
    if(observation__given_action.size() != action_to_observation.size()){// operator_index){
        
        assert(observation__given_action.size() < action_to_observation.size());
        observation__given_action.push_back(std::vector<Observational_State*>());
        observation_probability__given_action.push_back(std::vector<double>());

    }
    
    observation__given_action[action_to_observation.size() - 1]
        .push_back(observation);
    observation_probability__given_action[action_to_observation.size() - 1]
        .push_back(probability);
}


std::ostream& Markov_Decision_Process_State::operator<<(ostream& o) const
{
    o<<boolean_State<<std::endl;
    o<<integer_State<<std::endl;
    o<<float_State<<std::endl;

    return o;
}

namespace std
{
    
    /* (see \module{markov_decision_process_state.hh}) */
    std::ostream& operator<<(ostream& o, const Planning::Markov_Decision_Process_State& in)
    {
        return in.operator<<(o);
    }
}
