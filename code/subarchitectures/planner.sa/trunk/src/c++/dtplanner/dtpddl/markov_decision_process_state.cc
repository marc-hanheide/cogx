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

using namespace Planning;

Markov_Decision_Process_State::
Markov_Decision_Process_State
(uint propositions_count,
 uint function_count)
    :boolean_State(propositions_count),
     integer_State(function_count)
{
    INTERACTIVE_VERBOSER(true, 7000, "Made an MDP state with  :: "
                         <<propositions_count
                         <<" propositions.");
    
}

Markov_Decision_Process_State::
Markov_Decision_Process_State(const Markov_Decision_Process_State& markov_Decision_Process_State)
    :boolean_State(markov_Decision_Process_State.boolean_State),
     integer_State(markov_Decision_Process_State.integer_State),
     float_State(markov_Decision_Process_State.float_State)
{
}

void Markov_Decision_Process_State::
push__successor(uint _operator_index,
                 Markov_Decision_Process_State* successor_state,
                double probability_of_transition)
{
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

const Markov_Decision_Process_State::Successors& Markov_Decision_Process_State::
get__successors() const
{
    return successors;
}

const Markov_Decision_Process_State::Successor_Driver& Markov_Decision_Process_State::
get__successor_Driver() const
{
    return  successor_Driver;    
}

const Markov_Decision_Process_State::Successor_Probabilities& Markov_Decision_Process_State::
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
    integer_State.read(index);
}

void Markov_Decision_Process_State::set__int(uint index, int value)
{
    integer_State.write(index, value);
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
