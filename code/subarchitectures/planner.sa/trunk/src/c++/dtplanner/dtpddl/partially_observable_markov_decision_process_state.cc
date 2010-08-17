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


#include "partially_observable_markov_decision_process_state.hh"

using namespace Planning;


bool Partially_Observable_Markov_Decision_Process_State::
operator==(const POMDP_State& in) const
{
    return ( (in.expected_value == expected_value) &&
             (in.belief_State.size() == belief_State.size()) &&
             (in.belief_State == belief_State) );
}

bool Partially_Observable_Markov_Decision_Process_State::
operator<(const POMDP_State& in) const
{
    if(this->expected_value < in.expected_value){
        return true;
    } else if (this->expected_value == in.expected_value) {

        if(belief_State.size() < in.belief_State.size()){
        } else if (belief_State.size() == in.belief_State.size()) {
            auto atom_lhs = belief_State.begin();
            auto atom_rhs = in.belief_State.begin();
            for(
                    ; atom_lhs != belief_State.end()
                    ; atom_lhs++, atom_rhs++){
                if(atom_lhs->second < atom_rhs->second){
                    return true;
                } else if (atom_lhs->second == atom_rhs->second) {
                    if(static_cast< void*>(atom_lhs->first) <
                       static_cast< void*>(atom_rhs->first)){
                    } else if (atom_lhs->first == atom_rhs->first) {
                        continue;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
    }

    return false;
}

    
void Partially_Observable_Markov_Decision_Process_State::
add__belief_atom( MDP_State* mdp__state, double probability)
{
    belief_State.push_back(Belief_Atom(mdp__state, probability));
}

const Partially_Observable_Markov_Decision_Process_State::
Belief_State&
Partially_Observable_Markov_Decision_Process_State::
get__belief_state() const
{
    return belief_State;
}


std::size_t  Partially_Observable_Markov_Decision_Process_State::
hash_value() const
{
    return boost::hash_range(belief_State.begin(), belief_State.end());
}


namespace std
{
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State& in)
    {
        return in.hash_value();
    }
    
    std::ostream& operator<<(std::ostream&o,
                             const Planning::Partially_Observable_Markov_Decision_Process_State& pomdp__state)
    {
        auto state = pomdp__state.get__belief_state();

        for(auto s = state.begin(); s != state.end(); s++){
            o<<s->second<<" :--:> "<<*(s->first)<<std::endl;
        }
        
        return o;
    }
    
}
