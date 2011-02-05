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
 * CogX ::
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

#ifndef BELIEF_STATE_EVALUATION_HH
#define BELIEF_STATE_EVALUATION_HH

#include "partially_observable_markov_decision_process_state.hh"

namespace Planning
{
    class MDP_Heuristic
    {
    public:
        float operator()(POMDP_State*) const;
    };
    
    class Belief_State_Value
    {
    public:
        double operator()(POMDP_State*) const;
    };
    
    class Obtainable_Value
    {
    public:
        double operator()(POMDP_State*) const;
    };

    class Obtainable_Values_Count
    {
    public:
        double operator()(POMDP_State*) const;
    };

    class Entropy_Heuristic
    {
    public:
        float operator()(POMDP_State*) const;
    };

    class Greedy_Heuristic
    {
    public:
        Greedy_Heuristic();
        
        
        float operator()(POMDP_State*) const;
    private:
        mutable bool configured__expected_rewards_count__cache;
        mutable double expected_rewards_count__cache;

        mutable bool configured__expected_rewards_value__cache;
        mutable double expected_rewards_value__cache;
    };

    
    
    
//     class Proximity_To_Potential_Value
//     {
//     public:
//         double operator()(POMDP_State*);
//     };
}



#endif
