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


#ifndef PARTIALLY_OBSERVABLE_MARKOV_DECISION_PROCESS_STATE_HH
#define PARTIALLY_OBSERVABLE_MARKOV_DECISION_PROCESS_STATE_HH


#include "markov_decision_process_state.hh"

#include "planning_observation.hh"

namespace Planning
{
    
    class Partially_Observable_Markov_Decision_Process_State;
}

namespace std
{
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State&);
}


namespace Planning
{
    
    class Partially_Observable_Markov_Decision_Process_State;
    
    inline std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State& in)
    {
        return std::hash_value(in);
    }

    class Partially_Observable_Markov_Decision_Process_State
    {
    public:
        typedef Markov_Decision_Process_State MDP_State;
        typedef Partially_Observable_Markov_Decision_Process_State POMDP_State;

        typedef std::pair<MDP_State*, double> Belief_Point;
        typedef std::vector<Belief_Point> Belief_State;
        
        bool operator==(const POMDP_State&) const;
        bool operator<(const POMDP_State&) const;
	std::size_t hash_value() const;

        
        
        const Belief_State& get__belief_state() const;
        
    private:
        std::vector<uint> action_based_successor_driver;

        /* For each \member{action_based_successor_driver}, we store
         * the observation based successor drivers.*/
        std::vector< std::vector<uint> > observation_based_successor_driver;
        
        std::vector<
            std::vector<
                std::vector<
                    const Partially_Observable_Markov_Decision_Process_State*> > > successors;
        
        std::vector< 
            std::vector<
                std::vector<
                    double> > >  successor_probability;

        
        /*Expected value of this POMDP state.*/
        double expected_value;

        std::vector<std::pair<MDP_State*, double> > belief_State;
    };
}


#endif
