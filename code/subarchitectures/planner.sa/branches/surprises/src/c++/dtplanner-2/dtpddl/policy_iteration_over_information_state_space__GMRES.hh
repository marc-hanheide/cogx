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


#ifndef POLICY_ITERATION_OVER_INFORMATION_STATE_SPACE__GMRES_HH
#define POLICY_ITERATION_OVER_INFORMATION_STATE_SPACE__GMRES_HH


#include "partially_observable_markov_decision_process_state.hh"

#include "gmres.hh"

namespace Planning
{
    class Policy_Iteration__GMRES
    {
    public:
        Policy_Iteration__GMRES(Set_Of_POMDP_State_Pointers&,
                                double sink_state_penalty,
                                double discount_factor = 0.95);

//         void operator()();
        bool operator()();
        
        void reset__converged();
    private:
        GMRES gmres;
        
        void press_greedy_policy();
        void configure_reward_vector();
        void configure_transition_matrix();
        
        Set_Of_POMDP_State_Pointers& states;

        uint dimension;

        double discount_factor;
        double sink_state_penalty;
        
        GMRES::Vector instantanious_reward_vector;

        /* (I - A) where I is the identity matrix. */
        GMRES::Matrix state_transition_matrix;
        
        GMRES::Vector* value_vector;

        uint old__state_space_size;
        GMRES::Vector old__value_vector;
        bool converged;
        
    };
        
}


#endif

/* On July 7, 1981 the Solar Challenger, piloted by Steve Ptacek, flew
 * from Paris to Manston in the UK. This aircraft used solar energy as
 * its power source. */
