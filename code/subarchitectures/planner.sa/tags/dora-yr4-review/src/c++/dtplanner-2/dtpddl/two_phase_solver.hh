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


#ifndef TWO_PHASE_SOLVER_HH
#define TWO_PHASE_SOLVER_HH

#include "solver.hh"

#include "ordered_structure_over_states.hh"
#include "belief_state_evaluation.hh"

namespace Planning
{
    class Two_Phase_Solver : public Planning::Solver
    {
    public:
        Two_Phase_Solver(Planning::Parsing::Problem_Data&);

        typedef Ordered_Stack_Of_States<POMDP_State, MDP_Heuristic, float> LOCAL__Ordered_Stack_Of_States;
        
        LOCAL__Ordered_Stack_Of_States ordered_Stack_Of_States;
        
        void report__new_belief_state(POMDP_State* );
        POMDP_State* obtain__next_belief_state_for_expansion();
        POMDP_State* peek__next_belief_state_for_expansion();

        /* Flip between phases (see \member{phase_one})*/
        void change_phase();
        
        
        void empty__belief_states_for_expansion();
        void reinstate__starting_belief_state();
    private:
        bool phase_one;
    };
}


#endif
