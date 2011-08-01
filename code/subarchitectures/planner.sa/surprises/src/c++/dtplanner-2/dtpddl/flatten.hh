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



#ifndef FLATTEN_HH
#define FLATTEN_HH


#include "solver.hh"

#include "ordered_structure_over_states.hh"
#include "belief_state_evaluation.hh"

namespace Planning
{
    class Flatten : public Planning::Solver
    {
    public:
        Flatten(Planning::Parsing::Problem_Data&);

        void print_flat_problem();
    private:
        std::set<int> action_ids;
        
        int state_id;
        std::map<MDP_State*, int> state_ids;
        
        int observation_id;
        std::map<Observational_State*, int> observation_ids;

        std::map<int, double> starting;

        typedef std::tr1::tuple<int/*action*/, int/*state*/, int/*obs*/> Obs_Trans;
        std::map<Obs_Trans, double> observation_prob;

        
        typedef std::tr1::tuple<int/*action*/, int/*state*/, int/*state*/> State_Trans;
        std::map<State_Trans, double> state_prob;
    };
}


#endif
