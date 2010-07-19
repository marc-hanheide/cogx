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

#ifndef PROBLEM_GROUNDING_HH
#define PROBLEM_GROUNDING_HH


#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_problem.hh"
#include "planning_action_schema.hh"

#include "basic_action.hh"


namespace Planning
{
    class Problem_Grounding
    {
    public:
        Problem_Grounding(Parsing::Problem_Data&, CXX__PTR_ANNOTATION(Parsing::Domain_Data));

        
        void groud_actions(); 
        
    private:

        /* Populate \member{}*/
        void ground_action_schema(const Planning::Action_Schema& actionSchema);

        /* Problem actions. */
        std::vector<CXX__PTR_ANNOTATION(State_Transformation)> state_transformations;
        
        /*Problem targetted by this solver.*/
        Parsing::Problem_Data& problem_Data;
        
        /* Domain data associated with \member{problem} (see
         * \member{preprocess}).*/
        CXX__PTR_ANNOTATION(Parsing::Domain_Data) domain_Data;
    };
}


#endif
