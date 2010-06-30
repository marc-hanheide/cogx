/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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


#ifndef SOLVER_HH
#define SOLVER_HH

#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_problem.hh"

namespace Planning
{
    class Solver
    {
    public:
        /*Problem that is the target of the solution procedure.*/
        Solver(Planning::Parsing::Problem_Data&);

        /*
         *
         * - Merge instance data from the \member{problem} and its
         * associated problem.
         *
         */
        void preprocess();

        /*Is this solver in a sane state?*/
        bool sanity() const;

    private:

        /* - Add \member{domain_Data::constants} and assocaited data to
         * \member{problem_Data}.*/
        void proprocess__Constants_Data();
        
    private:
        
        /* PDDL types for \member{constants}*/
        C__PTR_ANNOTATION(Planning::Parsing::Problem_Data::Constants_Description) constants_Description;
        
        /* PDDL objects and constants.*/
        C__PTR_ANNOTATION(Constants) constants;
        
        /*(see \member{preprocess})*/
        bool preprocessed;

        /*Problem targetted by this solver.*/
        Planning::Parsing::Problem_Data& problem_Data;
        
        /* Domain data associated with \member{problem} (see
         * \member{preprocess}).*/
        CXX__PTR_ANNOTATION(Planning::Parsing::Domain_Data) domain_Data;
    };
}

#endif
