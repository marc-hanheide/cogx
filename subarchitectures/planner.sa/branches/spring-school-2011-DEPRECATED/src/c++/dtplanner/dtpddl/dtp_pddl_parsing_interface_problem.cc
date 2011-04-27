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
#include "dtp_pddl_parsing_interface.hh"

#include "dtp_pddl_problem.hh"
#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_domain.hh"

using namespace pegtl;
using namespace Planning::Parsing;

/* We assume a domain has already been parsed when we are given a
 * problem to parse. By default, the domain associated with the
 * problem is whatever is at
 * \global{Planning::Parsing::domain_stack}.*/
void Planning::Parsing::parse_problem(const std::string& problem_file_name)
{
    using Planning::Parsing::domain_Stack;
    
//     assert(domain_Stack.use_count());    

    QUERY_UNRECOVERABLE_ERROR(!domain_Stack.use_count(), "We cannot see that a domain was parsed"<<std::endl
                              <<"And yet we are trying to parse problem :: "<<problem_file_name<<std::endl);

    Planning::Parsing::problem_Stack.reset(new Problem_Stack(domain_Stack));
    
    std::string preprocessed_file_contents = pddl_preprocessor(problem_file_name);

    VERBOSER(501, "Got preprocessed problem file :: "<<preprocessed_file_contents<<std::endl);
    
//     pegtl::smart_parse_file< Planning::Parsing::PDDL_Preamble_Problem >
//         ( true/*_trace*/, problem_file_name, *Planning::Parsing::problem_Stack );
    
    pegtl::smart_parse_string< Planning::Parsing::PDDL_Preamble_Problem >
        ( true/*_trace*/,
          preprocessed_file_contents,
          *Planning::Parsing::problem_Stack );

    
    VERBOSER(501, "Finished parsing problem :: "<<problem_file_name<<std::endl);

    Problem_Identifier identifier(problem_Stack->get__domain_Data()->get__domain_Name(),
                                  problem_Stack->get__problem_Name());
    
    problems[identifier] = Planning::Parsing::problem_Stack;

    
//     QUERY_UNRECOVERABLE_ERROR
//         (Planning::Parsing::problems.find(pi) == Planning::Parsing::problems.end()
//          , "DTP Could not find problem for task :: "<<id<<std::endl);
}
