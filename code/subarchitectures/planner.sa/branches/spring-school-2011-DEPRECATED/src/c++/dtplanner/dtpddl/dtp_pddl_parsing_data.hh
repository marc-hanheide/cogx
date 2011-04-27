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
#ifndef DTP_PDDL_PARSING_DATA_HH
#define DTP_PDDL_PARSING_DATA_HH


#include "global.hh"
#include "planning_symbols.hh"
#include "planning_formula.hh"

namespace Planning
{
    namespace Parsing
    {
        /* (see \module{dtp_pddl_parsing_data_types}) */
        class Types_Data;
        
        /* (see \module{dtp_pddl_parsing_data_formula}). Virtually
         * inherits from \class{Types_Data}.*/
        class Formula_Data;
        
        /* (see \module{dtp_pddl_parsing_data_constants}) Virtually
         * inherits from \class{Types_Data}. */
        class Constants_Data;
        
        /* (see \module{dtp_pddl_parsing_data_domainn}). Inherits from
         * \class{Formula_Data} and \class{Constants_Data}. */
        class Domain_Data;
        
        /* (see \module{dtp_pddl_parsing_data_problem}). Inherits from
         * \class{Formula_Data} and \class{Constants_Data}. */
        class Problem_Data;
    }
}

namespace std
{
    /* Output streaming for domain descriptions (see
     * \module{dtp_pddl_parsing_data_domain__streaming}).*/
    std::ostream& operator<<(std::ostream&, const Planning::Parsing::Domain_Data&);
    
    /* Output streaming for problem descriptions (see
     * \module{dtp_pddl_parsing_data_problem__streaming}).*/
    std::ostream& operator<<(std::ostream&, const Planning::Parsing::Problem_Data&);
}

namespace Planning
{
    namespace Parsing
    {
        /* Setup the typedef for the datastructure to be used by the
         * parser when parsing domain definitions. (see
         * \class{Domain_Data} from \module{dtp_pddl_parsing_data_domain}) */ 
        typedef Domain_Data Domain_Stack;
        
        /* Setup the typedef for the datastructure to be used by the
         * parser when parsing domain definitions. (see
         * \class{Problem_Data} from \module{dtp_pddl_parsing_data_problem}) */
        typedef Problem_Data Problem_Stack;
    }
}

namespace Planning
{
    namespace Parsing
    {
        /* A problem is identified by its parsed name, along with the
         * domain it has been associated with.*/
        typedef std::tr1::tuple<Planning::Domain_Name, Planning::Problem_Name> Problem_Identifier;
        
        /* Each domain is given a unique name. Here we provide a
         * candidate datastructure to get the (parse) data associated
         * with a particular domain.*/
        typedef std::map<Planning::Domain_Name
                         , CXX__PTR_ANNOTATION(Domain_Stack) > Domain_Name__To__Domain_Data;
        
        /* Each problem is identified uniquely in terms of its name,
         * and the domain to which it is associated. Here we provide a
         * candidate datastructure to get the (parse) data associated
         * with a particular problem.*/
        typedef std::map<Problem_Identifier
                         , CXX__PTR_ANNOTATION(Problem_Stack) > Domain_And_Problem_Names__To__Problem_Data;
        
        /* Last problem parsed (if nothing has yet been parsed then
         * assume 0 == problem_stack.use_count()).*/
        extern CXX__PTR_ANNOTATION(Problem_Stack) problem_Stack;

        /* Last domain parsed (if nothing has yet been parsed then
         * assume 0 == problem_stack.use_count()).*/
        extern CXX__PTR_ANNOTATION(Domain_Stack) domain_Stack;

        /* Applications collection of parsed planning domains (see
         * \module{dtp_pddl_parsing_data.hh} \class{Domain_Data}).*/
        extern Domain_Name__To__Domain_Data domains;
        
        /* Applications collection of parsed planning problems (see
         * \module{dtp_pddl_parsing_data.hh} \class{Problem_Data}).*/
        extern Domain_And_Problem_Names__To__Problem_Data problems;
    }
}


#endif
