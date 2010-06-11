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
#ifndef DTP_PDDL_PARSING_DATA_CONSTANTS_HH
#define DTP_PDDL_PARSING_DATA_CONSTANTS_HH

#include "global.hh"
#include "planning_symbols.hh"

#include "dtp_pddl_parsing_data_types.hh"

namespace Planning
{
    namespace Parsing
    {
        class Constants_Data : virtual public Types_Data
        {
        public:
            void commit__constants();
            void convert__type_of_type__TO__type_of_constant();
            void add__constants();
            void add__constant(const std::string& str);
            void add__type_of_constant(const std::string& str);

        protected:
            
            /*Each constant/object symbol has a type. If non is given,
             * this defaults to the type "object".*/
            std::map<Constant, Types > constants_description;
            
            /* Last list of constants parsed.*/
            Planning::Constants constants;
            
            /*PDDL object/constant symbols can have a given type. */
            Planning::Types types_of_constants;
        };
    }
}


#endif
