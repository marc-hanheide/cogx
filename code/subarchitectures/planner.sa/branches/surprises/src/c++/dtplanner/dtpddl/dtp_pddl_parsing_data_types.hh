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

#ifndef DTP_PDDL_PARSING_DATA_TYPES_HH
#define DTP_PDDL_PARSING_DATA_TYPES_HH


#include "global.hh"
#include "planning_symbols.hh"


namespace Planning
{
    namespace Parsing
    {
        class Types_Data
        {
        public:
            /* - \member{symbol_theory} is initially NULL.*/
            Types_Data();
            
            ~Types_Data();

            /* Function and predicate names are associated with a
             * domain theory. Therefore, when we talk about them in
             * some descendants, when we create them they must be
             * linked to that theory otherwise they will seem like
             * distinct symbols. The \argument{void*} points to the
             * theory which initially defines those symbols.*/
            void report__symbol_name_reference(void*);
            
            /* In PDDL we have a specification element
             * (:types typename_1 typename_2 ... - (either typename_N typename N+1 ... ) )
             */
            void add__type(const std::string& str);//{types.push_back(str);};
            void add__type_of_type(const std::string& str);
            /* Adds the entries in \member{types} and
             * \member{types_of_types} to
             * \member{types_description}. The former are strings
             * occurring before the '-' is a PDDL type
             * specification. The latter occur after that in an
             * "(either T1 T2 ...)" clause.*/
            void add__types();
            
            /*Obtains the type of the argument variable from the indicated structure.*/
            Planning::Types find__stack_of__Typed_Arguments(const Planning::Variable&) const;
            
            virtual Planning::Types find__type_of_variable(const Planning::Variable&) const;
            
//             void stack__typed_Arguments();
            
            /* The contents of \member{types} is treated as domain
             * types from hereon -- unless a further commitment is
             * made. */
            void commit__types();

            /* see \member{types_description}*/
            const std::map<Type, Types >& get__types_description() const;
            
        protected:
            /*(see \member{stack__typed_Arguments} and
             * \member{complete__\XXX\_formula})*/
            std::stack<Typed_Arguments> stack_of__Typed_Arguments;
            
            /* Domain of \class{map} is a set of types that occur in
             * the domain description. Each domain element is mapped
             * into the set of types it is a subset of.*/
            std::map<Type, Types > types_description;
            
            /* Either:
             *
             * (1) A unit list containing the single type on the RHS
             * of an '-' elements in a :types distribution element
             *
             * (2) The list of types that occur in an "(either type_1
             * type_2 ...)" PDDL :types description.*/
            Planning::Types types_of_types;
            
            /* List of types that have been parsed and not used yet
             * (either in a descrption of the domain types, or
             * otherwise as part of an argument list -- see
             * \member{arguments_Description})*/
            Planning::Types types;
            
            /*(see \member{report__symbol_name_reference})*/
            void* symbol_theory;
        };
    }
}


#endif
