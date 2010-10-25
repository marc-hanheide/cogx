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
#ifndef PLANNING_OBSERVATION_SCHEMA_HH
#define PLANNING_OBSERVATION_SCHEMA_HH

#include "planning_formula.hh"


namespace Planning
{
    class Observation_Header :
        public First_Order_Symbol_Description<enum_types::observation_header
                                              , Observation_Name>
    {PRINTING;};
    
    class Observation_Schema : public type_wrapper<enum_types::observation_schema
                                              , Planning::Observation_Header
                                              , Planning::Formula::Subformula
                                              , Planning::Formula::Subformula
                                              , Planning::Formula::Subformula>
    {
        PRINTING;

            

    public:

            

        Observation_Header get__header() const;
        Observation_Name get__name() const;
        Planning::Formula::Subformula get__effect() const;
        Planning::Formula::Subformula get__precondition() const;
        Planning::Formula::Subformula get__execution_precondition() const;
        
        /* WARNING :: Make sure that the altered precondition is
         * logically equivalent to the original precondition,
         * otherwise the program behaviour will unlikely be what you
         * intended. */
        void alter__precondition(Planning::Formula::Subformula);
        void alter__execution_precondition(Planning::Formula::Subformula);
    };

    typedef std::set<Observation_Schema> Observation_Schemas;
}



#endif
