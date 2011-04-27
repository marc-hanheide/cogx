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
#include "planning_observation_schema.hh"

using namespace Planning;
using namespace Planning::Formula;

Observation_Header Observation_Schema::get__header() const {return std::tr1::get<0>(this->contents());}
Observation_Name Observation_Schema::get__name() const {return std::tr1::get<0>(get__header().contents());}
Subformula Observation_Schema::get__precondition() const {return std::tr1::get<1>(this->contents());}
Subformula Observation_Schema::get__effect() const {return std::tr1::get<2>(this->contents());}
Subformula Observation_Schema::get__execution_precondition() const {return std::tr1::get<3>(this->contents());}



void Observation_Schema::alter__precondition(Planning::Formula::Subformula subformula)
{
    std::tr1::get<1>(this->_contents()) = subformula;
}

void Observation_Schema::alter__execution_precondition(Planning::Formula::Subformula subformula)
{
    std::tr1::get<3>(this->_contents()) = subformula;
}

std::ostream& Planning::Observation_Header::operator<<(std::ostream& o) const
{
    auto name = std::tr1::get<0>(contents());
    
    
    o<<name<<" "<<std::endl;
    o<<":parameters ";

    auto arguments = std::tr1::get<1>(contents());;

    o<<"( "<<arguments<<" ) "<<std::endl;
    
    return o;
}


std::ostream& Observation_Schema::operator<<(ostream&o)const 
{
    o<<"(:observe ";

    o<<get__header()<<std::endl;
    
    o<<":execution ";

    o<<get__execution_precondition()<<std::endl;
    
    o<<":precondition ";
    
    o<<get__precondition()<<std::endl;
    
    o<<":effect ";
    
    o<<get__effect()<<std::endl;
    
    return o<<")"<<std::endl;
}
