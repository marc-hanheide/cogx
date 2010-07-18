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



#include "planning_state.hh"

using namespace Planning;

void State::add__compulsory_transformation(State_Transformation* in)
{
    applicable_compulsory_transformations.insert(in);
}

void State::retract__compulsory_transformation(State_Transformation* in)
{
    applicable_compulsory_transformations.erase(in);
}

void State::add__optional_transformation(State_Transformation* in)
{
    applicable_compulsory_transformations.insert(in);
}

void State::retract__optional_transformation(State_Transformation* in)
{
    applicable_compulsory_transformations.erase(in);
}



const Unsigned_Integer__Satisfaction_Status_Management& State::get__cnfs__count_status() const
{
    return cnfs__count_status;
}

Unsigned_Integer__Satisfaction_Status_Management& State::get__cnfs__count_status()
{
    return cnfs__count_status;
}

const Boolean__Satisfaction_Status_Management& State::get__cnfs__satisfaction_status() const
{
    return cnfs__satisfaction_status;
}

Boolean__Satisfaction_Status_Management& State::get__cnfs__satisfaction_status()
{
    return cnfs__satisfaction_status;
}




const Unsigned_Integer__Satisfaction_Status_Management& State::get__clauses__count_status() const
{
    return clauses__count_status;
}

Unsigned_Integer__Satisfaction_Status_Management& State::get__clauses__count_status()
{
    return clauses__count_status;
}

const Boolean__Satisfaction_Status_Management& State::get__clauses__satisfaction_status() const
{
    return clauses__satisfaction_status;
}

Boolean__Satisfaction_Status_Management& State::get__clauses__satisfaction_status()
{
    return clauses__satisfaction_status;
}


const Boolean__Satisfaction_Status_Management& State::get__literals__satisfaction_status() const
{
    return literals__satisfaction_status;
}

Boolean__Satisfaction_Status_Management& State::get__literals__satisfaction_status()
{
    return literals__satisfaction_status;
}


