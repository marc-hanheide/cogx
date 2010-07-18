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


#include "basic_action.hh"

#include "planning_state.hh"

using namespace Planning;


State_Transformation::State_Transformation(bool compulsory)
    :compulsory(compulsory)
{
}

        
void State_Transformation::report__newly_satisfied(State& state)
{
    if(compulsory){
        state.add__compulsory_transformation(this); 
    } else {
        state.add__optional_transformation(this);
    }
}

void State_Transformation::report__newly_unsatisfied(State& state)
{
    if(compulsory){
        state.retract__compulsory_transformation(this); 
    } else {
        state.retract__optional_transformation(this);
    }   
}
        
bool State_Transformation::is_compulsory() const
{
    return compulsory;
}

void State_Transformation::set__compulsory(bool in)
{
   compulsory = in; 
}



State& STRIPS_Action::operator()(const State&)
{
    UNRECOVERABLE_ERROR("unimplemented");
}

void STRIPS_Action::add__add(uint)
{
    UNRECOVERABLE_ERROR("unimplemented");
}

void STRIPS_Action::add__delete(uint)
{
    UNRECOVERABLE_ERROR("unimplemented");
}
