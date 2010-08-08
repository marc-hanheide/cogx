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



uint State::count__compulsory_generative_transformations() const
{
    return applicable_compulsory_generative_transformations.size();
}

uint State::count__compulsory_transformations() const
{
    return applicable_compulsory_transformations.size();
}


State_Transformation* State::pop__compulsory_generative_transformation()
{
    auto result = applicable_compulsory_generative_transformations.top();
    applicable_compulsory_generative_transformations.pop();
    return result;
}

State_Transformation* State::pop__compulsory_transformation()
{
    auto result = applicable_compulsory_transformations.top();
    applicable_compulsory_transformations.pop();
    return result;
}


uint State::count__probabilistic_transformations() const
{
   return  probabilistic_transformations.size();
}

Probabilistic_State_Transformation* State::pop__probabilistic_transformation()
{
    auto result = probabilistic_transformations.top();
    probabilistic_transformations.pop();
    return result;
}

void State::push__probabilistic_transformation
(Probabilistic_State_Transformation* probabilistic_State_Transformation)
{
    probabilistic_transformations.push(probabilistic_State_Transformation);
}


void State::push__compulsory_generative_transformation(State_Transformation*state_Transformation)
{
    applicable_compulsory_generative_transformations.push(state_Transformation);
}

void State::add__optional_transformation(State_Transformation*state_Transformation)
{
    
    applicable_optional_transformations.insert(state_Transformation);
}

void State::remove__optional_transformation(State_Transformation*state_Transformation)
{
    applicable_optional_transformations.erase(state_Transformation);
}

void State::push__compulsory_transformation(State_Transformation*state_Transformation)
{
    applicable_compulsory_transformations.push(state_Transformation);
}

std::set<State_Transformation*>& State::get__optional_transformations()
{
    return applicable_optional_transformations;
}

