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

Are_Doubles_Close State_Transformation::are_Doubles_Close = Are_Doubles_Close(1e-9);

State& State_Transformation::operator()(State& __successor)
{
    State* new_state;
    State& _successor = (get__compulsory() && are_Doubles_Close(get__probability(), 1.0))
        ?*(new_state = new State(__successor))
        :__successor;

    auto effects = get__effects();
    for(auto effect = effects.begin()
            ; effect != effects.end()
            ; effect++){
        if(!(*effect)->is_satisfied(_successor)){
            (*effect)->flip_satisfaction(_successor);
        }
    }

    return _successor;
}

const Formula::Action_Proposition& State_Transformation
::get__get_identifier() const
{
    return std::tr1::get<2>(contents());
}


const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& State_Transformation
::get__precondition() const
{
    return std::tr1::get<3>(contents());
}


const State_Formula::List__Literals& State_Transformation
::get__effects() const
{
    return std::tr1::get<4>(contents());
}

bool State_Transformation
::get__compulsory() const
{
    return std::tr1::get<5>(contents());
}

double State_Transformation
::get__probability() const
{
    return std::tr1::get<6>(contents());
}






void State_Transformation
::set__satisfied(State& state)
{
    state.get__transformation__satisfaction_status().satisfy(get__id());
}

void State_Transformation
::set__unsatisfied(State& state)
{
    state.get__transformation__satisfaction_status().unsatisfy(get__id());
}

void State_Transformation
::flip_satisfaction(State& state)
{
    state.get__transformation__satisfaction_status().flip_satisfaction(get__id());
}

bool State_Transformation
::is_satisfied(const State& state) const
{
    return state.get__transformation__satisfaction_status().satisfied(get__id());
}

void State_Transformation
::increment__level_of_satisfaction(State& state)
{
    state.get__transformation__count_status().increment_satisfaction(get__id());
}

void State_Transformation
::decrement__level_of_satisfaction(State& state)
{
    state.get__transformation__count_status().decrement_satisfaction(get__id());
}

void State_Transformation
::set__level_of_satisfaction(uint level, State& state)
{
    state.get__transformation__count_status().set_satisfaction(get__id(), level);
}

uint State_Transformation
::get__level_of_satisfaction(State& state) const
{
    return state.get__transformation__count_status().get_satisfaction_level(get__id());
}




uint State_Transformation::get__number_of_satisfied_conditions(State& state) const
{
    return get__level_of_satisfaction(state);
}


void State_Transformation
::report__newly_satisfied(State& state)
{
    increment__level_of_satisfaction(state);

    uint satisfaction_requirement = (get__compulsory())?2:1;

    if(satisfaction_requirement == get__number_of_satisfied_conditions(state)){
        set__satisfied(state);
        
        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<_Satisfaction_Listener>()
                ->report__newly_satisfied(state);
        }


        
        if(get__compulsory()){
            register double probability = get__probability();
            if(are_Doubles_Close(probability, 1.0)){
                state.add__compulsory_transformation(this);
            } else {
                state.add__compulsory_generative_transformation(this);
            }
        } else {
            state.add__optional_transformation(this);
        }
    }
}

void State_Transformation
::report__newly_unsatisfied(State& state)
{
    decrement__level_of_satisfaction(state);

    if(is_satisfied(state)){
        set__unsatisfied(state);
        
        auto parents = get__traversable_parents();
        for(auto parent = parents.begin()
                ; parent != parents.end()
                ; parent++){
            (*parent).cxx_get<_Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
        
        if(!get__compulsory()){    
            state.remove__optional_transformation(this);
        }
    }
    
}

