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

#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"

#include "planning_state.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"


using namespace Planning;


Are_Doubles_Close State_Transformation::are_Doubles_Close = Are_Doubles_Close(1e-9);


State*
State_Transformation::operator()(State* in) const
{
    
    if(!are_Doubles_Close(1.0, get__probability(*in))){
        INTERACTIVE_VERBOSER(true, 9071, "Applying probabilistic effect :: "<<*this<<std::endl);
        in->set__probability_during_expansion(in->get__probability_during_expansion()
                                             * get__probability(*in));
    }
    
//     /* If the action is not compulsory (i.e., is an agent executable
//      * action), then wake up all the derivative actions.*/
//     if(!get__compulsory()){
//         wake_sleepers(*in);
//     }

    
    const State_Formula::List__Literals& effects = get__effects();
    for(auto effect = effects.begin()
            ; effect != effects.end()
            ; effect++){
        /* If the effect is not satisfied in the state to which this
         * transformation is being applied.*/
        if(!(*effect)->is_satisfied(*in)){
            INTERACTIVE_VERBOSER(true, 9090, *this<<"Flipping :: "<<**effect<<std::endl);
            /* The the effect must be applied, because the parent
             * transformation was applied.*/
            (*effect)->flip(*in);//_satisfaction(*in);
        }
    }
    
    return in;
}

const Formula::Action_Proposition& State_Transformation
::get__identifier() const
{
    return std::tr1::get<0>(contents());
}


const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& State_Transformation
::get__precondition() const
{
    return std::tr1::get<1>(contents());
}


const State_Formula::List__Literals& State_Transformation
::get__effects() const
{
    return std::tr1::get<2>(contents());
}

bool State_Transformation
::get__compulsory() const
{
    return std::tr1::get<3>(contents());
}

bool State_Transformation
::get__lookup_probability() const
{
    return std::tr1::get<4>(contents());
}

double State_Transformation
::get__probability() const
{
    return std::tr1::get<5>(contents());
}

double State_Transformation
::get__probability(const State& state) const
{
    if(!get__lookup_probability()){
        return get__probability();
    } else {
        return state.get__float(std::tr1::get<6>(contents()));
    }
}


void State_Transformation
::set__satisfied(State& state) const
{
    assert(state.get__transformation__satisfaction_status().valid_index(get__id()));
    state.get__transformation__satisfaction_status().satisfy(get__id());
}

void State_Transformation
::set__unsatisfied(State& state) const
{
    assert(state.get__transformation__satisfaction_status().valid_index(get__id()));
    state.get__transformation__satisfaction_status().unsatisfy(get__id());
}

void State_Transformation
::flip(State& state) const
{
    assert(state.get__transformation__satisfaction_status().valid_index(get__id()));
    WARNING("Something just changed the executability status value associated with a"<<std::endl
            <<"transformation by an unusual mechanism -- "<<std::endl
            <<"i.e., the satisfaction for a CNF-based precondition was not reported changed.");
    state.get__transformation__satisfaction_status().flip_satisfaction(get__id());
}

bool State_Transformation
::is_satisfied(const State& state) const
{
    assert(state.get__transformation__satisfaction_status().valid_index(get__id()));
    return state.get__transformation__satisfaction_status().satisfied(get__id());
}

void State_Transformation
::increment__level_of_satisfaction(State& state) const
{
    assert(state.get__transformation__count_status().valid_index(get__id()));
    state.get__transformation__count_status().increment_satisfaction(get__id());
}

void State_Transformation
::decrement__level_of_satisfaction(State& state) const
{
    assert(state.get__transformation__count_status().valid_index(get__id()));
    state.get__transformation__count_status().decrement_satisfaction(get__id());
}

void State_Transformation
::set__level_of_satisfaction(uint level, State& state) const
{
    assert(state.get__transformation__count_status().valid_index(get__id()));
    state.get__transformation__count_status().set_satisfaction(get__id(), level);
}

uint State_Transformation
::get__level_of_satisfaction(State& state) const
{
    assert(state.get__transformation__count_status().valid_index(get__id()));
    return state.get__transformation__count_status().get_satisfaction_level(get__id());
}



            
void State_Transformation::forced_wake(State& state) const
{
    assert(get__compulsory());

    assert(get__traversable__listeners().size() == 0);
    
    if(!(get__precondition()->get__disjunctive_clauses().size())
       || is_satisfied(state)){

        
        if(get__lookup_probability()){
            WARNING("Unimplemented support for probability lookup.");
        }
        
        double probability = get__probability(state);
            
        if(are_Doubles_Close(probability, 1.0)){
            state.push__compulsory_transformation(this);
        } else {
            state.push__compulsory_generative_transformation(this);
        }
            
        wake_sleepers(state);
    }
}


uint State_Transformation::get__number_of_satisfied_conditions(State& state) const
{
    return get__level_of_satisfaction(state);
}


void State_Transformation
::report__newly_satisfied(State& state) const
{
    assert(get__level_of_satisfaction(state) == 0);
    
    increment__level_of_satisfaction(state);

    assert(get__precondition()->get__disjunctive_clauses().size() != 0);
    assert(get__level_of_satisfaction(state) == 1);
    
    set__satisfied(state);

    if(!get__compulsory()){
        state.add__optional_transformation(this);
    }
}

void State_Transformation
::report__newly_unsatisfied(State& state) const
{
    assert(get__level_of_satisfaction(state) == 1);
    
    decrement__level_of_satisfaction(state);
    assert(get__level_of_satisfaction(state) == 0);
    assert(get__precondition()->get__disjunctive_clauses().size() != 0);

    assert(is_satisfied(state));
    
    set__unsatisfied(state);
    
    if(!get__compulsory()){    
        state.remove__optional_transformation(this);
    }
}

std::ostream& State_Transformation::operator<<(std::ostream&o) const
{
    o<<"ACTION IDENTIFIER :: "<<get__identifier()<<std::endl;

    
    o<<"PRECONDITION :: "<<get__precondition()<<std::endl;
    
    if(!get__lookup_probability()){
        o<<"Prob :: "<<get__probability()<<std::endl;
    } else {
        o<<"Prob :: LOOKUP"<<std::endl;
    }
    
    o<<"ADD :: ";
    /*add effects*/
    for(auto effect = get__effects().begin()
            ; effect != get__effects().end()
            ; effect++){
        if(!(*effect)->get__sign()){
            o<<*effect<<", ";
        }
    }
    o<<std::endl;

    
    /*delete effects*/
    
    o<<"DELETE :: ";
    /*add effects*/
    for(auto effect = get__effects().begin()
            ; effect != get__effects().end()
            ; effect++){
        if((*effect)->get__sign()){
            o<<*effect<<", ";
        }
    }
    o<<std::endl;
    o<<"{"<<get__traversable__listeners().size()<<"-LISTENERS :: ";
    for(auto listener = get__traversable__listeners().begin()
            ; listener != get__traversable__listeners().end()
            ; listener++){
        if(listener->test_cast<basic_type>()){
            listener->cxx_get<basic_type>()->operator<<(o);
        }
        
    }
    o<<"}"<<std::endl;
    
    o<<std::endl;
    o<<"{"<<get__traversable__sleepers().size()<<"-SLEEPERS :: ";
    for(auto listener = get__traversable__sleepers().begin()
            ; listener != get__traversable__sleepers().end()
            ; listener++){
        if(listener->test_cast<basic_type>()){
            listener->cxx_get<basic_type>()->operator<<(o);
        }
        
    }
    o<<"}"<<std::endl;
    
    return o;
}

namespace std
{
    std::ostream& operator<<(std::ostream&o
                             , const Planning::State_Transformation&in)
    {
        return in.operator<<(o);
    }
    
}


