
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


#include "observation.hh"

#include "planning_state.hh"
#include "planning_observation.hh"

#include "action__literal.hh"
#include "action__disjunctive_clause.hh"
#include "action__conjunctive_normal_form_formula.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"



using namespace Planning;


Are_Doubles_Close Observation::are_Doubles_Close = Are_Doubles_Close(1e-9);

void Observation::increment__level_of_satisfaction(State& state) const
{
    QUERY_UNRECOVERABLE_ERROR(!state.get__observation__count_status().valid_index(get__id())
                              , get__id()<<"Can't read state of :: "<<*this<<std::endl);
    
//     assert(state.get__observation__count_status().valid_index(get__id()));
    state.get__observation__count_status().increment_satisfaction(get__id());
}

void Observation::decrement__level_of_satisfaction(State& state) const
{
    assert(state.get__observation__count_status().valid_index(get__id()));
    state.get__observation__count_status().increment_satisfaction(get__id());
}

void Observation::set__satisfied(State& state) const
{
    assert(state.get__observation__satisfaction_status().valid_index(get__id()));
    state.get__observation__satisfaction_status().satisfy(get__id());
}

void Observation::set__unsatisfied(State& state) const
{
    assert(state.get__observation__satisfaction_status().valid_index(get__id()));
    state.get__observation__satisfaction_status().satisfy(get__id());
}

bool Observation
::is_top_level() const
{
    return std::tr1::get<4>(contents());
    
}

const Formula::Observational_Proposition& Observation
::get__identifier() const
{
    return std::tr1::get<0>(contents());
}

const State_Formula::Conjunctive_Normal_Form_Formula__Pointer& Observation
::get__precondition() const
{
    return std::tr1::get<1>(contents());
}

const Action_Conjunctive_Normal_Form_Formula__Pointer& Observation
::get__execution_precondition() const
{
    return std::tr1::get<2>(contents());
}

const Formula::List__Perceptual_Propositions& Observation
::get__effects() const
{
    return std::tr1::get<3>(contents());
}

bool Observation
::get__lookup_probability() const
{
    return std::tr1::get<5>(contents());
}

double Observation
::get__probability() const
{
    return std::tr1::get<6>(contents());
}

double Observation
::get__probability(const State& state) const
{
    if(!get__lookup_probability()){
        return get__probability();
    } else {
        return state.get__float(std::tr1::get<6>(contents()));
    }
}

Planning::Observational_State* Observation::operator()
    (Planning::Observational_State* observational_State, Planning::State* state) const
{
    for(auto effect = get__effects().begin()
            ; effect != get__effects().end()
            ; effect++){
        assert((*effect)->get__id() <= observational_State->get__number_of_atoms());
        observational_State->flip_on((*effect)->get__id());
    }

    if(!are_Doubles_Close(1.0, get__probability(*state))){
       observational_State
           ->set__probability_during_expansion(state->get__probability_during_expansion()
                                               * get__probability(*state));
    }
    
    auto listeners = Satisfaction_Listener::get__traversable__listeners();
    for(auto listener = listeners.begin()
            ; listener != listeners.end()
            ; listener++){
        (*listener).cxx_get<Satisfaction_Listener>()
            ->report__newly_satisfied(*state);
    }
    
    /*All observations are compulsory.*/
    report__newly_unsatisfied(*state);
    
    return observational_State;
}
        
void Observation::report__newly_satisfied(State& state) const
{
    increment__level_of_satisfaction(state);

    if(is_top_level()){

        assert(!( (get__precondition()->get__disjunctive_clauses().size() == 0) &&
                  (get__execution_precondition()->get__disjunctive_clauses().size() == 0) ));
        
        if((get__precondition()->get__disjunctive_clauses().size() == 0) ||
           get__precondition()->is_satisfied(state)){
            
            if( (get__execution_precondition()->get__disjunctive_clauses().size() == 0) ||
                get__execution_precondition()->is_satisfied(state)){
                
                set__satisfied(state);
                state.push__observation
                    (this);
            }
        }
    } else {
        if( (get__precondition()->get__disjunctive_clauses().size() == 0) &&
            (get__execution_precondition()->get__disjunctive_clauses().size() == 0)){
            
            assert(get__level_of_satisfaction(state) == 1);
            set__satisfied(state);
            state.push__observation
                (this);
            
        } else if ( (get__precondition()->get__disjunctive_clauses().size() == 0) ||
                    (get__execution_precondition()->get__disjunctive_clauses().size() == 0) )
        {
            QUERY_UNRECOVERABLE_ERROR(get__level_of_satisfaction(state) > 2
                                      , "Expecting a level of satisfaction not grater than 2, but got :: "
                                      <<get__level_of_satisfaction(state)<<std::endl);
            
            if(get__level_of_satisfaction(state) == 2){
                set__satisfied(state);
                state.push__observation
                    (this);
            }
        } else if (get__level_of_satisfaction(state) == 3) {
            set__satisfied(state);
            state.push__observation
                (this);
        }
    }
    
    
    
//     /*If the action has no precondition.*/
//     if((get__precondition()->get__disjunctive_clauses().size() == 0) ||
//        get__precondition()->is_satisfied(state)){

//         if( (get__execution_precondition()->get__disjunctive_clauses().size() == 0) ||
//             get__execution_precondition()->is_satisfied(state)){

//             state.push__observation
//                 (this);
            
//             auto listeners = get__traversable__listeners();
//             for(auto listener = listeners.begin()
//                     ; listener != listeners.end()
//                     ; listener++){
//                 (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
//             }
//         }        
//     } 
}

bool Observation::is_satisfied(const State& state) const
{
    return state.get__observation__satisfaction_status().satisfied(get__id());
}

void Observation::report__newly_unsatisfied(State& state) const
{
    decrement__level_of_satisfaction(state);

    if(is_satisfied(state)){
        set__unsatisfied(state);
    }   
}


uint Observation::get__level_of_satisfaction(State& state) const
{
    return state.get__observation__count_status().get_satisfaction_level(get__id());
    
//     uint result = 0;
//     if((get__precondition()->get__disjunctive_clauses().size() == 0) ||
//        get__precondition()->is_satisfied(state)){
//         result++;
//     }
    
//     if( (get__execution_precondition()->get__disjunctive_clauses().size() == 0) ||
//         get__execution_precondition()->is_satisfied(state)){
//         result++;
//     }
        
//     return result;
}


uint Observation::get__number_of_satisfied_conditions(State& state) const
{
    return get__level_of_satisfaction(state);
}


std::ostream& Observation::operator<<(std::ostream&o) const
{
    o<<get__identifier()<<std::endl;

    
    o<<"PRECONDITION :: "<<get__precondition()<<std::endl;
    
    o<<"EXECUTION PRECONDITION :: "<<get__execution_precondition()<<std::endl;
    
    if(!get__lookup_probability()){
        o<<"Prob :: "<<get__probability()<<std::endl;
    } else {
        o<<"Prob :: LOOKUP"<<std::endl;
    }
    
    o<<"ADD PERCEPTIVE PROPOSITIONS :: ";
    /*add effects*/
    for(auto effect = get__effects().begin()
            ; effect != get__effects().end()
            ; effect++){
        o<<*effect<<", ";
    }
    o<<std::endl;

    
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
    
    return o;
}


namespace std
{
    std::ostream& operator<<(std::ostream& o
                             , const Planning::Observation& in)
    {
        return in.operator<<(o);
    }
    
}
