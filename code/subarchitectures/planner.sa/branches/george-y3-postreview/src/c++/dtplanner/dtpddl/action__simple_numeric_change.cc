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

#include "action__simple_numeric_change.hh"

#include "planning_state.hh"


namespace Planning
{
    template<>
    const Formula::Action_Proposition&
    Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    get__identifier() const
    {
        return std::tr1::get<0>(contents());
    }

//     //FIX here :: No idea at the moment why the compiler is complaining when this is uncommented.
    template<>
    ID_TYPE 
    Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    get__change_index() const
    {
        return std::tr1::get<1>(Parent::contents());
    }

    template<>
    int 
    Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    get__modification_value() const
    {
        return std::tr1::get<2>(contents());
    }

    template<>
    int 
    Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    get__modification_type() const
    {
        return std::tr1::get<3>(contents());
    }


    template<>
    const Formula::Action_Proposition&
    Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    get__identifier() const
    {
        return std::tr1::get<0>(contents());
    }

    template<>
    ID_TYPE 
    Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    get__change_index() const
    {
        return std::tr1::get<1>(contents());
    }

    template<>
    double
    Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    get__modification_value() const
    {
        return std::tr1::get<2>(contents());
    }

    template<>
    int 
    Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    get__modification_type() const
    {
        return std::tr1::get<3>(contents());
    }

    template<>
    State* Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    operator()(State* in) const
    {
        auto index = get__change_index();
        
        switch(get__modification_type()){
            case enum_types::increase:
            {
                in->set__float(index
                               , in->get__float(index)
                               + get__modification_value());
            }
            break;
            case enum_types::decrease:
            {
                in->set__float(index
                              , in->get__float(index)
                              - get__modification_value());
            }
            break;
            case enum_types::assign:
            {
                in->set__float(index
                              , get__modification_value());
            }
            break;
            default:
                UNRECOVERABLE_ERROR("Unknown modification executed.");
                break;
        }

        return in;
    }

    template<>
    State* Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    operator()(State* in) const
    {
        
        INTERACTIVE_VERBOSER(true, 9096, "Executing simple numeric effect :: "<<*this<<std::endl);
        
        auto index = get__change_index();
        
        switch(get__modification_type()){
            case enum_types::increase:
            {
                INTERACTIVE_VERBOSER(true, 9096, "Increasing by :: "<<get__modification_value()<<std::endl);
        
                in->set__int(index
                            , in->get__int(index)
                            + get__modification_value());
            }
            break;
            case enum_types::decrease:
            {
                INTERACTIVE_VERBOSER(true, 9096, "Decreasing by :: "<<get__modification_value()<<std::endl);
                
                in->set__int(index
                            , in->get__int(index)
                            - get__modification_value());
            }
            break;
            case enum_types::assign:
            {
                INTERACTIVE_VERBOSER(true, 9096, "Assign by :: "<<get__modification_value()<<std::endl);
                
                in->set__int(index
                            , get__modification_value());
            }
            break;
            default:
                UNRECOVERABLE_ERROR("Unknown modification executed.");
                break;
        }

        return in;
    }

    template<>
    void Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    // report__newly_satisfied
    forced_wake(State& state) const
    {
        this->operator()(&state);
    }
//     template<>
//     void Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
//     report__newly_unsatisfied(State&) const
//     {
//         /*NOTHING TO BE DONE. */
//     }
    
    template<>
    void Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    forced_wake(State& state) const
    {
        this->operator()(&state);
    }
    
//     template<>
//     void Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
//     report__newly_unsatisfied(State&) const
//     {
//         /*NOTHING TO BE DONE. */
//     }
    
    template<>
    std::ostream& Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    operator<<(std::ostream&o) const
    {
        o<<"NUMERIC :: "<<get__identifier()<<std::endl;

        switch(get__modification_type()){
            case enum_types::increase:
            {
                o<<"increase :: ";
            }
            break;
            case enum_types::decrease:
            {
                o<<"decrease :: ";
            }
            break;
            case enum_types::assign:
            {
                o<<"assign :: ";
            }
            break;
            default:
                UNRECOVERABLE_ERROR("Unknown modification :: "<<get__modification_type());
                break;
        }

        if(Formula::State_Ground_Function::ith_exists(get__runtime_Thread(), get__change_index())){
            auto function = Formula::State_Ground_Function
                ::make_ith<Formula::State_Ground_Function>(get__runtime_Thread()
                                                           , get__change_index());
            o<<function<<":: ";
        } else {
            o<<get__change_index()<<":: ";
        }

        return o<<get__modification_value()<<std::endl;
    }
    
    template<>
    std::ostream& Simple_Numeric_Transformation<double, enum_types::simple_double_transformation>::
    operator<<(std::ostream&o) const
    {
        o<<"NUMERIC :: "<<get__identifier()<<std::endl;

        switch(get__modification_type()){
            case enum_types::increase:
            {
                o<<"increase :: ";
            }
            break;
            case enum_types::decrease:
            {
                o<<"decrease :: ";
            }
            break;
            case enum_types::assign:
            {
                o<<"assign :: ";
            }
            break;
            default:
                UNRECOVERABLE_ERROR("Unknown modification :: "<<get__modification_type());
                break;
        }

        if(Formula::State_Ground_Function::ith_exists(get__runtime_Thread(), get__change_index())){
            auto function = Formula::State_Ground_Function
                ::make_ith<Formula::State_Ground_Function>(get__runtime_Thread()
                                                           , get__change_index());
            o<<function<<":: ";
        } else {
            o<<get__change_index()<<":: ";
        }

        return o<<get__modification_value()<<std::endl;
    }
    
}

void Planning::Simple_Int_Transformation::set__statically_false(State& state) const
{
    INTERACTIVE_VERBOSER(true, 17000, "Reporting statically false element.");
    
    
    call_count++;
    if(call_count > 1){
        return;
        assert(state.get__obtainable_rewards_count());
        UNRECOVERABLE_ERROR("Bad call count :: "<<call_count);
    }
    
    
    /*If the reward is positive.*/
    if( ((enum_types::increase == get__modification_type())
         && (get__modification_value() > 0)) ||
        ((enum_types::assign == get__modification_type())
         && (get__modification_value() > 0)) ||
        ((enum_types::decrease == get__modification_type())
         && (get__modification_value() < 0))){

        /*Making a positive reward change at a state.*/
        state.decrement__obtainable_positive_rewards_count();
    }
    
    state.decrement__obtainable_rewards_count();
    state.decrement__obtainable_rewards_value(get__modification_value());
    INTERACTIVE_VERBOSER(true, 17000, "State is now :: "<<state);
}

namespace std
{
    std::ostream& operator<<(std::ostream&o
                             , const Planning::Simple_Int_Transformation&in)
    {
        return o<<in;
    }
    
    std::ostream& operator<<(std::ostream&o
                             , const Planning::Simple_Double_Transformation&in)
    {
        return o<<in;
    }
    
}

#include "action_basics.hh"
#include "action__state_transformation.hh"

void Planning::count_reward_assignments_at_state
(Planning::State_Formula::Satisfaction_Listener__Pointer satisfaction_Listener
 , Planning::State& state
 , uint reward_index)
{
    assert(satisfaction_Listener.test_cast<basic_type>());
    
    if(satisfaction_Listener.test_cast<Planning::Simple_Int_Transformation>()){
        
        INTERACTIVE_VERBOSER(true, 10908, "Got integer transformation!");
        auto input = satisfaction_Listener.cxx_get<Planning::Simple_Int_Transformation>();
        INTERACTIVE_VERBOSER(true, 10908, "Value is :: "<<input->get__modification_value());
        if(input->get__change_index() == reward_index){
            
            /*If the reward is positive.*/
            if( ((enum_types::increase == input->get__modification_type())
                 && (input->get__modification_value() > 0)) ||
                ((enum_types::assign == input->get__modification_type())
                 && (input->get__modification_value() > 0)) ||
                ((enum_types::decrease == input->get__modification_type())
                 && (input->get__modification_value() < 0))){

                /*Making a positive reward change at a state.*/
                state.increment__obtainable_positive_rewards_count();
            }
            
            
            state.increment__obtainable_rewards_count();
            state.increment__obtainable_rewards_value(input->get__modification_value());
        }
    }
//     if(satisfaction_Listener.test_cast<Planning::Simple_Double_Transformation>()){
        
//         INTERACTIVE_VERBOSER(true, 10908, "Got double transformation!");
//         auto input = satisfaction_Listener.cxx_get<Planning::Simple_Int_Transformation>();
//         if(input->get__change_index() == reward_index){
//             state.increment__obtainable_rewards_count();
//         }
//     }

    
    VERBOSER(10908, "Non-Integer transformation :: "<<satisfaction_Listener);

    {
        auto& traversable__listeners = satisfaction_Listener
            .cxx_get<State_Formula::Satisfaction_Listener>()
            ->get__traversable__listeners();
        
        for(auto listener = traversable__listeners.begin()
                ; listener != traversable__listeners.end()
                ; listener++){
            Planning::count_reward_assignments_at_state(*listener, state, reward_index);
        }
    }
    
    {
        auto& traversable__listeners = satisfaction_Listener
            .cxx_get<State_Formula::Satisfaction_Listener>()
            ->get__traversable__sleepers();
        
        for(auto listener = traversable__listeners.begin()
                ; listener != traversable__listeners.end()
                ; listener++){
            Planning::count_reward_assignments_at_state(*listener, state, reward_index);
        }
    }
    
}
