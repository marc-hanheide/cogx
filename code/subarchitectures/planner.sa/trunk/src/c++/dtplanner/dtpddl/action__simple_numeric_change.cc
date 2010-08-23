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

    template<>
    ID_TYPE 
    Simple_Numeric_Transformation<int, enum_types::simple_int_transformation>::
    get__change_index() const
    {
        return std::tr1::get<1>(contents());
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
        auto index = get__change_index();
        
        switch(get__modification_type()){
            case enum_types::increase:
            {
                in->set__int(index
                            , in->get__int(index)
                            + get__modification_value());
            }
            break;
            case enum_types::decrease:
            {
                in->set__int(index
                            , in->get__int(index)
                            - get__modification_value());
            }
            break;
            case enum_types::assign:
            {
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
