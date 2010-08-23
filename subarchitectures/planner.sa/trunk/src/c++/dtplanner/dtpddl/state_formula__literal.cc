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

#include "state_formula__literal.hh"


#include "planning_state.hh"
// #include "planning_types_enum.hh"
#include "planning_formula.hh"


using namespace Planning;
using namespace Planning::State_Formula;
      

void Literal::set__satisfied(State& state) const
{
    assert(state.get__number_of_atoms() >= get__variable());
    
    if(is_satisfied(state)){
        return;
    }
    
    if(get__sign() && state.is_true(get__variable())){
        state.flip(get__variable());
    } else if (!get__sign() && !state.is_true(get__variable())) {
        state.flip(get__variable());
    }
    
    report__newly_satisfied(state);
}

void Literal::set__unsatisfied(State& state) const
{
    assert(state.get__number_of_atoms() >= get__variable());
    
    if(!is_satisfied(state)){
        return;
    }
    
    if(get__sign() && !state.is_true(get__variable())){
        state.flip(get__variable());
    } else if (!get__sign() && state.is_true(get__variable())) {
        state.flip(get__variable());
    }

    report__newly_unsatisfied(state);
}

            
bool Literal::is_satisfied(const State& state) const
{
    assert(state.get__number_of_atoms() >= get__variable());
    
    if(get__sign() && !state.is_true(get__variable())){
        return true;
    } else if (!get__sign() && state.is_true(get__variable())) {
       return true;
    }

    return false;
}

void Literal::report__newly_satisfied(State& state) const
{
    satisfy_listeners(state);
//     auto listeners = get__traversable__listeners();
//     for(auto listener = listeners.begin()
//             ; listener != listeners.end()
//             ; listener++){
//         INTERACTIVE_VERBOSER(true, 7002, "Just SATISFIED literal  :: "<<*this<<std::endl
//                              <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
//         (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
//     }
}

void Literal::report__newly_unsatisfied(State& state) const
{
    unsatisfy_listeners(state);
//     auto listeners = get__traversable__listeners();
//     for(auto listener = listeners.begin()
//             ; listener != listeners.end()
//             ; listener++){
//         INTERACTIVE_VERBOSER(true, 7002, "Just UNSATISFIED literal  :: "<<*this<<std::endl
//                              <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
//         (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
//     }
}


uint Literal::get__variable() const
{
    return std::tr1::get<0>(contents());
}

bool Literal::get__sign() const
{
    return std::tr1::get<1>(contents());
}

void Literal::flip(State& state) const
{
    assert(state.get__number_of_atoms() >= get__variable());
    
    INTERACTIVE_VERBOSER(true, 7002, "Flipping satisfaction of literal  :: "<<*this);
    
    if(is_satisfied(state)){
        if(has_complement){
            complement->set__satisfied(state);
            
            report__newly_unsatisfied(state);
        } else {
            set__unsatisfied(state);
        }
        
        assert(!is_satisfied(state));
        if(has_complement){
            assert(complement->is_satisfied(state));
        }
    } else {
        assert(!is_satisfied(state));
        
        if(has_complement){
            complement->set__unsatisfied(state);

            
            report__newly_satisfied(state);

        } else {
            set__satisfied(state);
        }
        
        assert(is_satisfied(state));
        if(has_complement){
            assert(!complement->is_satisfied(state));
        }
    }
}

void Literal::
configure__complement(const Literal__Pointer& _this,
                      const Literals& literals)
{
    assert(_this.get() == this);
    
    NEW_referenced_WRAPPED_deref_POINTER
        (get__runtime_Thread(),
         State_Formula::Literal,
         __complement,
         get__variable(),
         !get__sign());
    
    auto _complement = CXX__deref__shared_ptr<State_Formula::Literal>(__complement);

    auto _complement_iterator = literals.find(_complement);
    if(_complement_iterator != literals.end()){
        this->set__complement(*_complement_iterator);
        (*_complement_iterator)->set__complement(_this);
    } else {
        has_complement = false;
    }
}

void Literal::set__complement(const Literal__Pointer& _complement)
{
    assert(_complement.get() != this);

    has_complement = true;
    complement = _complement;
}

std::ostream& Literal::operator<<(std::ostream&o) const
{

    if(Formula::State_Proposition::ith_exists(get__runtime_Thread(), get__variable())){
        auto proposition = Formula::State_Proposition
            ::make_ith<Formula::State_Proposition>(get__runtime_Thread()
                                          , get__variable());
        o<<((get__sign())?"¬":"")<<proposition<<"[|- :l:"<<get__id()<<":  :v:"<<get__variable()<<": -|]";
    } else {
        o<<((get__sign())?"¬":"")<<get__variable()<<"[|- :l:"<<get__id()<<": -|]";
    }
    
    return o;
}
    

namespace std
{
    std::ostream& operator<<(std::ostream&o, const Planning::State_Formula::Literal&in)
    {
        return in.operator<<(o);
    }
    
}
