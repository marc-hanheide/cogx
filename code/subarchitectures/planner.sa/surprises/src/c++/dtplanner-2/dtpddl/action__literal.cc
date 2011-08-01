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


#include "action__literal.hh"

#include "planning_state.hh"

using namespace Planning;

Action_Literal::Action_Literal()
    :Action_Literal::Parent(true)
{
}

void Action_Literal::
configure__complement(const Action_Literal__Pointer& _this,
                      const Action_Literals& literals)
{
    assert(_this.get() == this);
    
    NEW_referenced_WRAPPED_deref_POINTER
        (get__runtime_Thread(),
         Action_Literal,
         __complement,
         get__action_symbol(),
         !get__sign());
    
    auto _complement = CXX__deref__shared_ptr<Action_Literal>(__complement);

    auto _complement_iterator = literals.find(_complement);
    if(_complement_iterator != literals.end()){
        this->set__complement(*_complement_iterator);
        (*_complement_iterator)->set__complement(_this);
    } else {
        has_complement = false;
    }
}

void Action_Literal::set__complement(const Action_Literal__Pointer& _complement)
{
    assert(_complement.get() != this);

    has_complement = true;
    complement = _complement;
}

void Action_Literal::forced_wake(State& state) const
{
    INTERACTIVE_VERBOSER(true, 9075, "Waking action literal :: "
                         <<*this<<std::endl);
    
    assert(!get__sign());
    
    if(state.get__action_Literal()){
        if(state.get__action_Literal() == this){
            unsatisfy_listeners(state);
            satisfy_listeners(state);
            return;
        }
    }
    
    if(has_complement){
        complement->unsatisfy_listeners(state);
    }
    
    if(state.get__action_Literal()){
        state.get__action_Literal()->unsatisfy_listeners(state);
    }
    
    satisfy_listeners(state);

    if(state.get__action_Literal()){
        state.get__action_Literal()->satisfy_complement(state);
    }

    state.set__action_Literal(const_cast<Action_Literal*>(this));
}


void Action_Literal::satisfy_complement(State& state)
{
    if(has_complement){
        assert(complement->get__sign());
        complement->satisfy_listeners(state);
    }
}


void Action_Literal::starting__newly_satisfied(State& state) const
{
    satisfy_listeners(state);
}


void Action_Literal::report__newly_satisfied(State& state) const
{
    assert(0);
}

void Action_Literal::report__newly_unsatisfied(State& state) const
{
    assert(0);
}

            
uint Action_Literal::get__action_symbol() const
{
    return std::tr1::get<0>(contents());
}


bool Action_Literal::get__sign() const
{
    return std::tr1::get<1>(contents());
}

std::ostream& Action_Literal::operator<<(std::ostream&o) const
{
    if(Formula::Action_Proposition::ith_exists(get__runtime_Thread(), get__action_symbol())){
        auto proposition = Formula::Action_Proposition
            ::make_ith<Formula::Action_Proposition>(get__runtime_Thread()
                                                    , get__action_symbol());
        o<<((get__sign())?"¬":"")<<proposition<<"[|- :l:"<<get__id()<<":  :v:"<<get__action_symbol()<<": -|]";
    } else {
        o<<((get__sign())?"¬":"")<<get__action_symbol()<<"[|- :l:"<<get__id()<<": -|]";
    }
    
    return o;
}
    

namespace std
{
    std::ostream& operator<<(std::ostream& o
                             , const Planning::Action_Literal& in)
    {
        return in.operator<<(o);
    }
    
}
