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


void Action_Literal::report__newly_satisfied(State& state) const
{
//     Action_Literal__Pointer negation_of_this;
    
    /*Is a positive symbol.*/
    if(!get__sign()){
        for(auto negative = negatives->begin()
                ; negative != negatives->end()
                ; negative++){
            if(get__action_symbol() != (*negative)->get__action_symbol()){
                (*negative)->report__newly_satisfied(state);
            } else {
                // (*negative)->report__newly_unsatisfied(state);
//                 negation_of_this = Action_Literal__Pointer(*negative); 
            }
        }
    }
    
    auto listeners = get__traversable__listeners();
    for(auto listener = listeners.begin()
            ; listener != listeners.end()
            ; listener++){
        INTERACTIVE_VERBOSER(true, 7002, "Just SATISFIED literal  :: "<<*this<<std::endl
                             <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
        (*listener).cxx_get<Satisfaction_Listener>()->report__newly_satisfied(state);
    }
    
    
    /*Is a positive symbol.*/
    if(!get__sign()){

        
        auto listeners = get__traversable__listeners();
        for(auto listener = listeners.begin()
                ; listener != listeners.end()
                ; listener++){
            INTERACTIVE_VERBOSER(true, 7002, "Just UNSATISFIED literal  :: "<<*this<<std::endl
                                 <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
            (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
        }
        
        for(auto negative = negatives->begin()
                ; negative != negatives->end()
                ; negative++){
            if(get__action_symbol() != (*negative)->get__action_symbol()){
                (*negative)->report__newly_unsatisfied(state);
            } // else {
//                 (*negative)->report__newly_unsatisfied(state);
//                 negation_of_this = Action_Literal__Pointer(*negative); 
//             }
        }
    }
}

void Action_Literal::report__newly_unsatisfied(State& state) const
{
    auto listeners = get__traversable__listeners();
    for(auto listener = listeners.begin()
            ; listener != listeners.end()
            ; listener++){
        INTERACTIVE_VERBOSER(true, 7002, "Just UNSATISFIED literal  :: "<<*this<<std::endl
                             <<"Waking listener :: "<<(*listener).cxx_get<Satisfaction_Listener>()<<std::endl);
        (*listener).cxx_get<Satisfaction_Listener>()->report__newly_unsatisfied(state);
    }
}

            
uint Action_Literal::get__action_symbol() const
{
    return std::tr1::get<0>(contents());
}


bool Action_Literal::get__sign() const
{
    return std::tr1::get<1>(contents());
}

            
void Action_Literal::configure__negatives(CXX__PTR_ANNOTATION(List__Action_Literals)& in)
{
    negatives = in;
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
