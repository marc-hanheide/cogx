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


#ifndef ACTION__LITERAL_HH
#define ACTION__LITERAL_HH


#include "action_basics.hh"
#include "state_formula.hh"
#include "planning_types_enum.hh"

namespace Planning
{
    /*Created as unsatisfied.*/
    class Action_Literal
        : public State_Formula::_Satisfaction_Listener<enum_types::action_literal /* Type identifier.*/
                                        , uint /* Action symbol identifier.*/
                                        , bool >
    {PRINTING;
    public:
        typedef State_Formula::_Satisfaction_Listener<enum_types::action_literal
                                                      , uint 
                                                      , bool > Parent;
        
        void forced_wake(State&) const;
        
        Action_Literal();
        
        void report__newly_satisfied(State&) const;
        void report__newly_unsatisfied(State&) const;
            
        /* Action symbol that is the subject of this literal.*/
        uint get__action_symbol() const;

        /* Sign of this literal (false is positive, true is negative). */
        bool get__sign() const;

        void starting__newly_satisfied(State& state) const;
        
        void satisfy_complement(State& state);
        
        void configure__complement(const Action_Literal__Pointer&, const Action_Literals&);
//         void configure__negatives(CXX__PTR_ANNOTATION(List__Action_Literals)&);
    private:
        void set__complement(const Action_Literal__Pointer&);
        /* The complement of this literal.*/
        bool has_complement;
        Action_Literal__Pointer complement;
        
//         /*Only one action can be executed at a time.*/
//         CXX__PTR_ANNOTATION(List__Action_Literals) negatives;
    };
 
}



#endif
