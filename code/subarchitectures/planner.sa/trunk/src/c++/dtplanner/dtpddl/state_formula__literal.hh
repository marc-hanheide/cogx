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

#ifndef STATE_FORMULA__LITERAL_HH
#define STATE_FORMULA__LITERAL_HH

#include "state_formula.hh"

namespace Planning
{
    namespace State_Formula
    {
        class Literal
            : public _Satisfaction_Listener<enum_types::literal /* Type identifier.*/
                                            , uint /* Boolean variable identifier.*/
                                            , bool >
        {PRINTING;
        public:

            void report__newly_satisfied(State&);
            void report__newly_unsatisfied(State&);
            
            void set__satisfied(State&);
            void set__unsatisfied(State&);
            void flip_satisfaction(State&);
            bool is_satisfied(const State&) const;

            
            /* Set the subject variable to have the truth value "TRUE".*/
            void flip_variable_on(State&);
            
            /* Set the subject variable to have the truth value "FALSE".*/
            void flip_variable_off(State&);
            
            /* Set the subject variable to have the opposite truth
             * value to its current assignment.*/
            void flip(State&);

            /* Variable, that is the subject of this literal.*/
            uint get__variable() const;

            /* Sign of this literal (true is positive, false is negative). */
            bool get__sign() const;                   
        };
 
    }
}


#endif
