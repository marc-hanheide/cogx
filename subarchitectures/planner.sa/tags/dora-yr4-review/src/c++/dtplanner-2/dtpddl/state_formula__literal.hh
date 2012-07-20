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
        /*Created as unsatisfied.*/
        class Literal
            : public _Satisfaction_Listener<enum_types::literal /* Type identifier.*/
                                            , uint /* Boolean variable identifier.*/
                                            , bool >
        {PRINTING;
        public:

            void report__newly_satisfied(State&) const;
            void report__newly_unsatisfied(State&) const;

            /*Is this literal satisfied?*/
            bool is_satisfied(const State&) const;
            
            /* Changes the satisfaction status in \argument{State} of
             * the Boolean valued object.*/
            void flip(State&) const;
            
            /* Variable, that is the subject of this literal.*/
            uint get__variable() const;

            /* Sign of this literal (false is positive, true is negative). */
            bool get__sign() const;

            /*(see \member{can_only_be_flipped_once})*/
            bool get__can_only_be_flipped_once() const;
            /*(see \member{can_only_be_flipped_once})*/
            void set__can_only_be_flipped_once(bool = true);
            
            
            void configure__complement(const Literal__Pointer&, const Literals&);
        private:
            void set__complement(const Literal__Pointer&);
            void set__satisfied(State&) const;
            void set__unsatisfied(State&) const;

            /* The complement of this literal.*/
            bool has_complement;
            Literal__Pointer complement;

            /* After the first change, this literal's Boolean value is fixed.*/
            bool can_only_be_flipped_once;
        };
 
    }
}


#endif
