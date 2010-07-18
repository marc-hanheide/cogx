
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


#ifndef STATE_BASICS_HH
#define STATE_BASICS_HH


#include "global.hh"

/*The type of the elements in the bit vector of a \class{State} --
 *Should be an unsigned integer.*/
//#define ELEM_TYPE unsigned char
#define ELEM_TYPE unsigned int

/*Number of bits in each element of the bitvector*/
#define SIZE_ELEM (sizeof(ELEM_TYPE) << 3)

#define CEIL(X) ( X % SIZE_ELEM )?( ( X / SIZE_ELEM ) + 1):(X / SIZE_ELEM)
#define FLOOR(X) X / SIZE_ELEM
#define REMAINDER(X) X % SIZE_ELEM

namespace Planning
{
    class Boolean_State;
    class Integer_State;
    class Markov_Decision_Process_State;
    class Partially_Observable_Markov_Decision_Process_State;
    class State;
}

namespace std
{    
    /* (see \module{boolean_state.hh}) */
    std::size_t hash_value(const Planning::Boolean_State&);

    /* (see \module{boolean_state.hh}) */
    std::ostream& operator<<(std::ostream&, const Planning::Boolean_State&);
    
    /* (see \module{integer_state.hh}) */
    std::size_t hash_value(const Planning::Integer_State&);

    /* (see \module{integer_state.hh}) */
    std::ostream& operator<<(std::ostream&, const Planning::Integer_State&);

    /* (see \module{markov_decision_process_state.hh}) */
    std::size_t hash_value(const Planning::Markov_Decision_Process_State&);
    
    /* (see \module{planning_state.hh}) */
    std::size_t hash_value(const Planning::State&);
}


#endif
