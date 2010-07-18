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


#ifndef MARKOV_DECISION_PROCESS_STATE_HH
#define MARKOV_DECISION_PROCESS_STATE_HH

#include "integer_state.hh"
#include "boolean_state.hh"


namespace Planning
{ 
    class Markov_Decision_Process_State
    {
    public:
        Markov_Decision_Process_State(uint propositions_count = 0, uint function_count = 0);
        Markov_Decision_Process_State(const Markov_Decision_Process_State&);

        Markov_Decision_Process_State& operator=(const Markov_Decision_Process_State&);
        
	bool operator==(const Markov_Decision_Process_State& state) const;
	bool operator<(const Markov_Decision_Process_State& state) const;
	inline std::size_t hash_value() const;
        
    protected:
        /* For each \LHS{action}, we have the \RHS{probability} that
         * that action leads to a particular successor. */
        typedef std::tr1::tuple<uint, double> Successor_Index;

        /* Description of state transition function from *this
         * state. */
        typedef std::map<Successor_Index
                         , Markov_Decision_Process_State> Successors;

        /* Probability of getting a particular successor state. */
        typedef std::map<Markov_Decision_Process_State
                         , Successor_Index> Successor_Probability;

        /* (see \typedef{Successors}) */
        Successors successors;
        
        /* (see \typedef{Successor_Probability}) */
        Successor_Probability successor_Probability;
        
        /* State-characterising functions (derived from PDDL "Fluents"). */
        Integer_State integer_State;

        /* State-characterising propositions. */
        Boolean_State boolean_State;
    };
}

#endif
