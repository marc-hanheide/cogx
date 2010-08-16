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
#include "float_state.hh"


namespace Planning
{ 
    class Markov_Decision_Process_State
    {
    public:
        std::ostream& operator<<(ostream&) const;
            
        Markov_Decision_Process_State(uint propositions_count = 0, uint function_count = 0);
        Markov_Decision_Process_State(const Markov_Decision_Process_State&);
        Markov_Decision_Process_State& operator=(const Markov_Decision_Process_State&);

        uint get__number_of_atoms() const;
        uint get__number_of_int_fluents() const;
        uint get__number_of_double_fluents() const; 
        
	bool operator==(const Markov_Decision_Process_State& state) const;
	bool operator<(const Markov_Decision_Process_State& state) const;
	std::size_t hash_value() const;

        /* (see \member{boolean_State}) */
        void flip(uint);
        bool is_true(uint) const;

        /* What is the value of the real number at \argument{index}.*/
        double get__float(uint index) const;
        void set__float(uint index, double value);
        
        /* What is the value of the int number at \argument{index}.*/
        int get__int(uint index) const;
        void set__int(uint index, int value);

        /* Interface assumes that all the transitions associated with
         * a given action A are entered before transitions associated
         * with some distinct transition B!=A are entered.
         *
         * - \argument{Markov_Decision_Process_State} is the successor
         * state under the transition.
         *
         * - \argument{double} is the probability of the transition occurring.
         *
         * - \argument{operator_index} is the index of the operator
         * that when executed can cause the transition.*/
        void push__successor(uint operator_index, const Markov_Decision_Process_State*, double);
    protected:
        /*Value of this MDP state.*/
        double value;
        
        std::vector<uint> successor_driver;
        std::vector< std::vector<const Markov_Decision_Process_State*> > successors;
        std::vector< std::vector<double> > successor_probability;
        
        /* State-characterising functions (derived from PDDL "Fluents"). */
        Integer_State integer_State;

        /* State-characterising propositions. */
        Boolean_State boolean_State;
        
        /* Functions into the reals. */
        Float_State float_State;
    };
}

#endif
