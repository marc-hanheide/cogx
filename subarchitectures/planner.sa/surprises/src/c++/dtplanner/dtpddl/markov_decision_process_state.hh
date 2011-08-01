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

#include "state_basics.hh"
#include "observation_basics.hh"

#include "expandable.hh"
#include "probability_during_expansion__state.hh"

namespace Planning
{ 
    class Markov_Decision_Process_State
        : public Probability_During_Expansion_State,
          public Expandable
    {
    public:
        virtual ~Markov_Decision_Process_State(){}
        
        std::ostream& operator<<(ostream&) const;
            
        Markov_Decision_Process_State(uint propositions_count = 0, uint function_count = 0);
        
        Markov_Decision_Process_State& operator=(const Markov_Decision_Process_State&) = delete;

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

        /*This should be the instantanious reward associated with being in this state. (see \member{reward})*/
        double get__reward() const;
        void set__reward(double);
        
        typedef std::vector<uint> Successor_Driver;/*action indices*/
        typedef std::vector< std::vector<Markov_Decision_Process_State*> > Successors;/*states*/
        typedef std::vector< std::vector<double> > Successor_Probabilities;/*their probabilities.*/
        
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
        void push__successor(uint operator_index,  Markov_Decision_Process_State*, double);
        const Successors& get__successors() const;
        const Successor_Probabilities& get__successor_Probabilities() const;
        const Successor_Driver& get__successor_Driver() const;

        /* For computing the set differences between actions available
         * at distinct MDP states, it is useful to have a sorted
         * version of \member{successor_Driver}.*/
        const Successor_Driver& get__sorted__successor_Driver() const;

        bool has__considered_observations_under_action(uint action_id) const;
        void report__considered_observations_under_action(uint action_id);
        
        void push__observation(uint action_id, Observational_State* observation, double probability);
        const std::vector<std::vector<Observational_State*> >& get__observations() const;
        const std::vector<std::vector<double> >& get__observation_Probabilities() const;
        const std::vector<uint>& get__observation_Driver() const;
        
        /*Get the element index of entry \argument{action_id} in \member{action_to_observation}.*/
        uint get__action_to_observation__index(uint action_id) const;
        bool action_to_observation__includes_index(uint action_id) const;

        /* The \member{value} gives the discounted cumulative reward
         * that an be expected over an infinite horizon, assuming
         * rewards are discounted geometrically into the future. */
        double get__value() const;
        void set__value(double);
        
    protected:
        Markov_Decision_Process_State(const Markov_Decision_Process_State&);// = delete;
        
        /* If a number is in this set, then we have already considered
         * what observations occur when the corresponding action is
         * executed and we arrive at this state.*/
        std::set<uint> considered_observations_under_action;
        
        
        std::vector<uint> action_to_observation;
        std::map<uint, uint> mirror__action_to_observation;
        
        std::vector<std::vector<Observational_State*> > observation__given_action;
        std::vector<std::vector<double> > observation_probability__given_action;
        
        
        Successor_Driver successor_Driver;
        mutable Successor_Driver sorted__successor_Driver;
        Successors successors;
        Successor_Probabilities successor_Probabilities;
        
        /* State-characterising propositions. */
        Boolean_State boolean_State;
        
        /* State-characterising functions (derived from PDDL "Fluents"). */
        Integer_State integer_State;

        /*Reward associated with being in this MDP state.*/
        double reward;

        /* Value associated with this MDP state. If meaningful, this
         * should have been computed by some kind of DP, or stochastic
         * DP. */
        double value;
        
        /* Functions into the reals. */
        Float_State float_State;
    };

    typedef Markov_Decision_Process_State MDP_State;
}

#endif
