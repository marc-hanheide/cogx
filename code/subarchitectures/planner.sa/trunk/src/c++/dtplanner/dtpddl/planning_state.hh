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

#ifndef PLANNING_STATE_HH
#define PLANNING_STATE_HH

#include "solver_basics.hh"
#include "state_basics.hh"
#include "action_basics.hh"

#include "markov_decision_process_state.hh"
#include "action_executability__state.hh"
#include "cnf__state.hh"

// inline std::size_t hash_value(const Planning::State& in)
// {
//     return std::hash_value(in);
// }

namespace Planning
{

    inline std::size_t hash_value(const Planning::State& in)
    {
        return std::hash_value(in);
    }

    class State : public Markov_Decision_Process_State,
        public CNF__State,
        public Action_Executability__State
    {
    public:
        std::ostream& operator<<(std::ostream&) const;
        
        State(Solver& solver,/*Object that is solving the problem that involves this state.*/
              uint propositions_count = 0,/*Number of non-static state-characterising PROPOSITIONS.*/
              uint function_count = 0,/*Number of non-static state-characterising FLUENTS/FUNCTIONS.*/
              uint formulae_count = 0,/*Number of CNF formulae in problem description.*/
              uint disjunctions_count = 0,/*Number of disjunctive-clauses in the problem description.*/
              uint literals_count = 0, /*Number of literals (i.e., CNF atoms) in the problem desorption.*/
              uint actions_count = 0 /*Number of state transformations, including actions, in the problem description.*/
              );
        
        /* Planner that generated this state.*/
        Solver& solver;

        void reset__probability_during_expansion();
        double get__probability_during_expansion() const;
        double set__probability_during_expansion(double);
        
        uint count__compulsory_generative_transformations() const;
        const State_Transformation* pop__compulsory_generative_transformation();
        void push__compulsory_generative_transformation(const State_Transformation*);

        
        uint count__compulsory_transformations() const;
        const State_Transformation* pop__compulsory_transformation();
        void push__compulsory_transformation(const State_Transformation*);

        
        uint count__probabilistic_transformations() const;
        const Probabilistic_State_Transformation* pop__probabilistic_transformation();
        void push__probabilistic_transformation(const Probabilistic_State_Transformation*);

        
        std::set<const State_Transformation*> get__optional_transformations();
        void add__optional_transformation(const State_Transformation*);
        void remove__optional_transformation(const State_Transformation*);
    private:

        /*Pending probabilistic transformations. All such transformations are compulsory.*/
        std::stack<const Probabilistic_State_Transformation*> probabilistic_transformations;
        
        /* A compulsory transformation is one that has to be evaluated before the search can continue.*/
        std::stack<const State_Transformation*> applicable_compulsory_transformations;
        
        /* A compulsory transformation is one that has to be evaluated before the search can continue.*/
        std::stack<const State_Transformation*> applicable_compulsory_generative_transformations;
        
        /* An optional transformation is an action that an agent can choose to execute at a state.*/
        std::set<const State_Transformation*> applicable_optional_transformations;

    private:
        /* Probability during expansion.*/
        double probability_during_expansion;
        
        /* For each action in
         * \member{Markov_Decision_Process_State::successor_driver},
         * we kept the expected value of executing that action. */
        std::vector<double> expected_successor_value;

        /* Index to the maximum value in \member{expected_successor_value}.*/
        uint index__max_expected_successor_value;

        /* Value of the maximum entry in \member{expected_successor_value}.*/
        double value__max_expected_successor_value;
    };
    
    /*State pointers.*/
    typedef std::tr1::unordered_set<State*
                                    , /*state_hash*/deref_hash<State>
                                    ,  deref_equal_to<State> > Set_Of_State_Pointers;
}


#endif
