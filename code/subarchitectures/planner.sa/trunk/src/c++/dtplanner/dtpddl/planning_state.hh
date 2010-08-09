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

namespace Planning
{

    class State : public Markov_Decision_Process_State,
        public CNF__State,
        public Action_Executability__State
    {
    public:
        /* Planner that generated this state.*/
        Solver* solver;
        
        uint count__compulsory_generative_transformations() const;
        State_Transformation* pop__compulsory_generative_transformation();
        void push__compulsory_generative_transformation(State_Transformation*);

        
        uint count__compulsory_transformations() const;
        State_Transformation* pop__compulsory_transformation();
        void push__compulsory_transformation(State_Transformation*);

        
        uint count__probabilistic_transformations() const;
        Probabilistic_State_Transformation* pop__probabilistic_transformation();
        void push__probabilistic_transformation(Probabilistic_State_Transformation*);

        
        std::set<State_Transformation*>& get__optional_transformations();
        void add__optional_transformation(State_Transformation*);
        void remove__optional_transformation(State_Transformation*);
    private:

        /*Pending probabilistic transformations. All such transformations are compulsory.*/
        std::stack<Probabilistic_State_Transformation*> probabilistic_transformations;
        
        /* A compulsory transformation is one that has to be evaluated before the search can continue.*/
        std::stack<State_Transformation*> applicable_compulsory_transformations;
        
        /* A compulsory transformation is one that has to be evaluated before the search can continue.*/
        std::stack<State_Transformation*> applicable_compulsory_generative_transformations;
        
        /* An optional transformation is an action that an agent can choose to execute at a state.*/
        std::set<State_Transformation*> applicable_optional_transformations;
    };
    
    /*State pointers.*/
    typedef std::tr1::unordered_set<State*, /*state_hash*/deref_hash<State>,  deref_equal_to<State> > SetOfStatePointers;
}

#endif
