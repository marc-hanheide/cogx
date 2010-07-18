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


#include "markov_decision_process_state.hh"

#include "boolean__satisfaction_status_management.hh"
#include "unsigned_integer__status_management.hh"

#include "basic_action.hh"

namespace Planning
{
    class Solver;
    
    class State : public Markov_Decision_Process_State
    {
    public:

//         bool operator<(const State&) const;
//         bool operator==(const State&) const;
// 	std::size_t hash_value() const;
        

        void add__compulsory_transformation(State_Transformation*);
        void retract__compulsory_transformation(State_Transformation*);
        void add__optional_transformation(State_Transformation*);
        void retract__optional_transformation(State_Transformation*);

        
        const Unsigned_Integer__Satisfaction_Status_Management& get__cnfs__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__cnfs__count_status();
        const Boolean__Satisfaction_Status_Management& get__cnfs__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__cnfs__satisfaction_status();
        const Unsigned_Integer__Satisfaction_Status_Management& get__clauses__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__clauses__count_status();
        const Boolean__Satisfaction_Status_Management& get__clauses__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__clauses__satisfaction_status();
        const Boolean__Satisfaction_Status_Management& get__literals__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__literals__satisfaction_status();
    private:
        Unsigned_Integer__Satisfaction_Status_Management cnfs__count_status;
        Boolean__Satisfaction_Status_Management cnfs__satisfaction_status;
        Unsigned_Integer__Satisfaction_Status_Management clauses__count_status;
        Boolean__Satisfaction_Status_Management clauses__satisfaction_status;
        Boolean__Satisfaction_Status_Management literals__satisfaction_status;


        /* A compulsory transformation is one that has to be evaluated before the search can continue.*/
        std::set<State_Transformation*> applicable_compulsory_transformations;
        /* An optional transformation is an action that an agent can choose to execute at a state.*/
        std::set<State_Transformation*> applicable_optional_transformations;

        
        /* Planner that generated this state.*/
        Solver* solver;
    };
}

#endif
