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


#ifndef CNF__STATE_HH
#define CNF__STATE_HH

#include "boolean__satisfaction_status_management.hh"
#include "unsigned_integer__status_management.hh"
#include "action_basics.hh"

namespace Planning
{

    class CNF__State
    {
    public:
        CNF__State(uint formulae_count = 0
                   , uint disjunctions_count = 0
                   , uint action_formulae_count = 0
                   , uint action_disjunctions_count = 0);
        
        const Unsigned_Integer__Satisfaction_Status_Management& get__cnfs__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__cnfs__count_status();
        const Boolean__Satisfaction_Status_Management& get__cnfs__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__cnfs__satisfaction_status();
        
        const Unsigned_Integer__Satisfaction_Status_Management& get__action_cnfs__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__action_cnfs__count_status();
        const Boolean__Satisfaction_Status_Management& get__action_cnfs__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__action_cnfs__satisfaction_status();
        
        
        const Unsigned_Integer__Satisfaction_Status_Management& get__clauses__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__clauses__count_status();
        const Boolean__Satisfaction_Status_Management& get__clauses__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__clauses__satisfaction_status();


        const Unsigned_Integer__Satisfaction_Status_Management& get__action_clauses__count_status() const;
        Unsigned_Integer__Satisfaction_Status_Management& get__action_clauses__count_status();
        const Boolean__Satisfaction_Status_Management& get__action_clauses__satisfaction_status() const;
        Boolean__Satisfaction_Status_Management& get__action_clauses__satisfaction_status();
        
        
        void set__action_Literal(Action_Literal*);
        Action_Literal* get__action_Literal();
    private:
//         int last_action_executed;
        
        /* Tracks the number of clauses satisfied in each CNF -- Where
         * this is not a starting state, the values should be
         * initially copied from the predecessor state. */
        Unsigned_Integer__Satisfaction_Status_Management cnfs__count_status;
        /* Tracks what CNFs are satisfied.  -- Where
         * this is not a starting state, the values should be
         * initially copied from the predecessor state. */
        Boolean__Satisfaction_Status_Management cnfs__satisfaction_status;

        
        /* Tracks the number of literals satisfied in each clause.  -- Where
         * this is not a starting state, the values should be
         * initially copied from the predecessor state. */
        Unsigned_Integer__Satisfaction_Status_Management clauses__count_status;
        /* Tracks what clauses are satisfied.  -- Where
         * this is not a starting state, the values should be
         * initially copied from the predecessor state. */
        Boolean__Satisfaction_Status_Management clauses__satisfaction_status;

        
        /* Tracks the number of action-literals satisfied in each
         * action-clause.  -- Where this is not a starting state, the
         * values should be initially all false. */
        Unsigned_Integer__Satisfaction_Status_Management action_clauses__count_status;
        /* Tracks what action-clauses are satisfied.  -- Where
         * this is not a starting state, the values should be
         * initially all false. */
        Boolean__Satisfaction_Status_Management action_clauses__satisfaction_status;

        
        
        /* Tracks the number of action-clauses satisfied in each CNF -- Where
         * this is not a starting state, the values should be
         * initially all false. */
        Unsigned_Integer__Satisfaction_Status_Management action_cnfs__count_status;
        /* Tracks what action-CNFs are satisfied.  -- Where
         * this is not a starting state, the values should be
         * initially all false. */
        Boolean__Satisfaction_Status_Management action_cnfs__satisfaction_status;

        Action_Literal* action_Literal;
        
//         /* Tracks what literals are satisfied.  -- Where
//          * this is not a starting state, the values should be
//          * initially copied from the predecessor state. */
//         Boolean__Satisfaction_Status_Management literals__satisfaction_status;
    };
}


#endif
