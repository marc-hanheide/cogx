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


#ifndef PLANNING_CNF_TO_STATE_CNF_HH
#define PLANNING_CNF_TO_STATE_CNF_HH


#include "planning_formula.hh"
#include "state_formula.hh"
#include "planning_formula.hh"
 

namespace Planning
{    
    class Planning_CNF__to__State_CNF : public Visitor<basic_type>
    {
    public:
        Planning_CNF__to__State_CNF(basic_type::Runtime_Thread
                                    , Formula::State_Propositions&
                                    , State_Formula::Literals&
                                    , State_Formula::Disjunctive_Clauses&
                                    , State_Formula::Conjunctive_Normal_Form_Formulae&);
        
        DECLARATION__UNARY_VISITOR(basic_type);
        
        State_Formula::Conjunctive_Normal_Form_Formula__Pointer get__answer() const ;
    private:
        State_Formula::Conjunctive_Normal_Form_Formula__Pointer answer;
        
        Formula::State_Propositions problem__state_Propositions;
        State_Formula::Literals& problem__literals;
        State_Formula::Disjunctive_Clauses& problem__clauses;
        State_Formula::Conjunctive_Normal_Form_Formulae& problem__cnfs;

        /* Last clause parsed. */
        State_Formula::List__Literals clause;
        State_Formula::Literals clause__as_set;
        
        /* Last problem parsed. */
        State_Formula::List__Disjunctive_Clause disjunctions;
        State_Formula::Disjunctive_Clauses disjunctions__as_set;

        
        
        /* Are we processing a negative literal? */
        bool processing_negative;

        /* Object with which to associate CNF generated by objects of this type.*/
        basic_type::Runtime_Thread runtime_Thread;
        
    };
}


#endif
