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


#ifndef PLANNING_FORMULA_TO_PROBLEM_FORMULA_HH
#define PLANNING_FORMULA_TO_PROBLEM_FORMULA_HH


#include "planning_formula.hh"
#include "dtp_pddl_parsing_data.hh"

namespace Planning
{
    /* Translates a formula so that it only contains problem
     * (resp. domain) constant symbols. For example, a formula might
     * contain a predicate symbol (Predicate ?x Y), where ?x is a
     * variable symbol and Y is a "constant" symbol from the domain
     * definition. To translate this formula to a "problem" formula,
     * then we have to ensure that Y is an object that has been
     * associated with the problem (resp. a constant symbol associated
     * with the domain).*/
    class Planning_Formula__to__Problem_Formula
    {
    public:
        Planning_Formula__to__Problem_Formula
        (basic_type::Runtime_Thread,
         Planning::Parsing::Problem_Data&);
        
        /*General formula symbols.*/
        typedef Planning::Formula::Subformulae Subformulae;
        typedef Planning::Formula::Subformula Subformula;
        typedef Planning::Formula::Negation Negation;
        typedef Planning::Formula::Conjunction Conjunction;
        typedef Planning::Formula::Disjunction Disjunction;

        /*Boolean symbols.*/
        typedef Planning::Formula::State_Proposition State_Proposition;
        typedef Planning::Formula::Perceptual_Predicate Perceptual_Predicate;
        typedef Planning::Formula::Perceptual_Proposition Perceptual_Proposition;
        typedef Planning::Formula::State_Predicate State_Predicate;
        typedef CXX__deref__shared_ptr<State_Proposition> Ground_Fact;
        typedef CXX__deref__shared_ptr<Perceptual_Predicate> Perception;
        typedef CXX__deref__shared_ptr<Perceptual_Proposition> Ground_Perception;
        typedef CXX__deref__shared_ptr<State_Predicate> Fact;

        /*Action symbols.*/
        typedef Planning::Formula::Action_Proposition Action_Proposition;
        typedef Planning::Formula::Action_Predicate Action_Predicate;
        typedef CXX__deref__shared_ptr<Action_Proposition> Action_Ground_Fact;
        typedef CXX__deref__shared_ptr<Action_Predicate> Action_Fact;
        
        /*Function symbols.*/
        typedef Planning::Formula::State_Ground_Function State_Ground_Function;
        typedef Planning::Formula::State_Function State_Function;
        typedef Planning::Formula::Perceptual_Ground_Function Perceptual_Ground_Function;
        typedef Planning::Formula::Perceptual_Function Perceptual_Function;
        typedef CXX__deref__shared_ptr<State_Ground_Function> Ground_Map;
        typedef CXX__deref__shared_ptr<State_Function> Map;
        typedef CXX__deref__shared_ptr<Perceptual_Ground_Function> Ground_Perceptual_Map;
        typedef CXX__deref__shared_ptr<Perceptual_Function> Perceptual_Map;

        /* Arithmetic, assignment, and equality symbols.*/
        typedef Planning::Formula::Increase Increase;
        typedef Planning::Formula::Decrease Decrease;
        typedef Planning::Formula::Assign Assign;
        typedef Planning::Formula::Equality_Test Equality_Test;
        typedef CXX__deref__shared_ptr<Increase> Addition;
        typedef CXX__deref__shared_ptr<Decrease> Subtraction;
        typedef CXX__deref__shared_ptr<Assign> Assignment;
        typedef CXX__deref__shared_ptr<Equality_Test> Equality;
        
//         typedef Planning::Formula:: ;
//         typedef Planning::Formula:: ;
//         typedef CXX__deref__shared_ptr<> ;
//         typedef CXX__deref__shared_ptr<> ;
        
        
        Subformula operator()(Subformula);
    private:
        
        Subformula operator()(Fact);
        Subformula operator()(Ground_Perception);
        Subformula operator()(Perception);
        Subformula operator()(Ground_Fact);
        Subformula operator()(Addition);
        Subformula operator()(Subtraction);
        Subformula operator()(Assignment);
        Subformula operator()(Equality);
        Subformula operator()(Ground_Map);
        Subformula operator()(Map);
        Subformula operator()(Ground_Perceptual_Map);
        Subformula operator()(Perceptual_Map);
        Subformula operator()(Action_Ground_Fact);
        Subformula operator()(Action_Fact);

        /* List of problem constants/objects. */
        Planning::Constants_Description& constants_Description;
        
        /* The thread-space in which new formulae should be
         * created.
         */
        basic_type::Runtime_Thread runtime_Thread;
        
        /* Problem of discourse. */
        Planning::Parsing::Problem_Data& problem_Data;
    };
}


#endif
