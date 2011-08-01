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

#ifndef PLANNING_FORMULA_TO_CNF_HH
#define PLANNING_FORMULA_TO_CNF_HH

#include "planning_formula.hh"
#include "planning_formula_to_nnf.hh"

#include "stl__deref_tools.hh"

namespace Planning
{
    /* We first convert to NNF ---Negation Normal Form---, and then to
     * CNF -- Conjunctive Normal Form. */
    class Planning_Formula__to__CNF
    {
    public:
        
        typedef Planning::Formula::Subformulae Subformulae;
        typedef Planning::Formula::Subformula Subformula;
        typedef Planning::Formula::Negation Negation;
        typedef Planning::Formula::Conjunction Conjunction;
        typedef Planning::Formula::Disjunction Disjunction;
        typedef Planning::Formula::Exists Exists;
        typedef Planning::Formula::Forall Forall;

        
        typedef CXX__deref__shared_ptr<Conjunction> Conjunct;
        typedef std::vector<Conjunct> Conjuncts;
        
        typedef CXX__deref__shared_ptr<Disjunction> Disjunct;
        typedef std::vector<Disjunct> Disjuncts;

        
        /* \argument{bool}: Should we compute an NNF version of the
         * argument before proceeding.
         *
         * WEARNING: If the \argument{Subformula} is \type{Vacuous},
         * then so is the output.*/
        Subformula operator()(Subformula, bool = false);
        
    private:
        
        /* Turns this disjunction-of-conjunctions \argument{data} into
         * a conjunction-of-disjunctions \result{Subformula}.*/
        Subformulae& distributive_law(Subformulae&,
                                      Subformulae,
                                      const Conjuncts& data,
                                      uint index);
        
        Subformula operator()(Disjunct);
        Subformula operator()(Conjunct);
        
        Planning_Formula__to__NNF planning_Formula__to__NNF;
    };
}


#endif
