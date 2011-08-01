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

#ifndef PLANNING_FORMULA_TO_VARIABLE_ORDERING_HH
#define PLANNING_FORMULA_TO_VARIABLE_ORDERING_HH


#include "planning_formula.hh"
#include "dtp_pddl_parsing_data.hh"

namespace Planning
{

    /* ASUME :: Iput formula is assumed to be in CNF format.
     *
     * Compute a good ordering over formula variables in which to try
     * a satisfying assignment from variables to problem objects. The
     * ordering is determined by the constrainedness of variables
     * occurring in the formula. Those that seem more constrained are
     * ordered before those that seem less constrained.*/
    class Planning_Formula__to__Variable_Ordering : public Visitor<basic_type>
    {
    public:
        Planning_Formula__to__Variable_Ordering(const Planning::Parsing::Domain_Data&);
        
        DECLARATION__UNARY_VISITOR(basic_type)

        /* \result{std::vector<Variable>} is ordered, so that elements
         * occurring earlier a better than those occurring later.
         *
         * WARNING: This does not necessarily return an entry for
         * every variable symbol in the formula. We do not transition
         * to subformulae in the case of a quantifier for example, so
         * would miss variable symbols that only occur in that
         * scope.  */
        std::vector<Variable> get__answer();
    private:
        void operator()(const Planning::Formula::Conjunction&);
        void operator()(const Planning::Formula::State_Predicate&);
        void operator()(const Planning::Formula::State_Function&);
//         void operator()(const Planning::Formula::Perceptual_Function&);
//         void operator()(const Planning::Formula::Observational_Predicate&);

        /* \arguemnt{uint} is the score associated with
         * \argument{Planning::Arguments_List}*/
        void operator()(const Planning::Argument_List&, uint);

        /* Domain of discourse. */
        const Planning::Parsing::Domain_Data& domain_Data;
        
        /* Are we processing a negative literal? */
        bool processing_negative;
        
        /* Are we processing a length-one clause? */
        bool processing_hard_constraint;
        
        /* Score of the problem variables */
        std::map<Variable, uint> scores;

        /* Variables of a given score. */
        std::map<uint, Variables> scores_to_variables;
    };
}


#endif
