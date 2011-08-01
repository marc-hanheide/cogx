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


#ifndef PLANNING_FORMULA_TO_NNF_HH
#define PLANNING_FORMULA_TO_NNF_HH

#include "planning_formula.hh"

namespace Planning
{
  class Planning_Formula__to__NNF
  {
  public:
      typedef Planning::Formula::Subformulae Subformulae;
      typedef Planning::Formula::Subformula Subformula;
      typedef Planning::Formula::Negation Negation;
      typedef Planning::Formula::Conjunction Conjunction;
      typedef Planning::Formula::Disjunction Disjunction;
      typedef Planning::Formula::Exists Exists;
      typedef Planning::Formula::Forall Forall;
      
      Subformula operator()(Subformula);
  private:
      Subformula operator()(Subformula, bool);

      /* Applies the double negative law*/
      Subformula operator()(Negation&, bool);
      
      /*Application of De Morgan's laws*/
      Subformula operator()(Conjunction&, bool);
      Subformula operator()(Disjunction&, bool);
  };
}

#endif
