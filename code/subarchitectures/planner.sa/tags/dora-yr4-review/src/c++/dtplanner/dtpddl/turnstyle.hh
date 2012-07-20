/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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

#ifndef TURNSTYLE_HH
#define TURNSTYLE_HH

#include "global.hh"

namespace Turnstyle
{   
    /* The plan is to turn action preconditions into propositional
     * formulae, and then those in-turn into CNF. */
    class CNF
    {
    public:
        typedef unsigned int Atom;
        typedef int Literal;
        
        typedef std::set<Literal> Clause;
        typedef std::set<Clause> Problem_Data;

        /* After construction, input is stored in a (simplified) form in
         * \member{problem_Data}.*/
        CNF(Problem_Data&);

        /* What is the longest clause in this problem? */
        int get_length_of_longest_clause(Problem_Data&) const;

        /* The empty disjunction is unsatisfiable, hence if this CNF
         * contains an empty clause it is not satisfiable.*/
        bool check__no_empty_clause();

        /* If a clause contains both a literal and its negation, then
         * it is trivially satisfiable -- i.e, If the valuation has
         * that the literal is true, then the clause is satisfied, and
         * otherwise it is satisfied because necessarily that literals
         * negation satisfies the clause. */
        Problem_Data simplify__remove_A_and_NOT_A(Problem_Data&);

        /* A clause can sometimes subsume another. In this case, the
         * clause that is subsumed can be removed from the problem .*/
        Problem_Data simplify__subsumption(Problem_Data&);

        /* Repeatedly performs unit-propagation until that is no
         * longer possible.*/
        Problem_Data simplify__unit_prop(Problem_Data&);

        /* Is the argument conjunction of unit-clauses satisfiable --
         * SOUND but NOT COMPLETE.*/
        bool satisfiable_conjunct(Problem_Data& ) const;
        
        /*Does A subsume B?*/
        bool clause_subsumes(const Clause& A, const Clause& B) const;
        bool clause_trivial(const Clause& A) const;
        
        Problem_Data collect__size_equals(Problem_Data&, unsigned int);
        Problem_Data collect__size_greater_than(Problem_Data&, unsigned int);
        
        enum Satisfiability {yes,no,unknown};
        Satisfiability satisfiability;
        
        Problem_Data problem_Data;
    };

    
    /* EARLY TESTING FUNCTION.*/
    void test__turnstyle_hh();
}

namespace std
{
    
    std::ostream& operator<<(std::ostream&, const Turnstyle::CNF&);
    std::ostream& operator<<(std::ostream&, const Turnstyle::CNF::Problem_Data&);
    std::ostream& operator<<(std::ostream&, const Turnstyle::CNF::Clause&);
}


#endif

/* April Fool, motherfucker.
 *
 * -- Vincent Okamoto quoted on page 361 of the 2003 edition of
 * "Patriots: the Vietnam War remembered from all sides", by Christian
 * G. Appy. Vincent was discussing the USA's Phoenix program.
 */
