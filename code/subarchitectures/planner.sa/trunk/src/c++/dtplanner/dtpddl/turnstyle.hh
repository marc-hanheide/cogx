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
    /* EARLY TESTING FUNCTION.*/
    void test__turnstyle_hh();
    
    using std::set;
    
    /* The plan is to turn action preconditions into propositional
     * formulae, and then those in turn into CNF. */
    class CNF
    {
    public:

        
        typedef unsigned int Atom;
        typedef int Literal;
        
        typedef set<Literal> Clause;
        typedef set<Clause> Problem_Data;

        /* After construction, input is stored in a (simplified) form in
         * \member{problem_Data}.*/
        CNF(Problem_Data&);
        
        int get_length_of_longest_clause(Problem_Data&) const;
        
        bool check__no_empty_clause();
        
        Problem_Data simplify__remove_A_and_NOT_A(Problem_Data&);
        Problem_Data simplify__subsumption(Problem_Data&);
        Problem_Data simplify__unit_prop(Problem_Data&);

        /*Is the argument conjunction of unit-clauses satisfiable.*/
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
}

std::ostream& operator<<(std::ostream&, const Turnstyle::CNF&);
std::ostream& operator<<(std::ostream&, const Turnstyle::CNF::Problem_Data&);
std::ostream& operator<<(std::ostream&, const Turnstyle::CNF::Clause&);

// std::ostream& operator<<(std::ostream&, const Turnstyle::CNF::Literal&);

namespace Turnstyle
{
    
    
//     enum type_names
//     {
//         atom,
//         predicate,
//         proposition,
//         variable,
//         constant,
//         object,
//         type_name,
//         finite_domain__function,
//         integer__function,
//         real_valued__function
//     };

//     class Atom : public type_wrapper<type_names::atom, int>{
//     public:
//         typename set<Symbol> Symbol_Table;
//         static Symbol_Table symbol_Table;
        
//         set<int> clauses_in_which_occurs_POSITIVE;
//         set<int> clauses_in_which_occurs_NAGATIVE;
//     };

//     class Literal : public Atom
//     {
//     public:
//         Literal(bool sign, const Atom& atom)
//             :sign(sign),Atom(atom){};
        
//         bool sign;
//     };

    
    
//     class Clause : public set<Literal> {

//         void add_atom__POSITIVE(const Atom& atom)
//         {insert(Literal(true,atom));};
//         void add_atom__NEGATIVE(const Atom& atom)
//         {insert(Literal(true,atom));};
//         void add_literal(const Literal& literal)
//         {insert(literal);};
//     };
    
//     class Problem : public set<Clause> {
//         void add_clause(const Clause& clause){insert(clause);};
//     };

    
    
//     class Disjunction
//     {
//     };

//     class Conjuncton
//     {
//     };

//     class Negation
//     {
//     };
    
//     class Universal_Quantification
//     {
//     };

//     class Existential_Quantification
//     {
//     };
}


#endif
