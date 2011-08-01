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

#include "turnstyle.hh"

using namespace Turnstyle;

CNF::CNF(Problem_Data& _problem_Data)
{
    satisfiability = unknown;
    
    problem_Data = _problem_Data;

    if(!check__no_empty_clause())satisfiability = no;
    
    bool different = false;
    do{
        auto  problem_Data__tmp = simplify__unit_prop(problem_Data);
        different = (problem_Data__tmp != problem_Data);
        if(different)problem_Data = problem_Data__tmp;
        
        if(!check__no_empty_clause()){satisfiability = no;return;};
    }while(different);

    problem_Data = simplify__subsumption(problem_Data);
    problem_Data = simplify__remove_A_and_NOT_A(problem_Data);
}

bool CNF::clause_trivial(const Clause& A) const
{
    for(auto p = A.begin(); p != A.end(); p++){
        if(A.find(-(*p)) != A.end()) return true;
    }

    return false;
}

CNF::Problem_Data  CNF::simplify__remove_A_and_NOT_A(Problem_Data& _problem_Data)
{
    Problem_Data problem_Data;
    for(auto p = _problem_Data.begin(); p != _problem_Data.end(); p++){
        if(!clause_trivial(*p))problem_Data.insert(*p);
    }

    return std::move(problem_Data);
}


CNF::Problem_Data CNF::collect__size_equals(Problem_Data& _problem_Data, unsigned int n)
{
    Problem_Data answer;
    for(auto p = _problem_Data.begin()
            ;  p != _problem_Data.end()
            ; p++){
        if(p->size() == n){
            answer.insert(*p);
        }
    }

    return std::move(answer);
}


CNF::Problem_Data CNF::collect__size_greater_than(Problem_Data& _problem_Data, unsigned int n)
{
    
    Problem_Data answer;
    for(auto p = _problem_Data.begin()
            ;  p != _problem_Data.end()
            ; p++){
        if(p->size() > n){
            answer.insert(*p);
        }
    }

    return std::move(answer);
}

bool CNF::satisfiable_conjunct(Problem_Data& conjunct) const
{
    Problem_Data tmp;
    for(auto p = conjunct.begin(); p != conjunct.end(); p++){

        QUERY_WARNING((p->size() != 1), "Asked to evaluate a conjunct of literals,"<<std::endl
                      <<"but got a conjunct with a disjunctive element :: "<<std::endl
                      <<conjunct<<std::endl
                      <<"RECOVERY is skipping disjunctive entry.");
        /*RECOVERY(CONTINUE)*/if(p->size() != 1){tmp.insert(*p); continue;}/*RECOVERY*/
        
        
        Clause clause;
        clause.insert(abs(*(p->begin()))); // <cstdlib>
        tmp.insert(clause);
    }

    if(tmp.size() == conjunct.size()){
        return true;
    } else {
        return false;
    }
    
}

int CNF::get_length_of_longest_clause(Problem_Data& problem) const
{
    int size = 0; 
    for(auto p = problem.begin(); p != problem.end(); p++){
        if(p->size() > static_cast<uint>(size)) size = static_cast<int>(p->size());
    }

    return size;
}

bool CNF::clause_subsumes(const Clause& smaller, const Clause& bigger) const
{
    std::vector<Literal> intersection(std::min(smaller.size(), bigger.size()));
    
    auto it = set_intersection(smaller.begin(), smaller.end(), bigger.begin(), bigger.end(), intersection.begin());

    return (it == intersection.end());
}


bool CNF::check__no_empty_clause()
{
    Problem_Data problem_Data__empty = collect__size_equals(problem_Data, 0);

    return !(problem_Data__empty.size());
}

CNF::Problem_Data CNF::simplify__unit_prop(Problem_Data& _problem_Data)
{
    std::vector<Clause> problem_Data(_problem_Data.begin(), _problem_Data.end());
    
    /*Collect unary problem fragment, and test satisfiability.*/
    Problem_Data problem_Data__unary = collect__size_equals(_problem_Data, 1);
    bool unsat = !satisfiable_conjunct(problem_Data__unary);
    /*RETURN(IF)*/if(unsat){return Problem_Data();}

    for(auto clause = problem_Data.begin(); clause != problem_Data.end(); clause++){
        if(clause->size() <= 1) continue;
        for(auto unit = problem_Data__unary.begin(); unit != problem_Data__unary.end(); unit++){
            Literal literal =  -1*(*unit->begin()); 
            clause->erase(literal);
            if(clause->size() == 0){return Problem_Data();}
        }
    }

    return Problem_Data(problem_Data.begin(), problem_Data.end());
}

CNF::Problem_Data CNF::simplify__subsumption(Problem_Data& _problem_Data)
{
//     int longest_clause_length = get_length_of_longest_clause(_problem_Data);
    
    Problem_Data problem_Data = _problem_Data;
//     bool something_happened = false;
//     bool unsat = false;
    
    for(auto i = 1; i < get_length_of_longest_clause(problem_Data); i++){
        Problem_Data problem_Data__clauses_that_could_subsume = collect__size_equals(problem_Data, i);
        Problem_Data problem_Data__clauses_that_could_be_subsumed = collect__size_greater_than(problem_Data, i);

        Problem_Data clauses_to_delete;
        for(auto subsumes = problem_Data__clauses_that_could_subsume.begin()
                ; subsumes != problem_Data__clauses_that_could_subsume.end()
                ; subsumes++){
            for(auto subsumed = problem_Data__clauses_that_could_be_subsumed.begin()
                ; subsumed != problem_Data__clauses_that_could_be_subsumed.end()
                ; subsumed++){
                if(clauses_to_delete.find(*subsumed) == clauses_to_delete.end()){
                    if(clause_subsumes(*subsumes, *subsumed)){
                        clauses_to_delete.insert(*subsumed);
                    }
                }
            }
        }

        for(auto clause = clauses_to_delete.begin()
                ; clause != clauses_to_delete.end()
                ; clause++){
            problem_Data.erase(*clause);
        }
    }

    return std::move(problem_Data);
}

namespace Turnstyle
{
    void test__turnstyle_hh()
    {
        
        using Turnstyle::CNF;
        CNF::Clause c1;c1.insert(1);c1.insert(10);c1.insert(-10);
        CNF::Clause c2;c2.insert(2);
        CNF::Clause c3;c3.insert(3);c3.insert(-2);
        CNF::Clause c4;c4.insert(4);c4.insert(-3);c4.insert(5);
        
        
        CNF::Problem_Data pd;
        pd.insert(c1);pd.insert(c2);pd.insert(c3);pd.insert(c4);
    
        CNF cnf(pd);

        std::cout<<cnf<<std::endl;
    }
}



std::ostream& std::operator<<(std::ostream&o, const Turnstyle::CNF::Problem_Data& problem_Data)
{
    o<<"CNF with N = "<<problem_Data.size()<<" clauses."<<std::endl;
    
    for(auto p = problem_Data.begin(); p != problem_Data.end(); p++){
        o<<*p<<std::endl;
    }
    
    
    return o;
}



std::ostream& std::operator<<(std::ostream&o, const Turnstyle::CNF::Clause& clause)
{
    for(auto p = clause.begin(); p != clause.end(); p++){
        o<<*p<<", ";
    }
    
    return o;  
}

std::ostream& std::operator<<(std::ostream&o, const Turnstyle::CNF& problem)
{
    return o<<problem.problem_Data;  
}


// void CNF::add__searchable__literal_in_clause(const Clause& clause)
// {
// }

// void CNF::add__searchable__atom_in_clause(const Clause& clause)
// {
// }

// void CNF::add__searchable__atom_in_clause__POSITIVE(const Clause& clause)
// {
// }

// void CNF::add__searchable__atom_in_clause__NEGATIVE(const Clause& clause)
// {
// }


// bool CNF::failure_test__empty_clause_on_construction(const Searchable__Clause& clause)
// {
//     return (potential_clause.size() == 0);
// }

// bool CNF::failure_test__unit_clause_negation_of_existing_clause(const Searchable__Clause& new_clause)
// {
//         if(new_clause.size() == 1){
//             Searchable__Clause negate_to_insert;
//             negate_to_insert.insert(-1 * (*new_clause.begin()));
            
//             if(searchable__problem_data.find(negate_to_insert) != searchable__problem_data.end()){
//                 satisfiability = no;
//                 return;
//             }
//         }
// }

// bool CNF::redundant_clause_test(const Searchable__Clause& new_clause)
// {
//     if(searchable__problem_data.find(potential_clause) != searchable__problem_data.end()) return true;

    
    
//     searchable__problem_data.insert(potential_clause);
// }


// CNF::CNF(const Problem_Data& problem_Data)
// {
//     for(auto clause = problem_Data.begin()
//             ; clause != problem_Data.end()
//             ; clause++){

//         Searchable__Clause potential_clause;
        
//         for(auto literal = clause->begin()
//                 ; literal != clause->end()
//                 ; literal ++){
//             potential_clause.push_back(*literal);
//         }

        
//         if(failure_test__empty_clause_on_construction(potential_clause))
//         {satisfiability = no; return;};
        
//         if(failure_test__unit_clause_negation_of_existing_clause(potential_clause))
//         {satisfiability = no; return;};
        
//         if(redundant_clause_test(potential_clause))continue;

        
        
//         Clause new_clause(potential_clause.begin(), potential_clause.end());
        
//         for(auto literal = new_clause.begin()
//                 ; literal != new_clause.end()
//                 ; literal ++){
//             if(searchable__literal_in_clause.find(*literal)
//                == searchable__literal_in_clause.end())
//                 searchable__literal_in_clause[*literal] = set<int>();
            
//             searchable__literal_in_clause[*literal]
//                 .insert(problem_Data.size());

//             Atom atom = abs(*literal);
            
//             if(searchable__atom_in_clause.find(atom)
//                == searchable__atom_in_clause.end())
//                 searchable__atom_in_clause[atom] = set<int>();
//             searchable__atom_in_clause[atom]
//                 .insert(problem_Data.size());
            

//             if(*literal > 0){
//                 if(searchable__atom_in_clause__POSITIVE.find(atom)
//                    == searchable__atom_in_clause__POSITIVE.end())
//                     searchable__atom_in_clause__POSITIVE[atom] = set<int>();

//                 searchable__atom_in_clause__POSITIVE[atom]
//                     .insert(problem_Data.size());
//             } else {    
//                 if(searchable__atom_in_clause__NEGATIVE.find(atom)
//                    == searchable__atom_in_clause__NEGATIVE.end())
//                     searchable__atom_in_clause__NEGATIVE[atom] = set<int>();
                
//                 searchable__atom_in_clause__NEGATIVE[atom]
//                     .insert(problem_Data.size());
//             }
//         }

//         if(new_clause.size() == 1){
//             searchable__unit_clauses.insert(problem_Data.size());
//         }
        
//         problem_Data.push_back(new_clause);

        
//     }
    
// }
