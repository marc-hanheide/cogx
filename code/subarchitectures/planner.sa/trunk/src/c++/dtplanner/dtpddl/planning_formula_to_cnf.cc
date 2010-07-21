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


#include "planning_formula_to_cnf.hh"

using namespace Planning;

Planning_Formula__to__CNF::Subformula Planning_Formula__to__CNF::operator()(const Conjunction& conjunction)
{
    Subformulae new_subformulae;
    
    auto subformulae = conjunction.get__subformulae();
    
    for(auto subformula = subformulae.begin()
            ; subformula != subformulae.end()
            ; subformula++){
        auto answer = (*this)(*subformula);

        /* the \local{answer} should be a conjunct.*/
        auto conjunct = *(answer.do_cast<Conjunction>());

        /* and a conjunct must have a bunch of elements. These should
         * either be disjunctions over atoms, or otherwise atoms.*/
        auto conjunctive_elements = conjunct.get__subformulae();
        for(auto element = conjunctive_elements.begin()
                ; element != conjunctive_elements.end()
                ; element++){
            /* the shape of these is subformuale assumed by the
             * correctness of my code ;) Which I have assumed also
             * ;;) like a Madona. */
            new_subformulae.push_back(*element);
        }
    }

    
    NEW_referenced_WRAPPED_deref_POINTER
        (new_subformulae.back().get()->get__runtime_Thread()
         , Planning::Formula::Conjunction
         , tmp
         , new_subformulae);
    
    return tmp;
}

Planning_Formula__to__CNF::Subformulae&
Planning_Formula__to__CNF::distributive_law(Subformulae& conjunct,
                                            Subformulae disjunct,
                                            const std::vector<Conjunction*>& data,
                                            uint index)
{
    if(index == data.size()) {
        assert(disjunct.size());
        
        NEW_referenced_WRAPPED_deref_POINTER
            (disjunct.back().get()->get__runtime_Thread()
             , Planning::Formula::Disjunction
             , disjunctive_formula
             , disjunct);
        
        conjunct.push_back(disjunctive_formula);
        
        return conjunct;
    }
    
    auto input_conjunction = *data[index];
    auto subformulae = input_conjunction.get__subformulae();

    
    for(auto subformula = subformulae.begin()
            ; subformula != subformulae.end()
            ; subformula++){
        if(subformula->test_cast<Disjunction>()){
            auto long_or = subformula->do_cast<Disjunction>();
            auto atoms = long_or->get__subformulae();
            
            for(auto atom = atoms.begin()
                    ; atom != atoms.end()
                    ; atom++){
                disjunct.push_back(*atom);
            }
        } else {
            disjunct.push_back(*subformula);
        }
        
        
        distributive_law(conjunct, disjunct, data, index + 1);
    }
    
    return conjunct;
}


Planning_Formula__to__CNF::Subformula Planning_Formula__to__CNF::operator()(const Disjunction& disjunction)
{
    
    std::vector<Conjunction*> conjuncts;
    
    auto subformulae = disjunction.get__subformulae();


    for(auto subformula = subformulae.begin()
            ; subformula != subformulae.end()
            ; subformula++){
        auto answer = (*this)(*subformula);
        
        /* the \local{answer} should be a conjunct.*/
        auto conjunct = answer.do_cast<Conjunction>();
        conjuncts.push_back(conjunct);
    }


    assert(conjuncts.size());
    
    Subformulae result;
    distributive_law(result,
                     Subformulae(),
                     conjuncts,
                     0);

    
    NEW_referenced_WRAPPED_deref_POINTER
        (result.back().get()->get__runtime_Thread()
         , Planning::Formula::Conjunction
         , conjunctive_formula
         , result);
    
    return conjunctive_formula;
}


Planning_Formula__to__CNF::Subformula
Planning_Formula__to__CNF::operator()(Subformula _input, bool computed__nnf_input)
{
    auto input = _input;
    if(!computed__nnf_input){
        input = planning_Formula__to__NNF(_input);
    }
    
    switch(input.get()->get__type_name()){//get__id()){
        case vacuous:
        {
            WARNING("Asked to convert \"vacuous\" formula into CNF.");
            return _input;
        }
        break;
        case conjunction:
        {
            auto tmp = input.do_cast<Conjunction>();
            return (*this)(*tmp);
        }
        break;
        case disjunction:
        {
            auto tmp = input.do_cast<Disjunction>();
            return (*this)(*tmp);
        }
        break;
        case exists:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<_input<<std::endl);
        }
        break;
        case forall:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates."<<std::endl
                                <<"Problematic formula is :: "<<_input<<std::endl);
        }
        break;
        default:
        {
            VERBOSER(3000, "In converting to CNF Formula :: "<<_input<<std::endl
                     <<"is considered to be an atom, and is being added to a conjunctive element."<<std::endl);
        }
            break;
    }


    /*The result of CNF conversion is always a conjunct.*/
    Subformulae _new_subformulae;
    _new_subformulae.push_back(input);
    NEW_referenced_WRAPPED_deref_POINTER
        (input.get()->get__runtime_Thread()
         , Planning::Formula::Disjunction
         , new_disjunct
         , _new_subformulae);
    Subformulae new_subformulae;
    new_subformulae.push_back(new_disjunct);
    NEW_referenced_WRAPPED_deref_POINTER
        (input.get()->get__runtime_Thread()
         , Planning::Formula::Conjunction
         , tmp
         , new_subformulae);
    
    return tmp;
}
