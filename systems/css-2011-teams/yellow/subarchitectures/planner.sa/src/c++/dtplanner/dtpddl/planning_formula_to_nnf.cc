
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

#include "planning_formula_to_nnf.hh"


using namespace Planning;


Planning_Formula__to__NNF::Subformula Planning_Formula__to__NNF::operator()(Subformula input)
{
    if(!input.use_count()) return input;

    return (*this)(input, false);
}


Planning_Formula__to__NNF::Subformula Planning_Formula__to__NNF::operator()(
    Subformula input,
    bool carrying_negation)
{
    VERBOSER(3000, "Turning :: "<<input<<std::endl
             <<"into negation normal form. "<<std::endl);
    
    switch(input->get__type_name()){//get__id()){
        case vacuous:
        {
            WARNING("Asked to convert \"vacuous\" formula into NNF.");
            return input;
        }
        break;
        case negation:
        {
            auto tmp = input.do_cast<Negation>();
            return (*this)(*tmp, carrying_negation);
        }
        break;
        case conjunction:
        {
            auto tmp = input.do_cast<Conjunction>();
            return (*this)(*tmp, carrying_negation);
        }
        break;
        case disjunction:
        {
            auto tmp = input.do_cast<Disjunction>();
            return (*this)(*tmp, carrying_negation);
        }
        break;
        case exists:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates.");
        }
        break;
        case forall:
        {
            UNRECOVERABLE_ERROR("All quantifiers should have been removed during parsing, "<<std::endl
                                <<"and replaced by derived predicates.");
        }
        break;
        default:
        {
            if(carrying_negation){
                NEW_referenced_WRAPPED_deref_visitable_POINTER
                    (input->get__runtime_Thread()
                     , Planning::Formula::Negation
                     , tmp
                     , input);

                return tmp;
            }   
        }
            break;
    }
    
    return input;
}

Planning_Formula__to__NNF::Subformula Planning_Formula__to__NNF::operator()(Negation& formula, bool carrying_negation)
{
    carrying_negation = !carrying_negation;

    auto subformula = formula.get__subformula();

    return (*this)(subformula, carrying_negation);
}

Planning_Formula__to__NNF::Subformula Planning_Formula__to__NNF::operator()(Conjunction& formula, bool carrying_negation)
{
    Subformulae new_subformulae;
    
    auto subformulae = formula.get__subformulae();

    /*Got an empty conjunction in the domain specification, treating this as satisfied.*/
    if(!subformulae.size()){
        WARNING("Got an empty conjunction in the domain specification, treating this formula as always satisfied.");
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (formula.get__runtime_Thread()
             , Planning::Formula::Vacuous
             , tmp
             , static_cast<void*>(0));

        return tmp;
    }

    
    
    for(auto subformula = subformulae.begin()
            ; subformula != subformulae.end()
            ; subformula++){
        auto tmp = (*this)(*subformula, carrying_negation);
        new_subformulae.push_back(tmp);
    }

    
    
    assert(new_subformulae.size());
    
    if(carrying_negation){
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (new_subformulae.back()->get__runtime_Thread()
             , Planning::Formula::Disjunction
             , tmp
             , new_subformulae);

        return tmp;
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (new_subformulae.back()->get__runtime_Thread()
             , Planning::Formula::Conjunction
             , tmp
             , new_subformulae);
        return tmp;
    }
}

Planning_Formula__to__NNF::Subformula Planning_Formula__to__NNF::operator()(Disjunction& formula, bool carrying_negation)
{
    
    Subformulae new_subformulae;
    
    auto subformulae = formula.get__subformulae();

    for(auto subformula = subformulae.begin()
            ; subformula != subformulae.end()
            ; subformula++){
        auto tmp = (*this)(*subformula, carrying_negation);
        new_subformulae.push_back(tmp);
    }

    assert(new_subformulae.size());
    
    if(!carrying_negation){
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (new_subformulae.back()->get__runtime_Thread()
             , Planning::Formula::Disjunction
             , tmp
             , new_subformulae);

        return tmp;
    } else {
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            ( new_subformulae.back()->get__runtime_Thread()
              , Planning::Formula::Conjunction
              , tmp
              , new_subformulae);
        return tmp;
    }
}
