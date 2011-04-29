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
#include "planning_formula.hh"

using namespace Planning::Formula;
using namespace std;

      
std::ostream& Planning::Formula::Printing::operator<<(std::ostream&o,
                                                      const std::tr1::tuple<Subformula>& _bt)
{
    auto bt = std::tr1::get<0>(_bt);
    return std::operator<<(o, bt);
}

#define LOOP_AND_PRINT(DATA)                                    \
    for(auto p = DATA.begin(); p != DATA.end(); p++){           \
        std::operator<<(o, *p);                                 \
        o<<" ";                                                 \
    }                                                           \
        

std::ostream& Printing::operator<<(std::ostream&o,
                                   const std::tr1::tuple<Planning::Formula::Subformulae>& _sf)
{
    auto sf = std::tr1::get<0>(_sf);
    LOOP_AND_PRINT(sf);
    
    return o;
}

std::ostream& Printing::operator<<(std::ostream& o, const Constant_Arguments& ca)
{
    LOOP_AND_PRINT(ca);
    return o;
}

std::ostream& Printing::operator<<(std::ostream& o, const Argument_List& al)
{
    LOOP_AND_PRINT(al);
    
    return o;
}
                                    \
    
#define PRINTING_IMPLEMENTATION(STRING, TYPENAME)            \
    ostream& TYPENAME::operator<<(ostream&o) const           \
    {                                                        \
        o<<"("<<STRING<<" ";                                 \
        Printing::operator<<(o, this->contents());           \
        o<<")"<<std::endl;                                   \
        return o;                                            \
    }                                                        \
    
PRINTING_IMPLEMENTATION("and", Conjunction)
PRINTING_IMPLEMENTATION("or", Disjunction)
PRINTING_IMPLEMENTATION("not", Negation)



const Planning::Variable& Forall::get__variable() const
{
    return tr1::get<0>(contents());
}

Planning::Variable Forall::get__variable()
{
    return tr1::get<0>(contents());
}

const Planning::Type& Forall::get__variable_type() const
{
    return tr1::get<1>(contents());
}
Planning::Type Forall::get__variable_type()
{
    return tr1::get<1>(contents());
}


const Planning::Variable& Exists::get__variable() const
{
    return tr1::get<0>(contents());
}
Planning::Variable Exists::get__variable()
{
    return tr1::get<0>(contents());
}

const Planning::Type& Exists::get__variable_type() const
{
    return tr1::get<1>(contents());
}

Planning::Type Exists::get__variable_type()
{
    return tr1::get<1>(contents());
}

const Subformula& Exists::get__subformula() const
{
    return tr1::get<2>(contents());
}

Subformula Exists::get__subformula()
{
    return tr1::get<2>(contents());
}

const Subformula& Forall::get__subformula() const
{
    return tr1::get<2>(contents());
}

Subformula Forall::get__subformula()
{
    return tr1::get<2>(contents());
}


Subformula Negation::get__subformula() 
{
    return tr1::get<0>(contents());
}

const Subformula& Negation::get__subformula() const
{
    return tr1::get<0>(contents());
}

const Subformulae& Disjunction::get__subformulae() const
{
    return tr1::get<0>(contents());
}

Subformulae Disjunction::get__subformulae()
{
    return tr1::get<0>(contents());
}


const Subformulae& Conjunction::get__subformulae() const
{
    return tr1::get<0>(contents());
}

Subformulae Conjunction::get__subformulae()
{
    VERBOSER(3000, "Getting the subformulae from a conjunction.");
    
    return tr1::get<0>(contents());
}




double Number::get__value() const {return std::tr1::get<0>(this->contents());}

Planning::Formula::Subformulae Probabilistic::get__probabilities() const {return std::tr1::get<1>(this->contents());}
Planning::Formula::Subformulae Probabilistic::get__formulae() const {return std::tr1::get<0>(this->contents());}


bool Planning::Formula::Probabilistic::leq1() const
{
    if(sanity()) return true;

    double total = 0.0;
    const auto& probabilities = get__probabilities();
    for(auto p = probabilities.begin(); p != probabilities.end(); p++){
        assert(p->test_cast<Number>());
        auto prob = p->cxx_get<Number>()->get__value();
        if(!is_admissible_probability(prob)){
            return false;
        }
        
        total += prob;
    }
    
    if (total <= 1.0) return true;
    else return false;
}


bool Planning::Formula::Probabilistic::sanity() const
{
    INTERACTIVE_VERBOSER(true, 5000, "Checking sanity on :: "<<*this);
    assert(get__probabilities().size());
    
    const auto& probabilities = get__probabilities();
    for(auto p = probabilities.begin(); p != probabilities.end(); p++){
        const auto&  probability = *p;
        
        INTERACTIVE_VERBOSER(true, 5000, "Checking sanity on :: "<<probability);
        if((*p)->get__type_name() != enum_types::number) {
            return true;
        } else {
            INTERACTIVE_VERBOSER(true, 5000, "Reported not to be a number :: "<<probability);
        }
    }
    
    INTERACTIVE_VERBOSER(true, 5000, "The following specificaton is numerically ground :: "
                         <<probabilities);
    
    Are_Doubles_Close are_Doubles_Close(1e-9);

    double total = 0.0;
    for(auto p = probabilities.begin(); p != probabilities.end(); p++){
        assert(p->test_cast<Number>());
        auto prob = p->cxx_get<Number>()->get__value();
        if(!is_admissible_probability(prob)){
            return false;
        }
        
        total += prob;
    }

    if(are_Doubles_Close(total, 1.0)) return true;
    else return false;
}


std::ostream& Planning::Formula::Probabilistic::operator<<(ostream&o)const 
{
    QUERY_UNRECOVERABLE_ERROR(
        get__probabilities().size() != get__formulae().size()
        ,"Asked to print incomprehensible probabilistic formula.");

    o<<"(probabilistic "<<std::endl;
    
    for(auto i = 0; i < get__formulae().size(); i++){
        o<<"\t "<<get__probabilities()[i]<<" "<<get__formulae()[i]<<std::endl;
    }

    return o<<")"<<std::endl;
}

#define get__operator_type_as_string__IMPLEMENTATION(TYPE_ID, STRING)   \
    std::string Planning::Formula::TYPE_ID::                            \
    get__operator_type_as_string() const                                \
    {                                                                   \
        return STRING;                                                  \
    }                                                                   \



get__operator_type_as_string__IMPLEMENTATION(Increase, "increase")
get__operator_type_as_string__IMPLEMENTATION(Decrease, "decrease")
get__operator_type_as_string__IMPLEMENTATION(Assign, "assign")
get__operator_type_as_string__IMPLEMENTATION(Equality_Test, "=")

Subformula Planning::Formula::Conditional_Effect::get__condition() const
{return std::tr1::get<0>(this->contents()); }

Subformula Planning::Formula::Conditional_Effect::get__effect() const
{return std::tr1::get<1>(this->contents()); }



std::ostream& Planning::Formula::Conditional_Effect::operator<<(ostream&o)const 
{
    o<<"(when "<<std::tr1::get<0>(this->contents())<<" "<<std::tr1::get<1>(this->contents());

    return o<<")"<<std::endl;
}


std::ostream& Planning::Formula::Vacuous::operator<<(ostream&o)const 
{
    o<<"(  )"<<std::endl;
    
    return o;
}


std::ostream& Planning::Formula::True::operator<<(ostream&o)const 
{
    o<<"(TRUE)"<<std::endl;
    
    return o;
}

std::ostream& Planning::Formula::False::operator<<(ostream&o)const 
{
    o<<"(FALSE)"<<std::endl;
    
    return o;
}


