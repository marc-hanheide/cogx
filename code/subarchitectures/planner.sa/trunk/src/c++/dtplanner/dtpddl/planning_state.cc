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



#include "planning_state.hh"


#include "solver.hh"
#include "planning_formula.hh"
#include "problem_grounding.hh"

#include "action__state_transformation.hh"
#include "observation.hh"
#include "observation__probabilistic.hh"


using namespace Planning;


State::State(Solver& solver,
             uint propositions_count,
             uint function_count,
             uint formulae_count,
             uint disjunctions_count,
             uint actions_count,
             uint action_formulae_count,
             uint action_disjunctions_count)
    :solver(solver),
     Markov_Decision_Process_State(propositions_count
                                   , function_count),
     CNF__State(formulae_count
                , disjunctions_count
                , action_formulae_count
                , action_disjunctions_count),
     Action_Executability__State(actions_count)   
{   
}

uint State::count__probabilistic_observations() const
{
    return probabilistic_observations.size();
}

const Probabilistic_Observation* State::pop__probabilistic_observation()
{
    auto result = probabilistic_observations.top();
    probabilistic_observations.pop();
    return result;
}

void State::push__probabilistic_observation(const Probabilistic_Observation* in)
{
   probabilistic_observations.push(in); 
}


uint State::count__observations() const
{
    return observations.size();
}

const Observation* State::pop__observation()
{
    auto result = observations.top();
    observations.pop();
    return result;
}

void State::push__observation(const Observation* in)
{
    observations.push(in);
}


double State::set__probability_during_expansion(double in)
{
    probability_during_expansion = in;
}

void State::reset__probability_during_expansion()
{
    probability_during_expansion = 1.0;
}

double State::get__probability_during_expansion() const
{
    return probability_during_expansion;
}

uint State::count__compulsory_generative_transformations() const
{
    return applicable_compulsory_generative_transformations.size();
}

uint State::count__compulsory_transformations() const
{
    return applicable_compulsory_transformations.size();
}


const State_Transformation* State::pop__compulsory_generative_transformation()
{
    auto result = applicable_compulsory_generative_transformations.top();
    applicable_compulsory_generative_transformations.pop();
    return result;
}

const State_Transformation* State::pop__compulsory_transformation()
{
    auto result = applicable_compulsory_transformations.top();
    applicable_compulsory_transformations.pop();
    return result;
}


uint State::count__probabilistic_transformations() const
{
   return  probabilistic_transformations.size();
}

const Probabilistic_State_Transformation* State::pop__probabilistic_transformation()
{
    auto result = probabilistic_transformations.top();
    probabilistic_transformations.pop();
    return result;
}

void State::push__probabilistic_transformation
(const Probabilistic_State_Transformation* probabilistic_State_Transformation)
{
    probabilistic_transformations.push(probabilistic_State_Transformation);
}


void State::push__compulsory_generative_transformation(const State_Transformation*state_Transformation)
{
    applicable_compulsory_generative_transformations.push(state_Transformation);
}

void State::add__optional_transformation(const State_Transformation*state_Transformation)
{
    
    applicable_optional_transformations.insert(state_Transformation);
}

void State::remove__optional_transformation(const State_Transformation*state_Transformation)
{
    applicable_optional_transformations.erase(state_Transformation);
}

void State::push__compulsory_transformation(const State_Transformation*state_Transformation)
{
    applicable_compulsory_transformations.push(state_Transformation);
}

std::set<const State_Transformation*> State::get__optional_transformations()
{
    return applicable_optional_transformations;
}


std::ostream& State::operator<<(std::ostream&o) const
{
    auto problem_Grounding = solver.get__problem_Grounding();

    basic_type::Runtime_Thread runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Planning::Problem_Grounding*>(problem_Grounding.get()));

    
    INTERACTIVE_VERBOSER(true, 7000, "Printing an planning state with  :: "
                         <<this->get__number_of_atoms()
                         <<" propositions.");
    
    o<<"  :-ACTIONS-: {"<<std::endl;
    if(applicable_optional_transformations.size()){
        for(auto applicable_optional_transformation =applicable_optional_transformations.begin()
                ; applicable_optional_transformation != applicable_optional_transformations.end()
                ; applicable_optional_transformation++){
            o<<(*applicable_optional_transformation)->get__identifier()<<" :-: ";
        }
    } else {
        o<<"POSSIBLE-SINK-STATE; i.e,. if there are no zero/null-precondition actions."<<std::endl;
    }
    
    o<<"}"<<std::endl;
    
    o<<" Prob. during expansion :: "<<probability_during_expansion<<"  :-STATE-: {"<<std::endl;
    for(auto i = 0; i < this->get__number_of_atoms()/*Markov_Decision_Process_State::get__number_of_atoms()*/; i++){
        if(is_true(i)){

            QUERY_UNRECOVERABLE_ERROR(!Formula::State_Proposition::
                                      ith_exists(runtime_Thread, i)
                                      , "Could not find a ground symbol associated with index :: "<<i);
            
            auto symbol = Formula::State_Proposition::
                make_ith<Formula::State_Proposition>
                (runtime_Thread,
                 i);
            o<<symbol<<"; ";
        }
    }
    o<<"}"<<std::endl;


    return o;
}

namespace std
{
    std::ostream& operator<<(ostream& o, const Planning::State& in)
    {
        return in.operator<<(o);
    }
    
    std::size_t hash_value(const Planning::State& in)
    {
        return in.Markov_Decision_Process_State::hash_value();
    }
    
}

