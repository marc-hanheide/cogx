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

void State::decrement__obtainable_positive_rewards_count()
{
    INTERACTIVE_VERBOSER(true, 14000, "Positive rewards :: "<<obtainable_positive_rewards_count);

    if(obtainable_positive_rewards_count == 0){
        WARNING("Something is wrong with reward counting. Heuristic should not be using reward counts.");
        return ;
    }
    
    assert(obtainable_positive_rewards_count > 0);
    obtainable_positive_rewards_count--;
}

void State::increment__obtainable_positive_rewards_count()
{
    obtainable_positive_rewards_count++;
    INTERACTIVE_VERBOSER(true, 14000, "Positive rewards :: "<<obtainable_positive_rewards_count);
}

uint State::get__obtainable_positive_rewards_count() const
{
    
    return obtainable_positive_rewards_count;
}

void State::decrement__obtainable_rewards_value(int val)
{
    obtainable_rewards_value -= val;
}

void State::increment__obtainable_rewards_value(int val)
{
    obtainable_rewards_value += val;
}

int State::get__obtainable_rewards_value() const
{
    return obtainable_rewards_value;
}


void State::decrement__obtainable_rewards_count()
{
    if(!obtainable_rewards_count) {

        WARNING("Something is wrong with reward counting. Heuristic should not be using reward counts.");
        
        return;
    }
    
    
    assert(obtainable_rewards_count);
    obtainable_rewards_count--;
}

void State::increment__obtainable_rewards_count()
{
    obtainable_rewards_count++;
}

uint State::get__obtainable_rewards_count() const
{
    return obtainable_rewards_count;
}


bool State::operator==(const State&in) const
{
    return Markov_Decision_Process_State::operator==(in);
}

bool State::operator<(const State&in) const
{
    return Markov_Decision_Process_State::operator<(in);
}


State::State(Solver& solver,
             uint propositions_count,
             uint function_count,
             uint formulae_count,
             uint disjunctions_count,
             uint actions_count,
             uint action_formulae_count,
             uint action_disjunctions_count,
             uint observations_count)
    :Markov_Decision_Process_State(propositions_count
                                   , function_count),
     CNF__State(formulae_count
                , disjunctions_count
                , action_formulae_count
                , action_disjunctions_count),
     Action_Executability__State(actions_count),
     Observational__State(observations_count),
     solver(solver),
     observational_state_during_expansion(0),
     obtainable_rewards_count(0),
     obtainable_positive_rewards_count(0),
     obtainable_rewards_value(0)
{
    
    INTERACTIVE_VERBOSER(true, 8060, "Observation state has size :: "
                         <<Observational__State::observations_count()<<std::endl);
}

Observational_State* State::get__observational_state_during_expansion() const
{
    return observational_state_during_expansion;
}

void State::set__observational_state_during_expansion(Observational_State* in)
{
    observational_state_during_expansion = in;
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
        for(auto applicable_optional_transformation = applicable_optional_transformations.begin()
                ; applicable_optional_transformation != applicable_optional_transformations.end()
                ; applicable_optional_transformation++){
//             Formula::State_Proposition::
//                 ith_exists(runtime_Thread, (*applicable_optional_transformation)->get__id());
            
            o<<(*applicable_optional_transformation)->get__id()<<" "
             <<(*applicable_optional_transformation)->get__identifier()<<" :-: ";
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

    o<<"VALUE :: "<<this->get__reward()<<std::endl;

    o<<"POTENTIAL ELEMENTS VALUE :: "<<this->get__obtainable_rewards_count()<<std::endl;
    o<<"ESTIMATED POTENTIAL VALUE :: "<<this->get__obtainable_rewards_value()<<std::endl;

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

