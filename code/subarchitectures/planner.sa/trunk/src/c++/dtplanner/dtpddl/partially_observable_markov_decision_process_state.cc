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


#include "partially_observable_markov_decision_process_state.hh"

#include "planning_state.hh"
#include "solver.hh"

namespace Planning
{
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State& in)
    {
        return in.hash_value();
    }
}


using namespace Planning;

bool Partially_Observable_Markov_Decision_Process_State::
useless() const
{
    for(auto _mdp_state = belief_State.begin()
            ; _mdp_state != belief_State.end()
            ; _mdp_state++){
        auto mdp_state = _mdp_state->first;
        assert(dynamic_cast<Planning::State*>(mdp_state));
        if(dynamic_cast<Planning::State*>(mdp_state)
           ->get__obtainable_rewards_count() != 0){
            return false;
        }        
    }

    return true;
}


const std::vector<Observational_State*>&
Partially_Observable_Markov_Decision_Process_State::
get__possible_observations_given_action(uint action_index) const
{
   
    QUERY_UNRECOVERABLE_ERROR
        (!successors.size()
         , "No successors at belief-state. "<<std::endl);
    
    uint index_to_actions = 0;
    while(action_index != action_based_successor_driver[index_to_actions]){
        index_to_actions++;

        std::string debug_string;
        if(index_to_actions >= action_based_successor_driver.size()){
            std::ostringstream oss;
            oss<<"Failing at a belief state "<<*this;

            for(auto atom = belief_State.begin()
                    ; atom != belief_State.end()
                    ; atom++){
                auto _state = atom->first;

                Planning::State* state = dynamic_cast<Planning::State*>(_state);

                QUERY_UNRECOVERABLE_ERROR
                    (!state
                     , "Invalid state at belief atom. "<<std::endl);

                oss<<*state<<std::endl<<std::endl;
            }
            debug_string = oss.str();
        }
        
        
        QUERY_UNRECOVERABLE_ERROR
            (index_to_actions >= action_based_successor_driver.size()
             , "Unregistered action :: "<<action_index<<" at belief-state. "<<std::endl
             <<debug_string<<std::endl);
    }

    return observation_based_successor_driver[index_to_actions];
}

Partially_Observable_Markov_Decision_Process_State::POMDP_State*
Partially_Observable_Markov_Decision_Process_State::
get__successor(uint action_index
               , Observational_State* observational_State)
{
    QUERY_UNRECOVERABLE_ERROR
        (!successors.size()
         , "No successors at belief-state. "<<std::endl);
    
    uint index_to_actions = 0;
    while(action_index != action_based_successor_driver[index_to_actions]){
        index_to_actions++;
        QUERY_UNRECOVERABLE_ERROR
            (index_to_actions >= action_based_successor_driver.size()
             , "Unregistered action at belief-state. "<<std::endl);
    }
    
    auto& _observation_based_successor_driver
        = observation_based_successor_driver[index_to_actions];

    uint index_to_observations = 0;
    while(observational_State !=
          _observation_based_successor_driver[index_to_observations]){
        index_to_observations++;
        
        QUERY_UNRECOVERABLE_ERROR
            (index_to_observations >= _observation_based_successor_driver.size()
             , "Unregistered observation at belief-state. "<<std::endl);
    }

    assert(index_to_actions < successors.size());
    assert(index_to_observations < successors[index_to_actions].size());
    return successors[index_to_actions][index_to_observations];
}

Partially_Observable_Markov_Decision_Process_State::
Partially_Observable_Markov_Decision_Process_State()
    :Expandable()
//     :expected_value(0.0)
{
    expected_value = 0.0;
    expected_reward = 0.0;
    belief_State = Belief_State(0);
}


bool Partially_Observable_Markov_Decision_Process_State::
operator==(const POMDP_State& in) const
{
    INTERACTIVE_VERBOSER(true, 9085, "Comparing :: "<<in<<" "<<*this<<std::endl);
    
    INTERACTIVE_VERBOSER(true, 9081, "Comparing :: "<<in.expected_value);
    INTERACTIVE_VERBOSER(true, 9081, "Comparing :: "<<this->expected_reward<<std::endl);
    
    if(in.expected_reward == this->expected_reward){
        if(in.belief_State.size() == this->belief_State.size()){
            if(in.belief_State == this->belief_State){
                return true;
            }  
        }
    }

    return false;
    
//     return ( (in.expected_value == expected_value) &&
//              (in.belief_State.size() == belief_State.size()) &&
//              (in.belief_State == belief_State) );
}

bool Partially_Observable_Markov_Decision_Process_State::
operator<(const POMDP_State& in) const
{
    if(this->expected_reward < in.expected_reward){
        return true;
    } else if (this->expected_reward == in.expected_reward) {

        if(belief_State.size() < in.belief_State.size()){
        } else if (belief_State.size() == in.belief_State.size()) {
            auto atom_lhs = belief_State.begin();
            auto atom_rhs = in.belief_State.begin();
            for(
                    ; atom_lhs != belief_State.end()
                    ; atom_lhs++, atom_rhs++){
                if(atom_lhs->second < atom_rhs->second){
                    return true;
                } else if (atom_lhs->second == atom_rhs->second) {
                    if(static_cast< void*>(atom_lhs->first) <
                       static_cast< void*>(atom_rhs->first)){
                    } else if (atom_lhs->first == atom_rhs->first) {
                        continue;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
    }

    return false;
}

void Partially_Observable_Markov_Decision_Process_State::
push__successor(uint action_index
                , Observational_State* observation
                , POMDP_State* successor_pomdp_state
                , double probability)
{
    
    INTERACTIVE_VERBOSER(true, 10060, "For state :: "<<this<<std::endl
                         <<"Adding :: "<<*observation<<std::endl
                         <<"At the execution of action :: "<<action_index<<std::endl
                         <<"With probability :: "<<probability<<std::endl);
    
    /* IF --- Either this is the first action that we have considered
     * executing from this belief-state, or otherwise we are
     * considering a new action in sequence.*/
    if(!action_based_successor_driver.size() ||
       action_based_successor_driver.back() != action_index){
        
        action_based_successor_driver.push_back(action_index);

#ifndef NDEBUG
        if(observation_probabilities.size()){/*DEBUG SANITY*/
            auto probs = observation_probabilities.back();
            double sum = 0.0;
            for(auto prob = probs.begin()
                    ; prob != probs.end()
                    ; prob++){
                sum += *prob;
            }
            assert(Solver::are_Doubles_Close(1.0, sum));
        }
#endif
        
        observation_based_successor_driver
            .push_back(std::vector<Observational_State*>());
        successors.push_back(
            std::vector< Partially_Observable_Markov_Decision_Process_State*>());
        
        observation_probabilities.push_back(
            std::vector<double>()
            );
    }

    /* So we are now considering observational entries that correspond
     * to observations received when we execute the action at
     * \member{action_based_successor_driver.back()}. */
    
    auto& observations = observation_based_successor_driver.back();
    auto& _successors = successors.back();
    auto& observation_probability = observation_probabilities.back();
    
    
    /* At this point we suppose that the observation is the first one
     * that we have considered for the obove action execution, or
     * otherwise that we have not considered it before.*/
    if(!observations.size() || observations.back() != observation){
        observations.push_back(observation);
        observation_probability.push_back(probability);
        _successors.push_back(successor_pomdp_state);// std::vector<
//                        Partially_Observable_Markov_Decision_Process_State*>());
    } else {
        QUERY_UNRECOVERABLE_ERROR(true, "We have a belief-state :: "<<*this<<std::endl
                                  <<"This has successor information for :: "
                                  <<observations.size()<<" observations."<<std::endl
                                  <<"The last observation is :: "<<*observations.back()<<" @"
                                  <<observations.back()<<std::endl
                                  <<"The observation we are adding is :: "<<*observation<<" @"
                                  <<observation<<std::endl);
    }
    

//     auto& successors_states = _successors.back();
//     successors_states.push_back(successor_pomdp_state);
}

void Partially_Observable_Markov_Decision_Process_State::initialise__prescribed_action_index()
{
    if(action_based_successor_driver.size() == 0){
        prescribed_action_index = 0;
        return;
    }
    
    prescribed_action_index = random() % action_based_successor_driver.size();
}

uint Partially_Observable_Markov_Decision_Process_State::get__prescribed_action() const
{
    if(!action_based_successor_driver.size()){
        /* It wasn't really unexpanded. This is a problem because some
         * states will not expand to anything. We need to sometimes
         * include a NULL action.*/
        assert(unexpanded());
        QUERY_UNRECOVERABLE_ERROR(unexpanded(),
                      "We do not seem to be able to expand POMDP state :: "
                      <<*this<<std::endl
                      <<"I suspect this is a sink state, but if not we have a more serious bug on our hands.\n"
                      <<"Because you didn't model an action that spins on this state, I assume you want things\n"
                      <<"To fail horribly at this state.\n"
                      <<"Now you have asked me to prescribe an action for this state.\n"
                      <<"Bad moove brother...\n");

        return 0;
        
    }
    
    
    assert(action_based_successor_driver.size());
    assert(prescribed_action_index <  action_based_successor_driver.size());
    return action_based_successor_driver[prescribed_action_index];
}



// std::pair<std::vector<Observational_State*>*
//           , std::vector<double>*  >
// Partially_Observable_Markov_Decision_Process_State::get_successor_information_at_prescribed_action()
// {
//     std::pair<std::vector<Observational_State*>*
//           , std::vector<double>*  > result;

//     result.first = &observation_based_successor_driver[get__prescribed_action()];
//     result.second = &observation_probabilities[get__prescribed_action()];
    
//     return result;
// }

const std::vector<Observational_State*>&
Partially_Observable_Markov_Decision_Process_State::
get_observations_at_prescribed_action() const
{
    assert(prescribed_action_index < observation_based_successor_driver.size());
    return observation_based_successor_driver[prescribed_action_index];//get__prescribed_action()];
}

const std::vector<double>&
Partially_Observable_Markov_Decision_Process_State::
get_observation_probabilities_at_prescribed_action() const
{
    assert(prescribed_action_index <  observation_probabilities.size());
    return observation_probabilities[prescribed_action_index];//get__prescribed_action()];
}

void
Partially_Observable_Markov_Decision_Process_State::
accept_values(boost::numeric::ublas::compressed_vector< double >& values)
{
   
//     bool assigned_score = false;
    double best_score = 1e-100;
    
    assert(successors.size() == action_based_successor_driver.size());
    assert(successors.size() == observation_based_successor_driver.size());
    assert(successors.size() == observation_probabilities.size());
    
    for( uint driver_index = 0
             ; driver_index < successors.size()
             ; driver_index++){

        auto& driven_successors = successors[driver_index];
        auto& probabilities = observation_probabilities[driver_index];
//         auto& observations = observation_based_successor_driver[driver_index];

        assert(driven_successors.size() == probabilities.size());
//         assert(observations.size() == driven_successors.size());
        
        double local_score = 0.0;
        for(uint successor_index = 0
                ; successor_index < probabilities.size()
                ; successor_index++){
            auto successor = driven_successors[successor_index];
            auto probability = probabilities[successor_index];

            assert(successor->get__index() < values.size());
            local_score += probability * values(static_cast<int>(successor->get__index()));//[successor->get__index()];
        }
        
        if(local_score > best_score){
           prescribed_action_index = driver_index;
           best_score = local_score;
        }
    }

    expected_value = best_score;  
}


void 
Partially_Observable_Markov_Decision_Process_State::
accept_values(boost::numeric::ublas::vector<double>& values)
{
//     bool assigned_score = false;
    double best_score = 1e-100;
    
    assert(successors.size() == action_based_successor_driver.size());
    assert(successors.size() == observation_based_successor_driver.size());
    assert(successors.size() == observation_probabilities.size());
    
    for( uint driver_index = 0
             ; driver_index < successors.size()
             ; driver_index++){

        auto& driven_successors = successors[driver_index];
        auto& probabilities = observation_probabilities[driver_index];
//         auto& observations = observation_based_successor_driver[driver_index];

        assert(driven_successors.size() == probabilities.size());
//         assert(observations.size() == driven_successors.size());
        
        double local_score = 0.0;
        for(uint successor_index = 0
                ; successor_index < probabilities.size()
                ; successor_index++){
            auto successor = driven_successors[successor_index];
            auto probability = probabilities[successor_index];

            assert(successor->get__index() < values.size());
            local_score += probability * values[successor->get__index()];
        }
        
        if(local_score > best_score){
           prescribed_action_index = driver_index;
           best_score = local_score;
        }
    }

    expected_value = best_score; 
}

bool
Partially_Observable_Markov_Decision_Process_State::
unexpanded() const
{
    return (action_based_successor_driver.size() == 0);
}

const std::vector< Partially_Observable_Markov_Decision_Process_State*>&
Partially_Observable_Markov_Decision_Process_State::get_successors_at_prescribed_action() const
{
    assert(prescribed_action_index < successors.size());
    return successors[prescribed_action_index];//get__prescribed_action()];
}


uint Partially_Observable_Markov_Decision_Process_State::get__index() const
{
    return this->index;
}

void Partially_Observable_Markov_Decision_Process_State::set__index(uint in) 
{
    this->index = in;
}

double Partially_Observable_Markov_Decision_Process_State::get__expected_value() const
{
    return this->expected_value;
}

void Partially_Observable_Markov_Decision_Process_State::set__expected_value(double in) 
{
    this->expected_value = in;
}

double Partially_Observable_Markov_Decision_Process_State::get__expected_reward() const
{
    return this->expected_reward;
}

void Partially_Observable_Markov_Decision_Process_State::
add__belief_atom( MDP_State* mdp__state, double probability)
{
    belief_State.push_back(Belief_Atom(mdp__state, probability));

//     assert(this->expected_value >= 0.0);
//     assert(this->expected_value < 1.1);

    INTERACTIVE_VERBOSER(true, 9097, "Adding to expected value :: "
                         <<(probability * mdp__state->get__reward()));
    
    
    this->expected_reward += (probability * mdp__state->get__reward());
    this->expected_value =  this->expected_reward;   
}

const Partially_Observable_Markov_Decision_Process_State::
Belief_State&
Partially_Observable_Markov_Decision_Process_State::
get__belief_state() const
{
    return belief_State;
}


std::size_t  Partially_Observable_Markov_Decision_Process_State::
hash_value() const
{   
    INTERACTIVE_VERBOSER(true, 9085, "Hash value of :: "
                         <<*this<<" = "
                         <<boost::hash_range(belief_State.begin(), belief_State.end())
                         <<std::endl);
    return boost::hash_range(belief_State.begin(), belief_State.end());
}


namespace std
{
    std::size_t hash_value(const Planning::Partially_Observable_Markov_Decision_Process_State& in)
    {
        return in.hash_value();
    }
    
    std::ostream& operator<<(std::ostream&o,
                             const Planning::Partially_Observable_Markov_Decision_Process_State& pomdp__state)
    {
        auto state = pomdp__state.get__belief_state();


        o<<"Expected reward :: "<<pomdp__state.get__expected_reward()<<std::endl;
        for(auto s = state.begin(); s != state.end(); s++){
            o<<s->second<<" :--:> "<<*(s->first)<<std::endl;
        }

        for(auto s = state.begin(); s != state.end(); s++){
            INTERACTIVE_VERBOSER(true, 10017, "Expanded POMDP state :: "
                                 <<*dynamic_cast<const State*>(s->first)<<std::endl);
        }
        
        
        return o;
    }
    
    std::ostream& operator<<(std::ostream&o, const Planning::Partially_Observable_Markov_Decision_Process_State::Action__to__Observation_to_Belief& in)
    {
        for(auto actions = in.begin()
                ; actions != in.end()
                ; actions++){
            for(auto observations = actions->second.begin()
                    ; observations != actions->second.end()
                    ; observations++){
                for(auto states = observations->second.begin()
                        ; states != observations->second.end()
                        ; states++){
                    o<<actions->first<<" "
                     <<*observations->first<<" "
                     <<*states->first<<" === "
                     <<states->second<<std::endl;
                }
            }
        }
        
        return o;
    }
}
