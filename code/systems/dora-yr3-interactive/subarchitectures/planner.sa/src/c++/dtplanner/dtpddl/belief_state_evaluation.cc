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
 * CogX ::
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

#include "belief_state_evaluation.hh"
#include "planning_state.hh"

using namespace Planning;



float MDP_Heuristic::operator()(POMDP_State* state) const
{
    auto& belief = state->get__belief_state();
    uint possibly_obtainable_positive_rewards_count = 0;
    double expected_value = 0.0;
    for(auto _atom = belief.begin()
            ; _atom != belief.end()
            ; _atom++){
        double probability = _atom->second;

        assert(dynamic_cast<State*>(_atom->first));

        possibly_obtainable_positive_rewards_count += dynamic_cast<State*>(_atom->first)
            ->get__obtainable_positive_rewards_count();

        expected_value += probability * dynamic_cast<State*>(_atom->first)
            ->get__value();
    }

    
//     double entropy = 0.0;
//     for(auto _atom = belief.begin()
//             ; _atom != belief.end()
//             ; _atom++){
//         double probability = _atom->second;

//         assert(dynamic_cast<State*>(_atom->first));

//         possibly_obtainable_positive_rewards_count += dynamic_cast<State*>(_atom->first)
//             ->get__obtainable_positive_rewards_count();

//         entropy += probability * log(probability);
//     }
    
    
//     if(possibly_obtainable_positive_rewards_count == 0){
//         return -1e10;
//     }

//     cerr<<expected_value<<" "
//         <<static_cast<float>(static_cast<long long>(expected_value))<<std::endl;

    assert(sizeof(long long) == sizeof(double));
    
    return static_cast<float>(static_cast<long long>(expected_value));
}


double Belief_State_Value::operator()(POMDP_State* state) const
{
    return state->get__expected_reward(); 
}


float Entropy_Heuristic::operator()(POMDP_State* state) const
{
//     /*If there is no possibility of future positive rewards.*/
//     if(state->get__obtainable_positive_rewards_count() == 0){
//         return -1.0;
//     }
    
    auto& belief = state->get__belief_state();
    uint possibly_obtainable_positive_rewards_count = 0;
    double entropy = 0.0;
    for(auto _atom = belief.begin()
            ; _atom != belief.end()
            ; _atom++){
        double probability = _atom->second;

        assert(dynamic_cast<State*>(_atom->first));

        possibly_obtainable_positive_rewards_count += dynamic_cast<State*>(_atom->first)
            ->get__obtainable_positive_rewards_count();

        entropy += probability * log(probability);
//         double local_contribution = probability * log(probability);//probability * (log(1/probability)) / log(2);
//         entropy += local_contribution;
    }

    if(possibly_obtainable_positive_rewards_count == 0){
        return -1e10;
    } 
    
    
    assert(-entropy >= 0.0);
    
    cerr<<entropy<<std::endl;

    double exp_rew = 0.0;
    auto& belief_State = state->get__belief_state(); 
    for(auto _mdp_state = belief_State.begin()
            ; _mdp_state != belief_State.end()
            ; _mdp_state++){
        auto mdp_state = _mdp_state->first;
        double probability = _mdp_state->second;
        double value = dynamic_cast<Planning::State*>(mdp_state)
            ->get__obtainable_rewards_value();
        exp_rew += probability * value;
    }
    
    

    assert(entropy != 0.0);
    return static_cast<float>(-exp_rew);// * (1/(-entropy)));
}

Greedy_Heuristic::Greedy_Heuristic()
    :configured__expected_rewards_count__cache(false),
     configured__expected_rewards_value__cache(false)
{
}


float Greedy_Heuristic::operator()(POMDP_State* state) const
{

    INTERACTIVE_VERBOSER(true, 900, "Configuration status is :: "
                         <<configured__expected_rewards_count__cache<<" "<<expected_rewards_count__cache<<" "
                         <<configured__expected_rewards_value__cache<<" "<<expected_rewards_value__cache<<std::endl);
    
    assert(configured__expected_rewards_count__cache == configured__expected_rewards_value__cache);
    
    double expected_rewards_count = 0.0;
    double expected_rewards_value = 0.0;
    
    auto& belief_State = state->get__belief_state(); 
    for(auto _mdp_state = belief_State.begin()
            ; _mdp_state != belief_State.end()
            ; _mdp_state++){
        auto mdp_state = _mdp_state->first;
        double probability = _mdp_state->second;
        assert(dynamic_cast<Planning::State*>(mdp_state));

        double count = static_cast<double>(dynamic_cast<Planning::State*>(mdp_state)
                                           ->get__obtainable_rewards_count());
        
        double value = static_cast<double>(dynamic_cast<Planning::State*>(mdp_state)
                                           ->get__obtainable_rewards_value());
        
        expected_rewards_count += probability * count;
        
        expected_rewards_value += probability * value;
    }

    
//     std::cerr<<"Starting configuration test."<<std::endl;
    if(!configured__expected_rewards_count__cache){
        INTERACTIVE_VERBOSER(true, 900, "Not configured yet."<<std::endl);
        
        expected_rewards_count__cache = expected_rewards_count;
        expected_rewards_value__cache = expected_rewards_value;

        INTERACTIVE_VERBOSER(true, 900, "CONFIGURED "<<expected_rewards_count__cache<<" "
                             <<expected_rewards_value__cache<<" "<<std::endl);
        
        
        configured__expected_rewards_count__cache
            = configured__expected_rewards_value__cache
            = true;
    }



    double entropy = 0.0;
    for(auto _atom = belief_State.begin()
            ; _atom != belief_State.end()
            ; _atom++){
        double probability = _atom->second;
        entropy += probability * log(probability);
    }
    
    float answer =  static_cast<float>(-expected_rewards_value * - entropy);// (1 / (- entropy))*(expected_rewards_value
//                                        + (expected_rewards_count / expected_rewards_count__cache)
//                                         + (expected_rewards_value / expected_rewards_value__cache)));

    INTERACTIVE_VERBOSER(true, 900, "EVALUATION :: "<<answer<<std::endl
                         <<" Reward :: "<<state->get__expected_reward()<<"\n"
                         <<" Reward element counting :: "<<expected_rewards_count
                         <<" -- "<<expected_rewards_count__cache<<" -- "
                         <<(expected_rewards_count / expected_rewards_count__cache)<<"\n"
                         <<" Reward values counting :: "<<expected_rewards_value
                         <<" "<<expected_rewards_value__cache<<" "
                         <<(expected_rewards_value / expected_rewards_value__cache)<<" "<<std::endl);

    return answer;//((((int)state)%2)?answer:-answer);// static_cast<float>(state->get__expected_reward()
//                               + (expected_rewards_count / expected_rewards_count__cache)
//                               + (expected_rewards_value / expected_rewards_value__cache));
    
    //return static_cast<int>(state->get__expected_reward()) // +
//         expected_rewards_count +
//         expected_rewards_value)
        ;
}

double Obtainable_Value::operator()(POMDP_State* state) const
{
    double answer = 0.0;
    auto& belief_State = state->get__belief_state(); 
    for(auto _mdp_state = belief_State.begin()
            ; _mdp_state != belief_State.end()
            ; _mdp_state++){
        auto mdp_state = _mdp_state->first;
        auto probability = _mdp_state->second;
        assert(dynamic_cast<Planning::State*>(mdp_state));

        answer += probability * static_cast<double>(
            dynamic_cast<Planning::State*>(mdp_state)
            ->get__obtainable_rewards_count());
    }
    
    return answer; 
}

double Obtainable_Values_Count::operator()(POMDP_State* state) const
{
    double answer = 0.0;
    auto& belief_State = state->get__belief_state(); 
    for(auto _mdp_state = belief_State.begin()
            ; _mdp_state != belief_State.end()
            ; _mdp_state++){
        auto mdp_state = _mdp_state->first;
        auto probability = _mdp_state->second;
        assert(dynamic_cast<Planning::State*>(mdp_state));

        answer += probability * static_cast<double>(
            dynamic_cast<Planning::State*>(mdp_state)
            ->get__obtainable_rewards_value());
    }
    
    return answer; 
}
