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

#include "flatten.hh"

#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning;

Flatten::Flatten(Planning::Parsing::Problem_Data& in)
    :Planning::Solver(in),
     state_id(1),
     observation_id(1)
{
}

#include <iomanip>

void Flatten::print_flat_problem()
{
    cout<<fixed<<setprecision(15);
    cout<<"discount: 0.95"<<std::endl;
    cout<<std::endl;
    cout<<"values: reward"<<std::endl;
    cout<<std::endl;
    
    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        
        if(*state == get__starting_belief_state()){
            continue;
        }
        
        auto belief_state = (*state)->get__belief_state();
        assert(belief_state.size() == 1);
        auto atom = *belief_state.begin();
        auto mdp_state = atom.first;

        if(state_ids.find(mdp_state) == state_ids.end()){
            state_ids[mdp_state] = state_id++;
        }
    }

    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        
        if(*state == get__starting_belief_state()){
            continue;
        }
        
        auto actions = (*state)->get__action_based_successor_driver();

        for(auto action = actions.begin(); action != actions.end()
                ; action++){
            action_ids.insert(*action);
        }   
    }
    
    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        
        if(*state == get__starting_belief_state()){
            continue;
        }
        
        auto obss = (*state)->get__observation_based_successor_driver();

        for(auto obs = obss.begin(); obs != obss.end(); obs++){
            for(auto ob = obs->begin(); ob != obs->end(); ob++){
                if(observation_ids.find(*ob) == observation_ids.end()){
                    observation_ids[*ob] = observation_id++;
                }
            }
            
        }
    }
    
    cout<<"actions: ";
    
    for(auto action = action_ids.begin()
            ; action != action_ids.end()
            ; action++){
        cout<<"a"<<*action<<" ";
    }
    
    cout<<std::endl;
    cout<<std::endl;
    
    
    cout<<"observations: ";
    
    for(auto ob = observation_ids.begin(); ob != observation_ids.end(); ob++){
        cout<<"o"<<ob->second<<" ";
    }
    
    cout<<std::endl;
    cout<<std::endl;

    cout<<"states: ";
    
    for(auto state = state_ids.begin(); state != state_ids.end(); state++){
        cout<<"s"<<state->second<<" ";
    }
    
    cout<<std::endl;
    cout<<std::endl;
    
    cout<<"start: ";
    auto start = get__starting_belief_state();
    auto starting_belief_state = start->get__belief_state();

    for(auto state = starting_belief_state.begin(); state != starting_belief_state.end(); state++){
        auto mdp_state = state->first;
        double prob = state->second;
        starting[state_ids[mdp_state]] = prob;
    }

    double sum = 0.0;
    for(int i =1 ; i < state_id; i++){
        if(starting.find(i) == starting.end()){
            cout<<"0 ";
        } else {
            sum += starting[i];
            cout<<starting[i]<<" ";
        }
    }
    cerr<<" sum :: "<<sum<<std::endl;
    
    cout<<std::endl;
    cout<<std::endl;
    

    // T: ACTION : STATE : STATE   PROB

    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        
        if(*state == get__starting_belief_state()){
            continue;
        }

        auto belief_state = (*state)->get__belief_state();
        assert(belief_state.size() == 1);
        auto atom = *belief_state.begin();
        auto mdp_state = atom.first;
        
        int from_state = state_ids[mdp_state];
        
        
        for(auto action = action_ids.begin()
                ; action != action_ids.end()
                ; action++){
            state_prob[State_Trans(*action, from_state, from_state)] = 1.0;

            observation_prob[Obs_Trans(*action,
                                       from_state,
                                       observation_ids.begin()->second)] = 1.0;
        }
        
    }
    
//     for(auto obs = state_prob.begin(); obs != state_prob.end(); obs++){
//         auto thing = obs->first;

//         auto action = get<0>(thing);
//         auto start = get<1>(thing);
//         auto end = get<2>(thing);
//         cerr<<"T: a"<<action<<" : s"<<start<<" : s"<<end<<"   1.0"<<std::endl;
//     }
//     for(auto obs = observation_prob.begin(); obs != observation_prob.end(); obs++){
//         auto thing = obs->first;

//         auto action = get<0>(thing);
//         auto start = get<1>(thing);
//         auto end = get<2>(thing);
        
//         cerr<<"O: a"<<action<<" : s"<<start<<" : o"<<end<<"   1.0"<<std::endl;
//     }
    
    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        
        if(*state == get__starting_belief_state()){
            continue;
        }

        
        auto belief_state = (*state)->get__belief_state();
        assert(belief_state.size() == 1);
        auto atom = *belief_state.begin();
        auto mdp_state = atom.first;
        
        int from_state = state_ids[mdp_state];
        

        auto actions = (*state)->get__action_based_successor_driver();
        auto observations = (*state)->get__observation_based_successor_driver();
        auto successors = (*state)->get__successors_under_actions();
        auto probabilities = (*state)->get__observation_probabilities();
        
        std::set<int> skipped_actions = action_ids;
        for(uint i = 0; i < actions.size(); i++){

            skipped_actions.erase(actions[i]);
            
            int action_index = actions[i];

            assert(successors[i].size() == observations[i].size());
            assert(successors[i].size() == probabilities[i].size());
            
            std::set<MDP_State*> set_of_states;
            for(int j = 0; j < successors[i].size(); j++){//auto successor = successors[i].begin(); successor != successors[i].end(); successor++){
                auto successor_POMDP_state = successors[i][j];//successors[i].begin();
                auto POMDP_state = successor_POMDP_state->get__belief_state();
                assert(POMDP_state.size() == 1);
                auto tmp = *POMDP_state.begin();
                auto mdp_successor_state = tmp.first;
                set_of_states.insert(mdp_successor_state);
                
                auto observation = observations[i][j];

//                 if(observation_prob.find(Obs_Trans(action_index,
//                                            state_ids[mdp_successor_state],
//                                            observation_ids[observation]))
//                    == observation_prob.end()){
//                     observation_prob[Obs_Trans(action_index,
//                                            state_ids[mdp_successor_state],
//                                            observation_ids[observation])] = 0.0;
//                 } else {
//                     assert(observation_prob[Obs_Trans(action_index,
//                                            state_ids[mdp_successor_state],
//                                            observation_ids[observation])] == probabilities[i][j]);
//                 }


                auto tuple = Obs_Trans(action_index,
                                       state_ids[mdp_successor_state],
                                       observation_ids.begin()->second);
                if(observation_prob.find(tuple) != observation_prob.end()){

                    if(observation_prob[Obs_Trans(action_index,
                                                  state_ids[mdp_successor_state],
                                                  observation_ids.begin()->second)] == 1.0){

                        observation_prob[Obs_Trans(action_index,
                                                   state_ids[mdp_successor_state],
                                                   observation_ids.begin()->second)] = 1.0 - probabilities[i][j];
                            //observation_prob.erase(tuple);
                    }
                }
                
                
                observation_prob[Obs_Trans(action_index,
                                           state_ids[mdp_successor_state],
                                           observation_ids[observation])] = probabilities[i][j];
                
//                 cerr<<"O: a"<<action_index<<" : s"<<state_ids[mdp_successor_state]<<" : o"<<observation_ids[observation]<<"   "<<observation_prob[Obs_Trans(action_index,
//                                            state_ids[mdp_successor_state],
//                                            observation_ids[observation])]<<std::endl;
            }
            
            assert(set_of_states.size() == 1);
//             assert(successors[i].size() == 1);
            auto successor_POMDP_state = *successors[i].begin();
            auto POMDP_state = successor_POMDP_state->get__belief_state();
            assert(POMDP_state.size() == 1);
            auto tmp = *POMDP_state.begin();
            auto mdp_successor_state = tmp.first;

            state_prob[State_Trans(action_index, from_state, state_ids[mdp_successor_state])] = 1.0;

            if(from_state != state_ids[mdp_successor_state]){
                state_prob.erase(State_Trans(action_index, from_state, from_state));
            }
            
            //cout<<"T: a"<<action_index<<" : s"<<from_state<<" : s"<<state_ids[mdp_successor_state]<<"   1.0"<<std::endl;

            //R: ams : s000000    : * : * -100.000000

            
            cout<<"R: a"<<action_index<<" : s"<<from_state<<" : s"<<state_ids[mdp_successor_state]
                <<" : *   "<<mdp_successor_state->get__reward()<<std::endl;
            
        }
        
    }

    for(auto obs = state_prob.begin(); obs != state_prob.end(); obs++){
        auto thing = obs->first;

        auto action = get<0>(thing);
        auto start = get<1>(thing);
        auto end = get<2>(thing);
        cout<<"T: a"<<action<<" : s"<<start<<" : s"<<end<<"   1.0"<<std::endl;
    }
    

    for(auto p = observation_prob.begin(); p != observation_prob.end(); p++){
        auto spec = p->first;
        auto action = get<0>(spec);
        auto state = get<1>(spec);
        auto observation = get<2>(spec);
        
        double prob = p->second;

        cout<<"O: a"<<action<<" : s"<<state<<" : o"<<observation<<"   "<<prob<<std::endl;
    }
    
}
