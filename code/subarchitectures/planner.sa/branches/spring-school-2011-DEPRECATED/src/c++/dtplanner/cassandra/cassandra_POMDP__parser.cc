/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
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
 *
 * SEE ALSO :
 *
 * \c++-0x-source{cassandra_POMDP__parser____observations_implementation.cc}
 *
 * \c++-0x-source{cassandra_POMDP__parser____rewards_implementation.cc}
 *
 * \c++-0x-source{cassandra_POMDP__parser____transitions_implementation.cc}
 *
 * \c++-0x-source{cassandra_POMDP__parser____indexed_strings.cc}
 *
 */

#include "cassandra_POMDP__parser.hh"

using namespace POMDP::Parsing;

using std::cin;


using std::istringstream;
using std::ostringstream;

namespace POMDP
{
    namespace Parsing
    {   
        std::tr1::shared_ptr<Problem_Data> problem_Data;
    }
}

void Problem_Data::add__number(const std::string& str)
{
    istringstream iss(str);
    iss>>number;
    VERBOSER(1, "Pushing back :: "<<number);
    numbers.push_back(number);
}

void Problem_Data::add__natural_number(const std::string& str)
{
    istringstream iss(str);
    iss>>natural_number;
}



            

void Problem_Data::add__actions_given_number(const std::string& str)
{
    for(auto i = 0; i < natural_number; i++){
        ostringstream oss;
        oss<<'a'<<i;
        actions.push_back(oss.str());
    }
}

void Problem_Data::add__states_given_number(const std::string& str)
{
    for(auto i = 0; i < natural_number; i++){
        ostringstream oss;
        oss<<'s'<<i;
        states.push_back(oss.str());
    }
}

void Problem_Data::add__observations_given_number(const std::string& str)
{
    for(auto i = 0; i < natural_number; i++){
        ostringstream oss;
        oss<<'o'<<i;
        observations.push_back(oss.str());
    }
}

void Problem_Data::add__start_state_distribution(const std::string&)
{
    VERBOSER(30, "We have :: "<<states.size()<<" and :: "<<numbers.size()<<" starting state elements."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(numbers.size() != states.size(),
                              "Start state distribution does not determine the truth value of every state.");
    assert(numbers.size() == states.size());

    assert(start_state_distribution.size() == 0);

    start_state_distribution = numbers;
//     for(auto a_number = numbers.begin()
//             ; a_number != numbers.end()
//             ; a_number++){
//         start_state_distribution.push_back(*a_number);
//     }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__action(const std::string& str)
{
    actions.push_back(str);
}

void Problem_Data::add__state(const std::string& str)
{
    states.push_back(str);
}

void Problem_Data::add__observation(const std::string& str)
{
    observations.push_back(str);
}


void Problem_Data::report__action(const std::string& str)
{
    action = str;
}

void Problem_Data::report__starting_state(const std::string& str)
{
    start_state = str;
}

void Problem_Data::report__successor_state(const std::string& str)
{
    successor_state = str;
}

void Problem_Data::report__observation(const std::string& str)
{
    observation = str;
}

void Problem_Data::report__action_as_number(const std::string& str)
{
//     ostringstream oss;
//     oss<<'a'<<natural_number;
//     action = oss.str();

    assert(natural_number < actions.size());
    action = actions[natural_number];
    
    assert(find(actions.begin()
                , actions.end()
                , action)
           != actions.end());
}

void Problem_Data::report__starting_state_as_number(const std::string& str)
{
//     ostringstream oss;
//     oss<<'s'<<natural_number;
    assert(natural_number < states.size());
    //start_state = oss.str();
    start_state = states[natural_number];
    
    assert(find(states.begin()
                , states.end()
                , start_state)
           != states.end());
}

void Problem_Data::report__successor_state_as_number(const std::string& str)
{
//     ostringstream oss;
//     oss<<'s'<<natural_number;
    assert(natural_number < states.size());
    successor_state = states[natural_number];
//     _state = oss.str();
    
    assert(find(states.begin()
                , states.end()
                , successor_state)
           != states.end());
}

void Problem_Data::report__observation_as_number(const std::string& str)
{
//     ostringstream oss;
//     oss<<'o'<<natural_number;
    assert(natural_number < observations.size());
    observation = observations[natural_number];
//     observation = oss.str();

    assert(find(observations.begin()
                , observations.end()
                , observation)
           != observations.end());
}


void Problem_Data::report__discount_factor(const std::string& str)
{
    discount_factor = number;
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::report__value_Type_is_Reward(const std::string& str)
{
    value_Type = Reward;
}

void Problem_Data::report__value_Type_is_Cost(const std::string& str)
{
    value_Type = Cost;
}


/******************************************************************************************************
 ******************************************************************************************************
 * OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- OSTREAM ostream -- 
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/
       
std::ostream& POMDP::Parsing::operator<<(std::ostream& o
                                         , const POMDP::Parsing::Problem_Data::Value_Type& value_Type)
{
    return o<<((value_Type == POMDP::Parsing::Problem_Data::Value_Type::Cost)?"Cost":"Reward");
}

std::ostream& POMDP::Parsing::operator<<(std::ostream& o,
                                         const POMDP::Parsing::Problem_Data& problem_Data)
{
    auto observation_model = problem_Data.observation_model;
    auto transition_model = problem_Data.transition_model;
    auto reward_model = problem_Data.reward_model;
    auto states = problem_Data.states;
    auto actions = problem_Data.actions;
    auto observations = problem_Data.observations; 
    
    o<<"discount : "<<problem_Data.discount_factor<<std::endl;

    o<<"values : "<<problem_Data.value_Type<<std::endl;
    
    o<<"states : "<<states<<std::endl;

    o<<"actions : "<<actions<<std::endl;

    o<<"observations : "<<observations<<std::endl;

    o<<"start: ";//<<problem_Data.start_state_distribution<<std::endl;
    for(auto i = 0//problem_Data.start_state_distribution.begin()
            ; i < problem_Data.start_state_distribution.size()
            ; i++){
        o<<problem_Data.start_state_distribution[i];
        if(i+1 != problem_Data.start_state_distribution.size()){
            o<<" ";
        }
        
    }
    o<<std::endl;

    
    if(!problem_Data.already__called__initialise__transition_model){
        WARNING("Parsed no action transition information, now initialising the model.");
        const_cast<POMDP::Parsing::Problem_Data&>(problem_Data).initialise__transition_model();
    }

    if(!problem_Data.already__called__initialise__reward_model){
        WARNING("Parsed no reward information, now initialising the model.");
        const_cast<POMDP::Parsing::Problem_Data&>(problem_Data).initialise__reward_model();
    }
    
    if(!problem_Data.already__called__initialise__observation_model){
        WARNING("Parsed no observation information, now initialising the model.");
        const_cast<POMDP::Parsing::Problem_Data&>(problem_Data).initialise__observation_model();
    }
    
    for(auto p= reward_model.begin()
            ; p!= reward_model.end()
            ; p++){

        QUERY_UNRECOVERABLE_ERROR(find(actions.begin(), actions.end(), p->first) == actions.end()
                                  , "action :: "<<p->first<<" was not found for some reason in the set of actions.\n");
        assert(find(actions.begin(), actions.end(), p->first) != actions.end());
        
        for(auto pp = p->second.begin()
                ; pp != p->second.end()
                ; pp++){
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), pp->first) == states.end()
                                      , "state :: "<<pp->first<<" was not found for some reason in the set of states.\n");
            assert(find(states.begin(), states.end(), pp->first) != states.end());
        
            for(auto ppp = pp->second.begin()
                    ; ppp != pp->second.end()
                    ; ppp++){
                QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), ppp->first) == states.end()
                                          , "state :: "<<ppp->first<<" was not found for some reason in the set of states.\n");
                assert(find(states.begin(), states.end(), ppp->first) != states.end());
                for(auto pppp = ppp->second.begin()
                        ; pppp != ppp->second.end()
                        ; pppp++){
                    QUERY_UNRECOVERABLE_ERROR(find(observations.begin(), observations.end(), pppp->first) == observations.end()
                                              , "observation :: "<<pppp->first<<" was not found for some reason in the set of observations.\n");
                    assert(find(observations.begin(), observations.end(), pppp->first) != observations.end());
                    
                    if (pppp->second != 0.0){
                        VERBOSER(1, p->first<<" "<<pp->first <<" "<<ppp->first <<" "<<pppp->first<<" = "<<pppp->second);
                        o<<"R:"<<p->first<<" "<<pp->first <<" "<<ppp->first <<" "<<pppp->first<<" = "<<pppp->second<<std::endl;
                    }
                }
                    
            }   
        }
    }

    
    assert(transition_model.size());
    for(auto p = transition_model.begin()
            ; p != transition_model.end()
            ; p++){
        
        QUERY_UNRECOVERABLE_ERROR(find(actions.begin(), actions.end(), p->first) == actions.end()
                                  , "action :: "<<p->first<<" was not found for some reason in the set of actions.\n");
        
        assert(p->second.size());
        
        for(auto pp = p->second.begin()
                ; pp != p->second.end()
                ; pp++){
            
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), pp->first) == states.end()
                                      , "state :: "<<pp->first<<" was not found for some reason in the set of states.\n");

            assert(pp->second.size());
            
            for(auto ppp = pp->second.begin()
                    ; ppp != pp->second.end()
                    ; ppp++){
                
                QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), ppp->first) == states.end()
                                          , "state :: "<<ppp->first<<" was not found for some reason in the set of states.\n");
                

                if(0 != ppp->second){
                    o<<"T:"<<p->first<<" "<<pp->first <<" "<<ppp->first <<" = "<<ppp->second<<std::endl;
                }
                
            }
        }
    }
    
    for(auto p = observation_model.begin()
            ; p != observation_model.end()
            ; p++){
        
        QUERY_UNRECOVERABLE_ERROR(find(actions.begin(), actions.end(), p->first) == actions.end()
                                  , "action :: "<<p->first<<" was not found for some reason in the set of actions.\n");
        
        for(auto pp = p->second.begin()
                ; pp != p->second.end()
                ; pp++){
            
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), pp->first) == states.end()
                                      , "state :: "<<pp->first<<" was not found for some reason in the set of states.\n");
            
            for(auto ppp = pp->second.begin()
                    ; ppp != pp->second.end()
                    ; ppp++){
                
                QUERY_UNRECOVERABLE_ERROR(find(observations.begin(), observations.end(), ppp->first) == observations.end()
                                          , "observation :: "<<ppp->first<<" was not found for some reason in the set of observations.\n");
                

                if(0 != ppp->second){
                    o<<"O:"<<p->first<<" "<<pp->first <<" "<<ppp->first <<" = "<<ppp->second<<std::endl;
                }
            }
        }
    }
    
    return o;
        
// //     return o;


//     assert(actions.size());
//     for(auto _action = actions.begin()
//             ; _action != actions.end()
//             ; _action++){
        
//         bool _action__zeroed = false;
        
//         auto reward_for_action__iterator
//             = reward_model.find(*_action);
        
//         if(reward_for_action__iterator == reward_model.end()) {

//             VERBOSER(1, "\"zeroing\" action :: "<<*_action<<std::endl);
//             {char ch; cin>>ch;};
            
//             _action__zeroed = true;
//         }
        
        
//         assert(states.size());
//         for(auto _starting_state = states.begin()
//                 ; _starting_state != states.end()
//                 ; _starting_state++){

//             bool _starting_state__zeroed = _action__zeroed;
//             decltype(reward_for_action__iterator->second.find(*_starting_state)) reward_for_starting_state__iterator;
//             if(!_starting_state__zeroed){
//                 reward_for_starting_state__iterator
//                     = reward_for_action__iterator->second.find(*_starting_state);
                
//                 if(reward_for_starting_state__iterator == reward_for_action__iterator->second.end()) {
                    
//                     VERBOSER(1, "\"zeroing\" starting-state :: "<<*_starting_state<<std::endl);
//                     {char ch; cin>>ch;};
//                     _starting_state__zeroed = true;
//                 }
                
//             }
            
//             assert(states.size());
//             for(auto _successor_state = states.begin()
//                     ; _successor_state != states.end()
//                     ; _successor_state++){

//                 bool _successor_state__zeroed = _starting_state__zeroed;
                
//                 decltype(reward_for_starting_state__iterator->second.find(*_successor_state)) reward_for_successor_state__iterator;
//                 if(!_successor_state__zeroed){
//                     reward_for_successor_state__iterator
//                         = reward_for_starting_state__iterator->second.find(*_successor_state);
                
//                     if(reward_for_successor_state__iterator == reward_for_starting_state__iterator->second.end()) {
                        
//                         VERBOSER(1, "\"zeroing\" successor-state :: "<<*_successor_state<<std::endl);
//                         {char ch; cin>>ch;};
//                         _successor_state__zeroed = true;
//                     }
                    
//                 }
                
//                 assert(observations.size());
//                 for(auto _observation = observations.begin()
//                         ; _observation != observations.end()
//                         ; _observation++){

//                     bool _observation__zeroed = _successor_state__zeroed;
                    
//                     decltype(reward_for_successor_state__iterator->second.find(*_observation)) reward_for_observation__iterator;
//                     if(!_observation__zeroed){
//                         reward_for_observation__iterator
//                             = reward_for_successor_state__iterator->second.find(*_observation);
                        
//                         if(reward_for_observation__iterator == reward_for_successor_state__iterator->second.end()) {
                            
//                             VERBOSER(1, "\"zeroing\" observation :: "<<*_observation<<std::endl);
//                             {char ch; cin>>ch;};
//                             _observation__zeroed = true;
//                         }
                        
//                     }

//                     if(!_observation__zeroed){
//                         VERBOSER(1, "Reward :: "<<*_action<<" "
//                                  <<*_starting_state<<" "
//                                  <<*_successor_state<<" "
//                                  <<*_observation<<" = "
//                                  <<reward_model[*_action][*_starting_state][*_successor_state][*_observation]<<std::endl);

//                         assert(reward_model.find(*_action) != reward_model.end());
//                         assert(reward_model[*_action].find(*_starting_state)
//                                != reward_model[*_action].end());
//                         assert(reward_model[*_action][*_starting_state].find(*_successor_state)
//                                != reward_model[*_action][*_starting_state].end());
//                         assert(reward_model[*_action][*_starting_state][*_successor_state].find(*_observation)
//                                != reward_model[*_action][*_starting_state][*_successor_state].end());
//                         o<<"R : "
//                          <<*_action<<" : "
//                          <<*_starting_state<<" : "
//                          <<*_successor_state<<" : "
//                          <<*_observation<<" "
//                          <<reward_model[*_action][*_starting_state][*_successor_state][*_observation]//((!_observation__zeroed)?reward_model[*_action][*_starting_state][*_successor_state][*_observation]:0.0)
//                          <<std::endl;
//                     }
//                 }
//             }
//         }
//     }
    
}



void POMDP::Parsing::parse_Cassandra_POMDP_problem(
    const std::string& problem_file_name,
    std::tr1::shared_ptr<Problem_Data>& input_problem){
    if(!input_problem){
        input_problem = std::tr1::shared_ptr<Problem_Data>(new Problem_Data());
    }
            
    pegtl::smart_parse_file< problem >
        ( true/*_trace*/, problem_file_name, input_problem);    
}

/* "Stigmergy" : (biology) stimulation of workers by the performance
 * they have achieved. -- Pierre-Paul Grassé */
