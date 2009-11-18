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
 */

#include "cassandra_POMDP__parser.hh"

using namespace POMDP::Parsing;
bool Problem_Data::sanity__transition_model__has_non_zero_entries() const
{

    for(auto p = transition_model.begin()
            ; p != transition_model.end()
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
                
                QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), ppp->first) == states.end()
                                          , "state :: "<<ppp->first<<" was not found for some reason in the set of states.\n");
                

                if(0.0 != ppp->second){
                    return true;
                }
                
            }
        }
    }

    return false;
}


bool Problem_Data::sanity__transition_model() const
{

    for(auto starting_state = states.begin()
            ; starting_state != states.end()
        ; starting_state++){

        for(auto action = actions.begin()
                ; action != actions.end()
                ; action++){

            double sum_of_transition_probability = 0.0;
            
            for(auto successor_state = states.begin()
                    ; successor_state != states.end()
                    ; successor_state++){

                sum_of_transition_probability +=
                    get__transition_probability(*action,
                                                *starting_state,
                                                *successor_state);
            }

            Are_Doubles_Close are_Doubles_Close(1e-5);
            if(!are_Doubles_Close(sum_of_transition_probability, 1.0)){
                WARNING("State transition model is broken. Transitions probabilities from :: "
                        <<*starting_state<<" using :: "
                        <<*action<<" sum to :: "<<sum_of_transition_probability<<std::endl);
                
                return false;
            }
        }
    }
    
    return true;
}



bool Problem_Data::sanity__observation_model__has_non_zero_entries() const
{
    assert(observation_model.size());
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
            
            double sum_of_transition_probability = 0.0;
            for(auto ppp = pp->second.begin()
                    ; ppp != pp->second.end()
                    ; ppp++){
                
                QUERY_UNRECOVERABLE_ERROR(find(observations.begin(), observations.end(), ppp->first) == observations.end()
                                          , "observation :: "<<ppp->first<<" was not found for some reason in the set of observations.\n");
                
                if(0.0 != ppp->second){
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool Problem_Data::sanity__observation_model() const
{

    assert(actions.size());
    
    for(auto action = actions.begin()
            ; action != actions.end()
            ; action++){
            
        for(auto successor_state = states.begin()
                ; successor_state != states.end()
                ; successor_state++){

            double sum_of_transition_probability = 0.0;

            for(auto observation = observations.begin()
                    ; observation != observations.end()
                    ; observation++){
                    
                sum_of_transition_probability +=
                    get__observation_probability(*action,
                                                 *successor_state,
                                                 *observation);
            }
                
            Are_Doubles_Close are_Doubles_Close(1e-5);
            if(!are_Doubles_Close(sum_of_transition_probability, 1.0)){
                WARNING("Observation model is broken. Observation probabilities to :: "
                        <<*successor_state<<" using :: "
                        <<*action<<" sum to :: "<<sum_of_transition_probability<<std::endl);
                    
                return false;
            }
        }
    }
    return true;
}
