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

using std::cin;


using std::istringstream;
using std::ostringstream;

void Problem_Data::initialise__observation_model()
{
    assert(actions.size());
    assert(states.size());

    
    if(already__called__initialise__observation_model) {return;}
    already__called__initialise__observation_model = true;

    
    for(auto action = actions.begin()
            ; action != actions.end()
            ; action++){

        assert("" != *action);
        
        observation_model[*action] = decltype(observation_model.find(*action)->second)();

        decltype(observation_model[*action])& observation_model__level1
            = observation_model[*action];
        
        for(auto start_state = states.begin()
                ; start_state != states.end()
                ; start_state++){
            
            assert("" != *start_state);
        
            observation_model__level1[*start_state]
                = decltype(observation_model__level1.find(*action)->second)();
        
            decltype(observation_model__level1[*start_state])& observation_model__level2
                = observation_model__level1[*start_state];
            
            for(auto observation = observations.begin()
                    ; observation != observations.end()
                    ; observation++){
                assert("" != *observation);
                
                observation_model__level2[*observation] = 0.0;
            }    
        }       
    }
}

/******************************************************************************************************
 ******************************************************************************************************
 * INTERFACE TO :: DESCRIPTION OF OBSERVATIONS
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/
            
void Problem_Data::add__observations____action__successor_state__observation__UNIFORM(const std::string&  str)
{
    initialise__observation_model();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    number = static_cast<double>(1.0) / static_cast<double>(observations.size());
    numbers.push_back(number);
    add__observations____action__successor_state__observation_();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action__successor_state__UNIFORM(const std::string&  str)
{
    initialise__observation_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability = static_cast<double>(1.0) / static_cast<double>(observations.size());
    
    for(auto _observation = observations.begin()
            ; _observation != observations.end()
            ; _observation++){        
        numbers.push_back(transition_probability);
    }
    
    add__observations____action__successor_state_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action__UNIFORM(const std::string&  str)
{
    
    initialise__observation_model();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability = static_cast<double>(1.0) / static_cast<double>(observations.size());
    
    for(auto _successor_state = states.begin()
            ; _successor_state != states.end()
            ; _successor_state++){

        for(auto _observation = observations.begin()
                ; _observation != observations.end()
                ; _observation++){        
            numbers.push_back(transition_probability);
        }
    }
    
    add__observations____action_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

                

void Problem_Data::add__observations____action__successor_state__observation__IDENTITY(const std::string&  str)
{
    initialise__observation_model();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    number = (observation == successor_state)?1.0:0.0;
    numbers.push_back(number);
    add__observations____action__successor_state__observation_();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action__successor_state__IDENTITY(const std::string&  str)
{
    initialise__observation_model();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability;
    
    for(auto _observation = observations.begin()
            ; _observation != observations.end()
            ; _observation++){
        if(*_observation == successor_state){
            transition_probability = 1.0;
        } else {
            transition_probability = 0.0; 
        }
            
        numbers.push_back(transition_probability);
    }

    
    add__observations____action__successor_state_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action__IDENTITY(const std::string&  str)
{
    initialise__observation_model();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability;
    
    for(auto _successor_state = states.begin()
            ; _successor_state != states.end()
            ; _successor_state++){

        for(auto _observation = observations.begin()
                ; _observation != observations.end()
                ; _observation++){
            if(*_observation == *_successor_state){
                transition_probability = 1.0;
            } else {
                transition_probability = 0.0; 
            }
            
            numbers.push_back(transition_probability);
        }
    }
    
    add__observations____action_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}



void Problem_Data::add__observations____action__successor_state__observation_()
{
    if("*" == observation){
        
        for(auto _observation = observations.begin()
                ; _observation != observations.end()
                ; _observation++){
            observation = *_observation;
            
            QUERY_WARNING(number == 0,
                          "Reward :: "
                          <<action<<" "
                          <<start_state
                          <<" "<<successor_state
                          <<" "<<observation
                          <<" = "<<number<<std::endl);

            
            QUERY_UNRECOVERABLE_ERROR(find(actions.begin(), actions.end(), action) == actions.end()
                                  , "action :: "<<action<<" was not found for some reason in the set of actions.\n");
            
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), successor_state) == states.end()
                                  , "state :: "<<successor_state<<" was not found for some reason in the set of states.\n");
            
            QUERY_UNRECOVERABLE_ERROR(find(observations.begin(), observations.end(), observation) == observations.end()
                                  , "observation :: "<<observation<<" was not found for some reason in the set of observations.\n");
            

            
            observation_model[action][successor_state][observation] = number;
        }
        
       observation = "*";
    } else {
        
        QUERY_WARNING(number == 0,
                      "Observation :: "
                      <<action<<" "
                      <<start_state<<" "
                      <<successor_state<<" "
                      <<observation<<" = "
                      <<number<<std::endl);
        
        observation_model[action][successor_state][observation] = number;
    }
    
}

void Problem_Data::add__observations____action__successor_state___observation()
{
    if("*" == successor_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            successor_state = *_state;
            add__observations____action__successor_state__observation_();
        }
        
        successor_state = "*";
    } else {
        add__observations____action__successor_state__observation_();
    }
}

void Problem_Data::add__observations____action___successor_state__observation()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__observations____action__successor_state___observation();
        }
        
        action = "*";
    } else {
        add__observations____action__successor_state___observation();
    }
}

void Problem_Data::add__observations____action__successor_state__observation(const std::string& str)
{
    
    initialise__observation_model();
    add__observations____action___successor_state__observation();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

int a_counter = 0;

void Problem_Data::add__observations____action__successor_state_NUMBERS()
{
    for(auto _observation = observations.begin()
            ; _observation != observations.end()
            ; _observation++, numbers_index++){
        assert(numbers_index < numbers.size());
                
        decltype(number) number__before = number;
        decltype(observation) observation__before = observation;
                
        observation = *_observation;
        number = numbers[numbers_index];
        add__observations____action__successor_state__observation_();

//         std::cerr<<number<<" ";
//         a_counter ++;

//         if(!(a_counter % 3)){
//             std::cerr<<std::endl;
//         }
        
        
//         VERBOSER(200, "Action :: "<<action
//                  <<" successor-state :: "<<successor_state
//                  <<" observation :: "<<observation
//                  <<" = "<<number<<std::endl);
        
        number = number__before;
        observation = observation__before;
    }
}

void Problem_Data::add__observations____action__successor_state_()
{
    if("*" == successor_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            successor_state = *_state;
            numbers_index = 0;
            add__observations____action__successor_state_NUMBERS();
        }
        
        successor_state = "*";
    } else {
        numbers_index = 0;
        add__observations____action__successor_state_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action___successor_state()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__observations____action__successor_state_();
        }
        
        action = "*";
    } else {
        add__observations____action__successor_state_();
    }
}

void Problem_Data::add__observations____action__successor_state(const std::string& )
{
    
    initialise__observation_model();
    add__observations____action___successor_state();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}


void Problem_Data::add__observations____action_NUMBERS()
{
    
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        decltype(successor_state) successor_state__before = successor_state ;
        successor_state = *_state;

        
        add__observations____action__successor_state_NUMBERS();

        successor_state = successor_state__before;
    }
}

void Problem_Data::add__observations____action_()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            numbers_index = 0;
            add__observations____action_NUMBERS();
        }
        
        action = "*";
    } else {
        numbers_index = 0;
        add__observations____action_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__observations____action(const std::string& )
{
    initialise__observation_model();
    add__observations____action_();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

