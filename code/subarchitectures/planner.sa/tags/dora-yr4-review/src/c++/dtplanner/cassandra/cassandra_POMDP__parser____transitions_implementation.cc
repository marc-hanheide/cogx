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

void Problem_Data::initialise__transition_model()
{
    assert(actions.size());
    assert(states.size());

    
    if(already__called__initialise__transition_model) {return;}
    already__called__initialise__transition_model = true;

    
    for(auto action = actions.begin()
            ; action != actions.end()
            ; action++){

        assert("" != *action);
        
        transition_model[*action] = decltype(transition_model.find(*action)->second)();

        decltype(transition_model[*action])& transition_model__level1
            = transition_model[*action];
        
        for(auto start_state = states.begin()
                ; start_state != states.end()
                ; start_state++){
            
            assert("" != *start_state);
        
            transition_model__level1[*start_state]
                = decltype(transition_model__level1.find(*action)->second)();
        
            decltype(transition_model__level1[*start_state])& transition_model__level2
                = transition_model__level1[*start_state];
            
            for(auto successor_state = states.begin()
                    ; successor_state != states.end()
                    ; successor_state++){
                assert("" != *successor_state);
                
                transition_model__level2[*successor_state] = 0.0;
            }    
        }       
    }
}

/******************************************************************************************************
 ******************************************************************************************************
 * INTERFACE TO :: DESCRIPTION OF TRANSITIONS
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/

void Problem_Data::add__transitions____action__start_state__successor_state__UNIFORM(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    number = static_cast<double>(1.0) / static_cast<double>(states.size());
    numbers.push_back(number);
    add__transitions____action__start_state__successor_state_();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__transitions____action__start_state__successor_state__IDENTITY(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    number = (start_state == successor_state)?1.0:0.0;
    numbers.push_back(number);
    add__transitions____action__start_state__successor_state_();
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    
}

void Problem_Data::add__transitions____action__start_state__IDENTITY(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability;
    
    for(auto _successor_state = states.begin()
            ; _successor_state != states.end()
            ; _successor_state++){

        if(*_successor_state == start_state){
            transition_probability =1.0;
        } else {
            transition_probability = 0.0;
        }
        
        numbers.push_back(transition_probability);
    }
    
    add__transitions____action__start_state_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__transitions____action__start_state__UNIFORM(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability = static_cast<double>(1.0) / static_cast<double>(states.size());
    
    for(auto _successor_state = states.begin()
            ; _successor_state != states.end()
            ; _successor_state++){
        
        numbers.push_back(transition_probability);
    }
    
    add__transitions____action__start_state_();

    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__transitions____action__UNIFORM(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability = static_cast<double>(1.0) / static_cast<double>(states.size());
    
    for(auto _starting_state = states.begin()
            ; _starting_state != states.end()
            ; _starting_state++){
        
        for(auto _successor_state = states.begin()
                ; _successor_state != states.end()
                ; _successor_state++){

            numbers.push_back(transition_probability);
        }
    }

    add__transitions____action_();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}


void Problem_Data::add__transitions____action__IDENTITY(const std::string& str)
{
    initialise__transition_model();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);

    double transition_probability;
    
    for(auto _starting_state = states.begin()
            ; _starting_state != states.end()
            ; _starting_state++){
        
        for(auto _successor_state = states.begin()
                ; _successor_state != states.end()
                ; _successor_state++){

            if(*_successor_state == *_starting_state){
                transition_probability = 1.0;
            } else {
                transition_probability = 0.0;
            }
            
            numbers.push_back(transition_probability);
        }
    }

    add__transitions____action_();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}



void Problem_Data::add__transitions____action__start_state__successor_state_()
{
    if("*" == successor_state){

        for(auto _successor_state = states.begin()
                ; _successor_state != states.end()
                ; _successor_state++){
            successor_state = *_successor_state;
            transition_model[action][start_state][successor_state] = number;
        }
        
        successor_state = "*";
    } else {
        transition_model[action][start_state][successor_state] = number;
    }
}

void Problem_Data::add__transitions____action__start_state___successor_state()
{
    if("*" == start_state){

        for(auto _start_state = states.begin()
                ; _start_state != states.end()
                ; _start_state++){
            start_state = *_start_state;
            add__transitions____action__start_state__successor_state_();
        }
        
        start_state = "*";
    } else {
        add__transitions____action__start_state__successor_state_();
    }
}

void Problem_Data::add__transitions____action___start_state__successor_state()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__transitions____action__start_state___successor_state();
        }
        
        action = "*";
    } else {
        add__transitions____action__start_state___successor_state();
    }
}

void Problem_Data::add__transitions____action__start_state__successor_state(const std::string& str)
{
    VERBOSER(20, "Adding transition for "<<action<<" "<<start_state<<" "<<successor_state<<" = "<<number);
    initialise__transition_model();
    add__transitions____action___start_state__successor_state();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__transitions____action__start_state_NUMBERS()
{
    for(auto _successor_state = states.begin()
            ; _successor_state != states.end()
            ; _successor_state++, numbers_index++){
        assert(numbers_index < numbers.size());
                
        decltype(number) number__before = number;
        decltype(successor_state) successor_state__before = successor_state;
                
        successor_state = *_successor_state;
        number = numbers[numbers_index];
        add__transitions____action__start_state__successor_state_();
                
        number = number__before;
        successor_state = successor_state__before;
    }
}

void Problem_Data::add__transitions____action__start_state_()
{
    if("*" == start_state){

        for(auto _start_state = states.begin()
                ; _start_state != states.end()
                ; _start_state++){
            start_state = *_start_state;
            numbers_index = 0;
            add__transitions____action__start_state_NUMBERS();
        }
        
        start_state = "*";
    } else {
        numbers_index = 0;
       add__transitions____action__start_state_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
    
}

void Problem_Data::add__transitions____action___start_state()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
           add__transitions____action__start_state_();
        }
        
        action = "*";
    } else {
       add__transitions____action__start_state_();
    }
}

void Problem_Data::add__transitions____action__start_state(const std::string& str)
{
    VERBOSER(20, ""<<str);
    initialise__transition_model();
    add__transitions____action___start_state();

    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}


void Problem_Data::add__transitions____action_NUMBERS()
{
    
    for(auto _start_state = states.begin()
            ; _start_state != states.end()
            ; _start_state++// , numbers_index++
        ){
        assert(numbers_index < numbers.size());
                
//         decltype(number) number__before = number;
        decltype(start_state) start_state__before = start_state;
                
        start_state = *_start_state;
//         number = numbers[numbers_index];
        add__transitions____action__start_state_NUMBERS();
                
//         number = number__before;
        start_state = start_state__before;
    }
}

void Problem_Data::add__transitions____action_()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            numbers_index = 0;
            add__transitions____action_NUMBERS();
        }
        
        action = "*";
    } else {
        numbers_index = 0;
        add__transitions____action_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__transitions____action(const std::string& str)
{
    VERBOSER(20, ""<<str);
    
    initialise__transition_model();
    add__transitions____action_();

    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}
