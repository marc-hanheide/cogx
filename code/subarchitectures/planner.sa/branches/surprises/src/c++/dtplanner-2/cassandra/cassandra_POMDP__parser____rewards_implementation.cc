
#include "cassandra_POMDP__parser.hh"

using namespace POMDP::Parsing;

using std::cin;


using std::istringstream;
using std::ostringstream;

void Problem_Data::initialise__reward_model()
{
    if(already__called__initialise__reward_model) {return;}
    already__called__initialise__reward_model = true;
    
    assert(actions.size());
    assert(states.size());
    assert(observations.size());

    for(auto action = actions.begin()
            ; action != actions.end()
            ; action++){
        
        reward_model[*action] = decltype(reward_model.find(*action)->second)();

        decltype(reward_model[*action])& reward_model__level1
            = reward_model[*action];
        
        for(auto start_state = states.begin()
                ; start_state != states.end()
                ; start_state++){
            
            reward_model__level1[*start_state]
                = decltype(reward_model__level1.find(*start_state)->second)();
        
            decltype(reward_model__level1[*start_state])& reward_model__level2
                = reward_model__level1[*start_state];
            
            for(auto successor_state = states.begin()
                    ; successor_state != states.end()
                    ; successor_state++){

                reward_model__level2[*successor_state]
                    = decltype(reward_model__level2.find(*successor_state)->second)();
                
                decltype(reward_model__level2[*successor_state])& reward_model__level3
                    = reward_model__level2[*successor_state];
                
//                 reward_model__level2[*start_state]
//                     = decltype(reward_model__level2.find(*action)->second)();
                
//                 decltype(reward_model__level2[*start_state])& reward_model__level3
//                     = reward_model__level2[*start_state];
            
                for(auto observation = observations.begin()
                        ; observation != observations.end()
                        ; observation++){
                    
                    reward_model__level3[*observation] = 0.0;
                    
                }
            }
        }   
    }
}
/******************************************************************************************************
 ******************************************************************************************************
 * INTERFACE TO :: DESCRIPTION OF REWARDS
 ******************************************************************************************************
 ******************************************************************************************************
 ******************************************************************************************************/




void Problem_Data::add__rewards____action__start_state__successor_state__observation_()
{
    if("*" == observation){
        
        for(auto _observation = observations.begin()
                ; _observation != observations.end()
                ; _observation++){
            observation = *_observation;
            
            if(number != 0){
                VERBOSER(1, "Reward :: "<<action<<" "<<start_state<<" "<<successor_state<<" "<<observation<<" = "<<number<<std::endl);
            }

            
            QUERY_UNRECOVERABLE_ERROR(find(actions.begin(), actions.end(), action) == actions.end()
                                  , "action :: "<<action<<" was not found for some reason in the set of actions.\n");
            
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), start_state) == states.end()
                                  , "state :: "<<start_state<<" was not found for some reason in the set of states.\n");
            
            QUERY_UNRECOVERABLE_ERROR(find(states.begin(), states.end(), successor_state) == states.end()
                                  , "state :: "<<successor_state<<" was not found for some reason in the set of states.\n");
            
            QUERY_UNRECOVERABLE_ERROR(find(observations.begin(), observations.end(), observation) == observations.end()
                                  , "observation :: "<<observation<<" was not found for some reason in the set of observations.\n");
            

            
            reward_model[action][start_state][successor_state][observation] = number;
        }
        
       observation = "*";
    } else {
        
        if(number != 0){
            VERBOSER(1, "Reward :: "<<action<<" "<<start_state<<" "<<successor_state<<" "<<observation<<" = "<<number<<std::endl);
        }
        reward_model[action][start_state][successor_state][observation] = number;
    }
    

    
}

void Problem_Data::add__rewards____action__start_state__successor_state___observation()
{
    if("*" == successor_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            successor_state = *_state;
            add__rewards____action__start_state__successor_state__observation_();
        }
        
        successor_state = "*";
    } else {
        add__rewards____action__start_state__successor_state__observation_();
    }
}

void Problem_Data::add__rewards____action__start_state___successor_state__observation()
{
    if("*" == start_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            start_state = *_state;
            add__rewards____action__start_state__successor_state___observation();
        }
        
        start_state = "*";
    } else {
            add__rewards____action__start_state__successor_state___observation();
    }
}

    
void Problem_Data::add__rewards____action___start_state__successor_state__observation()
{
    if("*" == action){

        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__rewards____action__start_state___successor_state__observation();
        }
        
        action = "*";
    } else {
        add__rewards____action__start_state___successor_state__observation();
    }
}


void Problem_Data::add__rewards____action__start_state__successor_state__observation(const std::string& str)
{
    initialise__reward_model();
    add__rewards____action___start_state__successor_state__observation();

    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__rewards____action__start_state__successor_state_NUMBERS()
{
    
    for(auto _observation = observations.begin()
            ; _observation != observations.end()
            ; _observation++, numbers_index++){
        assert(numbers_index < numbers.size());
                
        decltype(number) number__before = number;
        decltype(observation) observation__before = observation;
                
        observation = *_observation;
        number = numbers[numbers_index];
        add__rewards____action__start_state__successor_state__observation_();
                
        number = number__before;
        observation = observation__before;
    }
}

void Problem_Data::add__rewards____action__start_state__successor_state_()
{
    
    if("*" == successor_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            successor_state = *_state;
            numbers_index = 0;
            add__rewards____action__start_state__successor_state_NUMBERS();
        }
        successor_state = "*";
    } else {
        numbers_index = 0;
        add__rewards____action__start_state__successor_state_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__rewards____action__start_state___successor_state()
{
   
    if("*" == start_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            start_state = *_state;
            add__rewards____action__start_state__successor_state_();
        }
        
        start_state = "*";
    } else {
        add__rewards____action__start_state__successor_state_();
    } 
}

void Problem_Data::add__rewards____action___start_state__successor_state()
{
    if("*" == action){
        
        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__rewards____action__start_state___successor_state();
        }
        
        action = "*";
    } else {
        add__rewards____action__start_state___successor_state();
    }
}


void Problem_Data::add__rewards____action__start_state__successor_state(const std::string& str)
{
    initialise__reward_model();
    add__rewards____action___start_state__successor_state();
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__rewards____action__start_state_NUMBERS()
{
    for(auto _state = states.begin()
            ; _state != states.end()
            ; _state++){
        decltype(successor_state) successor_state__before = successor_state ;
        successor_state = *_state;

        add__rewards____action__start_state__successor_state_NUMBERS();
        
//         for(auto _observation = observations.begin()
//                 ; _observation != observations.end()
//                 ; _observation++, numbers_index++){
//             assert(numbers_index < numbers.size());
                
//             decltype(number) number__before = number;
//             decltype(observation) observation__before = observation;
                
//             observation = *_observation;
//             number = numbers[numbers_index];
//             add__rewards____action__start_state__successor_state__observation_();
                
//             number = number__before;
//             observation = observation__before;
//         }

        successor_state = successor_state__before;
    }
}

void Problem_Data::add__rewards____action__start_state_()
{
    
    if("*" == start_state){

        for(auto _state = states.begin()
                ; _state != states.end()
                ; _state++){
            start_state = *_state;
            
            numbers_index = 0;
            add__rewards____action__start_state_NUMBERS();
        }
        
        start_state = "*";
    } else {
        
        numbers_index = 0;
        add__rewards____action__start_state_NUMBERS();
    }
    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}

void Problem_Data::add__rewards____action___start_state()
{
    
    if("*" == action){
        
        for(auto _action = actions.begin()
                ; _action != actions.end()
                ; _action++){
            action = *_action;
            add__rewards____action__start_state_();
        }
        
        action = "*";
    } else {
        add__rewards____action__start_state_();
    }
}

void Problem_Data::add__rewards____action__start_state(const std::string& str)
{
    initialise__reward_model();
    add__rewards____action___start_state();

    
    /*numbers--RESET -- The list of numbers last parsed has been used.*/
    numbers = decltype(numbers)(0);
}



            
