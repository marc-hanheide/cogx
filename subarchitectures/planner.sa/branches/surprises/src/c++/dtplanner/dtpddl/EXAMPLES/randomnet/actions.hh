#ifndef ACTIONS_HH
#define ACTIONS_HH

namespace DTPDDL
{
    bool isolated_between_faults(Line_Id line, set<Line_Id> visited = set<Line_Id>())
    {
        visited.insert(line);

        if(faulty_lines.find(line) != faulty_lines.end()) return true;
        
        auto& devices = line__to__devices[line];
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){

            if(device->second == circuit_breaker) return false;
            
            auto& adjacent_lines = device__to__line__to__side[*device];
            for(auto _line = adjacent_lines.begin()
                    ; _line != adjacent_lines.end()
                    ; _line++){
                auto new_line = _line->first;
                if(visited.find(new_line) == visited.end()){
                    if(!isolated_between_faults(new_line, visited)){
                        return false;
                    }
                }
            }
        }
        
        return true;
    }
    
    
    string rewards(set<Line_Id>::const_iterator line = lines.begin(), int value = 0)
    {
        ostringstream answer;

//         set<Line_Id> rewardable_lines;
//         for(auto line = lines.begin()
//                 ; line != lines.end()
//                 ; line++){
//             if(faulty_lines.find(*line) != faulty_lines.end()){
//                 if(!isolated_between_faults(*line)){
//                     rewardable_lines.insert(*line);
//                 }
//             }
//         }        
        
        if(line == lines.end()){
            answer<<"(assign (reward) "<<value<<")";
            return answer.str();
        }
        
        set<Line_Id>::const_iterator _line = line;
        set<Line_Id>::const_iterator __line = line;
        _line++;
        __line++;
        
        answer<<"(and ";
        answer<<"(when "
              <<Powered__predicate(to_string(*line))<<" ";
        answer<<rewards(_line, value + 10)
              <<" )"<<std::endl;
        answer<<"(when (not "
              <<Powered__predicate(to_string(*line))<<" ) ";
        answer<<rewards(__line, value)
              <<" )"<<std::endl;
        answer<<") \n";
        
        return answer.str();
    }
    
    
    string implement_open__formula(Line_Id input_line, Device_Id input_device, set<Line_Id> visited = set<Line_Id>())    
    {
        auto& devices = line__to__devices[input_line];

        visited.insert(input_line);

        ostringstream answer;
        answer<<"(and (not "<<Powered__predicate(to_string(input_line))<<" ) "<<std::endl;
        
        ostringstream or_conditions;
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(input_device != *device){

                if(device->second == circuit_breaker){
                    //or_conditions<<"(open "<<DTPDDL::to_string(device)<<" )"<<std::endl;
                    continue;
                }

                auto input_side = device__to__line__to__side[*device][input_line];
                answer<<"(not "<<Source__predicate(to_string(*device), to_string(input_side))<<" ) "<<std::endl;
                
                auto& lines = device__to__line__to__side[*device];
                Line_Id outgoing_line = 0;

                
                for(auto line = lines.begin()
                        ; line != lines.end()
                        ; line++){
                    if(line->first != input_line){
                        outgoing_line = line->first;
                    }
                }
                
                if(outgoing_line == 0){assert(0);}
                
                if(visited.find(outgoing_line) != visited.end()){
                    continue;
                }
                
                visited.insert(outgoing_line);
                auto subformula = implement_open__formula(outgoing_line, *device, visited);
                visited.erase(outgoing_line);
                or_conditions<<"(when (closed "<<DTPDDL::to_string(*device)
                             <<" ) (and (not "<<Powered__predicate(to_string(outgoing_line))<<" ) "
                             // <<"(not "<<Source__predicate(to_string(*device),
                             //                              to_string((input_side == Side::side1)
                             //                                        ?Side::side2
                             //                                        :Side::side1)<<" ) "
                             <<subformula<<"  ) ) "<<std::endl;

                
                // or_conditions<<"(when  "<<Open__predicate(DTPDDL::to_string(*device))
                //              <<" (and "
                //              <<"(not "<<Source__predicate(to_string(*device), to_string(in_side))<<" ) ) ) "<<std::endl;
            }
        }

        string sub_conditions = or_conditions.str();
        
        if(sub_conditions.size()){
            answer<<" "<<sub_conditions<<std::endl;
        }
        
        answer<<" ) "<<std::endl;
        
        return answer.str();
    }
    
    string implement_close__formula(Line_Id input_line, Device_Id input_device, set<Line_Id> visited = set<Line_Id>())
    {
        visited.insert(input_line);
        auto& devices = line__to__devices[input_line];

        ostringstream or_conditions;
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(input_device != *device){

                if(device->second == circuit_breaker){
                    // Nothing to be done when opening a line adjacent a closed circuit breaker. 
                    //or_conditions<<"(open "<<DTPDDL::to_string(device)<<" )"<<std::endl;
                    continue;
                }
                
                
                auto& lines = device__to__line__to__side[*device];
                Line_Id outgoing_line = 0;

                
                for(auto line = lines.begin()
                        ; line != lines.end()
                        ; line++){
                    if(line->first != input_line){
                        outgoing_line = line->first;
                    }
                }
                
                if(outgoing_line == 0){assert(0);}

                
                if(visited.find(outgoing_line) != visited.end()){
                    continue;
                } 


                visited.insert(outgoing_line);
                auto subformula = implement_close__formula(outgoing_line, *device, visited);
                visited.erase(outgoing_line);
                
                if(device__to__side__to__line[*device][side1] == input_line){
                    or_conditions<<Source__predicate(to_string(*device), to_string(side1))<<std::endl;
                } else if (device__to__side__to__line[*device][side2] == input_line){
                    or_conditions<<Source__predicate(to_string(*device), to_string(side2))<<std::endl;
                } else {
                    UNRECOVERABLE_ERROR("Line powers device from no side.");
                }
                
                
                or_conditions<<"(when "<<Closed__predicate(DTPDDL::to_string(*device))
                             <<" (and "<<Powered__predicate(to_string(outgoing_line))<<" "<<subformula<<"  ) ) "<<std::endl;
            }
        }

        string sub_conditions = or_conditions.str();

        ostringstream answer;
        
        if(sub_conditions.size()){
            answer<<"(and "<<sub_conditions<<" ) "<<std::endl;
            
            return answer.str();
        } else {
            return "";
        }
    }
    
    string safe_close__formula(Line_Id input_line, Device_Id input_device, set<Line_Id> visited = set<Line_Id>())
    {
        visited.insert(input_line);
        
        auto& devices = line__to__devices[input_line];
        
        ostringstream or_conditions;
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(input_device != *device){

                cerr<<to_string(*device)<<" ";
                
                if(device->second == circuit_breaker){
                    cerr<<Open__predicate(DTPDDL::to_string(*device))<<std::endl;
                    
                    or_conditions<<Open__predicate(DTPDDL::to_string(*device))<<std::endl;
                    continue;
                }
                
                
                auto& lines = device__to__line__to__side[*device];
                Line_Id outgoing_line = 0;


                assert(lines.size() <= 2);
                
                for(auto line = lines.begin()
                        ; line != lines.end()
                        ; line++){
                    if(line->first != input_line){
                        outgoing_line = line->first;
                    }
                }
                
                if(outgoing_line == 0){assert(0);}


                if(visited.find(outgoing_line) != visited.end()){
                    continue;
                } 


                visited.insert(outgoing_line);
                auto subformula = safe_close__formula(outgoing_line, *device, visited);
                visited.erase(outgoing_line);
                
                or_conditions<<"(or "<<Open__predicate(DTPDDL::to_string(*device))
                             <<" (and (not "<<Faulty__predicate(to_string(outgoing_line))<<") "<<subformula<<"  ) ) "<<std::endl;
            }
        }

        string sub_conditions = or_conditions.str();

        ostringstream answer;
        
        if(sub_conditions.size()){
            answer<<"(and "<<sub_conditions<<" ) "<<std::endl;
            
            return answer.str();
        } else {
            return "";
        }
    }
    
    

    string close__switch_side_unpowered__action(Device_Id device,
                                                Line_Id powered_line,
                                                Line_Id unpowered_line,
                                                Side side)
    {
        assert(device.second == remote_switch);
        
        ostringstream answer;
        
        answer<<"(:action "<<to_string(side)<<"-close-switch-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;
        answer<<":precondition (and "<<Open__predicate(to_string(device))<<std::endl;
        answer<<"                   "<<Powered__predicate(to_string(powered_line))<<std::endl;
        answer<<"                   (not "<<Powered__predicate(to_string(unpowered_line))<<" )"<<std::endl;
        answer<<"                   (not "<<Faulty__predicate(to_string(unpowered_line))<<")"<<std::endl;
        answer<<""<<std::endl;
        answer<<safe_close__formula(unpowered_line, device)<<std::endl;
        answer<<"       "<<std::endl;        
        answer<<")"<<std::endl;
        answer<<":effect (and (not "<<Open__predicate(to_string(device))<<") "
              <<Closed__predicate(to_string(device))<<std::endl
              <<Powered__predicate(to_string(unpowered_line))<<std::endl;
        answer<<implement_close__formula(unpowered_line, device)<<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//         answer<<rewards()<<std::endl;
        answer<<"   ) "<<std::endl;      
        answer<<")"<<std::endl;

        return answer.str();
    }

    
    string close__switch__action(Device_Id device)
    {
        ostringstream answer;
        assert(device.second == remote_switch);

        auto side1_line = device__to__side__to__line[device][side1];
        auto side2_line = device__to__side__to__line[device][side2];


        answer<<"(:action basic-close-switch-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;
        answer<<":precondition (and (not "<<Powered__predicate(to_string(side1_line))<<") (not "
              <<Powered__predicate(to_string(side2_line))<<") "<<Open__predicate(to_string(device))<<" )"<<std::endl;
        answer<<""<<std::endl;
        answer<<":effect (and "<<Closed__predicate(to_string(device))
              <<" (not "<<Open__predicate(to_string(device))<<" ) "<<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//               <<rewards()<<std::endl;
        answer<<")"<<std::endl;
        answer<<""<<std::endl;
        answer<<")"<<std::endl;


        
        answer<<close__switch_side_unpowered__action(device, side2_line, side1_line, side1);
        
        answer<<close__switch_side_unpowered__action(device, side1_line, side2_line, side2);

        return answer.str();
    }
    
    string close__circuit_breaker__action(Device_Id device)
    {
        ostringstream answer;
        assert(device.second == circuit_breaker);

        assert(device__to__side__to__line[device].find(side1) == device__to__side__to__line[device].end() ||
               device__to__side__to__line[device].find(side2) == device__to__side__to__line[device].end());


        auto line = (device__to__side__to__line[device].find(side1) != device__to__side__to__line[device].end())
            ?(device__to__side__to__line[device][side1])
            :(device__to__side__to__line[device][side2]);
        
        answer<<"(:action basic-close-circuit-breaker-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;

        answer<<":precondition (and (not "<<Powered__predicate(to_string(line))<<")"<<std::endl
              <<Open__predicate(to_string(device))<<std::endl
              <<" (not "<<Faulty__predicate(to_string(line))<<" ) "<<std::endl;

        
        answer<<safe_close__formula(line, device)<<std::endl
              <<")"<<std::endl;
        answer<<""<<std::endl;
        
        answer<<":effect (and "<<Closed__predicate(to_string(device))<<std::endl
              <<" (not "<<Open__predicate(to_string(device))<<" )"<<std::endl
              <<Powered__predicate(to_string(line))<<std::endl
              <<implement_close__formula(line, device)<<std::endl
              <<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//               <<rewards()<<std::endl;
        answer<<")"<<std::endl;
        answer<<""<<std::endl;
        answer<<")"<<std::endl;

        return answer.str();
    }
    
    string close_actions()
    {
        ostringstream answer;
        
        for(auto _device = device__to__side__to__line.begin()
                ; _device != device__to__side__to__line.end()
                ; _device++){
            auto& device = _device->first;

            if(_device->first.second == circuit_breaker){
                answer<<close__circuit_breaker__action(device)<<std::endl;
            } else if(_device->first.second == remote_switch) {
                answer<<close__switch__action(device)<<std::endl;
            } else {
                UNRECOVERABLE_ERROR("Unknown device");
            }
        }

        return answer.str();
    }
    


    string open__circuit_breaker__action(Device_Id device)
    {
        assert(device.second == circuit_breaker);
        
        assert(device__to__side__to__line[device].find(side1) == device__to__side__to__line[device].end() ||
               device__to__side__to__line[device].find(side2) == device__to__side__to__line[device].end());


        auto line = (device__to__side__to__line[device].find(side1) != device__to__side__to__line[device].end())
            ?(device__to__side__to__line[device][side1])
            :(device__to__side__to__line[device][side2]);

        ostringstream answer;
        
        answer<<"(:action open-circuit-breaker-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;
        answer<<":precondition (and "
              <<Closed__predicate(to_string(device))<<" )"<<std::endl;
        answer<<""<<std::endl;
        answer<<":effect (and (not "<<Closed__predicate(to_string(device))<<") "
              <<Open__predicate(to_string(device))<<std::endl;
        answer<<implement_open__formula(line, device)<<std::endl;
        answer<<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//               <<rewards()<<std::endl;
        answer<<")"<<std::endl;
        answer<<")"<<std::endl;

        return answer.str();
        
    }




    string open__switch__action(Device_Id device)
    {
        ostringstream answer;
        
        assert(device.second == remote_switch);

        auto side1_line = device__to__side__to__line[device][side1];
        auto side2_line = device__to__side__to__line[device][side2];
        
        
        
        answer<<"(:action basic-open-switch-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;
        answer<<":precondition (and (not "<<Powered__predicate(to_string(side1_line))<<") (not "
              <<Powered__predicate(to_string(side2_line))<<") "
              <<Closed__predicate(to_string(device))<<" )"<<std::endl;
        answer<<""<<std::endl;
        answer<<":effect (and (not "<<Closed__predicate(to_string(device))<<") "
              <<Open__predicate(to_string(device))<<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//               <<rewards()<<std::endl
        answer<<" )"<<std::endl;
        answer<<""<<std::endl;
        answer<<")"<<std::endl;


        
        answer<<"(:action open-switch-"<<to_string(device)<<std::endl;
        answer<<""<<std::endl;
        answer<<":parameters ()"<<std::endl;
        answer<<":precondition (and "<<Powered__predicate(to_string(side1_line))<<" "
              <<Powered__predicate(to_string(side2_line))<<" "
              <<Closed__predicate(to_string(device))<<" )"<<std::endl;
        answer<<""<<std::endl;
        answer<<":effect (and (not "<<Closed__predicate(to_string(device))<<") "
              <<Open__predicate(to_string(device))<<std::endl;
        answer<<"( when "<<Source__predicate(to_string(device), to_string(side1))<<" "
              <<implement_open__formula(side2_line, device)<<" ) "<<std::endl;
        answer<<"( when "<<Source__predicate(to_string(device), to_string(side2))<<" "
              <<implement_open__formula(side1_line, device)<<" ) "<<std::endl;
        answer<<std::endl;
        answer<<"(assign (reward) 0)"<<std::endl;
//               <<rewards()<<std::endl
        answer<<" ) "<<std::endl;
        answer<<")"<<std::endl;

        return answer.str();
    }
    
    string open_actions()
    {
        ostringstream answer;
        
        for(auto _device = device__to__side__to__line.begin()
                ; _device != device__to__side__to__line.end()
                ; _device++){
            auto& device = _device->first;

            if(_device->first.second == circuit_breaker){
                answer<<open__circuit_breaker__action(device)<<std::endl;
            } else if(_device->first.second == remote_switch) {
                answer<<open__switch__action(device)<<std::endl;
            } else {
                UNRECOVERABLE_ERROR("Unknown device");
            }
        }

        return answer.str();
    }
    
    
}



#endif
