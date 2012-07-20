#ifndef PREPROCESSING_HH
#define PREPROCESSING_HH

string type_to_string(Device_Type dt)
{
    if(dt == circuit_breaker){
        return "cb";
    } else if (dt == remote_switch) {
        return "sd";
    }
    else {
        UNRECOVERABLE_ERROR("Invalid type.");
    }

    return "";
}

int side_to_int(Side side)
{
    switch(side){
        case Side::side1:
            return 1;
            break;
        case Side::side2:
            return 2;
            break;
        default:
            UNRECOVERABLE_ERROR("Invalid side.");
            break;
    }
    return 0;
}


namespace DTPDDL
{
    string to_string(Device_Id id)
    {
        ostringstream answer;
        answer<<type_to_string(id.second)<<id.first;

        return answer.str();
    }
    
    string to_string(Line_Id id)
    {
        ostringstream answer;
        answer<<"l"<<id;

        return answer.str();
    }
    
    string to_string(Side s)
    {
        ostringstream answer;
        
        if(s == side1){
            answer<<"side1";
        } else if (s == side2) {
            answer<<"side2";
        } else {
            UNRECOVERABLE_ERROR("Invalid side.");
        }
        
        return answer.str();
    }
}


bool complete_unary__POWERED__line(Line_Id line_Id,
                                   Device_Id source,
                                   set<Line_Id>& powered_lines,
                                   map<Device_Id, Side>& source_relation,
                                   set<Line_Id>& visited)
{
    std::cerr<<"visiting line :: "<<line_Id<<std::endl;
    
    if(visited.find(line_Id) != visited.end())return true;
    visited.insert(line_Id);

    
    if(faulty_lines.find(line_Id) != faulty_lines.end()){
        std::cerr<<"Got a faulty line on the feeder with source :: "<<source.second<<" "<<source.first<<std::endl;
        
        powered_lines.erase(powered_lines.begin(), powered_lines.end());
        return false;
    }

    powered_lines.insert(line_Id);
    
    if(line__to__devices.find(line_Id) == line__to__devices.end()) return true;
    
    auto& devices = line__to__devices[line_Id];

    for(auto device = devices.begin()
            ; device != devices.end()
            ; device++){

        if(*device == source) continue;
        
        if(device->second == circuit_breaker){
            if(closed_devices.find(*device) != closed_devices.end()){
                powered_lines.erase(powered_lines.begin(), powered_lines.end());

                std::cerr<<"Two circuit breakers closed onto the same line."<<std::endl;
                
                return false;
            }
        }

        std::cerr<<"Testing switch :: "<<device->first<<std::endl;
        
        if(closed_devices.find(*device) != closed_devices.end()){
            auto& lines__to__sides =  device__to__line__to__side[*device];

            source_relation[*device] = device__to__line__to__side[*device][line_Id];
                
            for(auto next = lines__to__sides.begin()
                    ; next != lines__to__sides.end()
                    ; next++ ){
                if(next->first != line_Id){
                    bool result = complete_unary__POWERED__line(next->first,
                                                                *device,
                                                                powered_lines,
                                                                source_relation,
                                                                visited);

                    if(!result) return false;
                }
            }
        }
    }

    std::cerr<<"Got "<<powered_lines.size()<<" powered lines."<<std::endl;
    
    return true;
}


void complete_unary__POWERED__one_circuit_breaker(Device_Id device)
{
    if(device__to__side__to__line.find(device)
       == device__to__side__to__line.end()) return;

    if(closed_devices.find(device) == closed_devices.end()){/*IT'S OPEN*/
        return;
    }
    
    assert(device__to__side__to__line[device].size() == 1);

    auto& sides_to_lines = device__to__side__to__line[device];

    std::cerr<<" Device :: "<<device.second<<" "<<device.first<<" has line count :: "<<sides_to_lines.size()<<std::endl;
    
    for(auto sides_to_line = sides_to_lines.begin()
            ; sides_to_line != sides_to_lines.end()
            ; sides_to_line++){
        auto line_Id = sides_to_line->second;

        set<Line_Id> powered_lines;
        map<Device_Id, Side> source_relation;
        set<Line_Id> initial_visited;
        if(complete_unary__POWERED__line(line_Id, device, powered_lines, source_relation, initial_visited)){
            for(auto powered_line = powered_lines.begin()
                    ; powered_line != powered_lines.end()
                    ; powered_line++){
                
                INITIALLY__POWERED__lines.insert(*powered_line);
                if(INITIALLY__POWERED_BY__lines__circuit_breakers.find(*powered_line)
                   == INITIALLY__POWERED_BY__lines__circuit_breakers.end()){
                    INITIALLY__POWERED_BY__lines__circuit_breakers[*powered_line] = set<Device_Id>();
                }
                
                INITIALLY__POWERED_BY__lines__circuit_breakers[*powered_line].insert(device);
                
                if(INITIALLY__POWERED_BY__lines__circuit_breakers[*powered_line].size() > 1){
                    INITIALLY__POWERED__lines.erase(*powered_line);
                }   
            }
            
        }

        for(auto p = source_relation.begin()
                ; p != source_relation.end()
                ; p++){
            if(INITIALLY__SOURCE__remote_switch__side.find(p->first) ==
               INITIALLY__SOURCE__remote_switch__side.end()){
                INITIALLY__SOURCE__remote_switch__side[p->first] = std::set<Side>();
            }
            
            INITIALLY__SOURCE__remote_switch__side[p->first].insert(p->second);
        }
    }
}

void complete_unary__POWERED()
{
    for(auto device = devices.begin()
            ; device != devices.end()
            ; device++){
        if(device->second == circuit_breaker){
            std::cerr<<"Got a circuit breaker :: "<<device->first<<std::endl;
            
            complete_unary__POWERED__one_circuit_breaker(*device);
        }
    }

    for(auto device = devices.begin()
            ; device != devices.end()
            ; device++){
        if(device->second == remote_switch){/*SWITCH*/
            if(closed_devices.find(*device) == closed_devices.end()){/*OPEN*/
                /*If the adjacent line is powered, then write Source
                 * predicate to the initial state.*/
                
                if(device__to__side__to__line[*device].find(Side::side1) !=
                   device__to__side__to__line[*device].end()){
                    auto line = device__to__side__to__line[*device][Side::side1];
                    if(INITIALLY__POWERED__lines.find(line) != INITIALLY__POWERED__lines.end()){
                        
                        if(INITIALLY__SOURCE__remote_switch__side.find(*device) ==
                           INITIALLY__SOURCE__remote_switch__side.end()){
                            INITIALLY__SOURCE__remote_switch__side[*device] = std::set<Side>();
                        }
                        INITIALLY__SOURCE__remote_switch__side[*device].insert(Side::side1);
                    }
                }
                
                if(device__to__side__to__line[*device].find(Side::side2) !=
                   device__to__side__to__line[*device].end()){
                    auto line = device__to__side__to__line[*device][Side::side2];
                    if(INITIALLY__POWERED__lines.find(line) != INITIALLY__POWERED__lines.end()){
                        if(INITIALLY__SOURCE__remote_switch__side.find(*device) ==
                           INITIALLY__SOURCE__remote_switch__side.end()){
                            INITIALLY__SOURCE__remote_switch__side[*device] = std::set<Side>();
                        }
                        INITIALLY__SOURCE__remote_switch__side[*device].insert(Side::side2);
                    }
                }
            }
        }
    }
}

bool dangerous_opened(Device_Id device, set<Line_Id> lines = set<Line_Id>())
{
    if(closed_devices.find(device) == closed_devices.end()){
        return false;
    }

    auto& __lines = device__to__line__to__side[device];
    for(auto _line = __lines.begin()
            ; _line != __lines.end()
            ; _line++){
        auto line = _line->first;
        
        if(faulty_lines.find(line) != faulty_lines.end()){
            return true;
        }
    }
    
    for(auto _line = __lines.begin()
            ; _line != __lines.end()
            ; _line++){
        auto line = _line->first;

        if(lines.find(line) != lines.end()) continue;
        lines.insert(line);
        
        auto& devices = line__to__devices[line];
        for(auto _device = devices.begin()
                ; _device != devices.end()
                ; _device++){
            if(*_device != device){
                if(closed_devices.find(*_device) != closed_devices.end()){   
                    if(dangerous_opened(*_device, lines)){
                        return true;
                    }
                }
            }
        }
        
    }

    return false;
}

void open_breakers_on_broken_feeders()
{
    
    for(auto device = devices.begin()
            ; device != devices.end()
            ; device++){
        if(device->second == circuit_breaker){
            if(closed_devices.find(*device) != closed_devices.end()){
                if(dangerous_opened(*device)){
                    closed_devices.erase(*device);
                }
            }
        }
    }
}


void preprocess()
{
    open_breakers_on_broken_feeders();
    
    complete_unary__POWERED();
}


#endif
