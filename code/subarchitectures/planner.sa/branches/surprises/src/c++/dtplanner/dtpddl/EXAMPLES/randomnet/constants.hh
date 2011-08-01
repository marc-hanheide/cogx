#ifndef CONSTANTS_HH
#define CONSTANTS_HH


namespace DTPDDL
{
    string constants()
    {
        ostringstream answer;
        
        answer<<"(:constants   "<<std::endl;

        answer<<to_string(Side::side1)<<" "
              <<to_string(Side::side2)<<" - "<<Side__typestring()<<std::endl;

        for(auto line = lines.begin()
                ; line != lines.end()
                ; line++){
            answer<<to_string(*line)<<" ";
        }
        answer<<" - "<<Line__typestring()<<std::endl;
        
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(device->second == Device_Type::circuit_breaker){
                answer<<to_string(*device)<<" ";
            }
        }
        answer<<" - "<<Circuit_Breaker__typestring()<<std::endl;
        
        
        
        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(device->second == Device_Type::remote_switch){
                answer<<to_string(*device)<<" ";
            }
        }
        
        answer<<" - "<<Remote_Switch__typestring()<<std::endl;
        
        answer<<" ) "<<std::endl;

        return answer.str();
    }
    
}


#endif
