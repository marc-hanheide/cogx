#ifndef OBJECTS_HH
#define OBJECTS_HH


namespace DTPDDL
{
    string objects()
    {
        
        ostringstream answer;

        answer<<"(:objects "<<std::endl;

//         for(auto device = devices.begin()
//                 ; device != devices.end()
//                 ; device++){
//             if(device->second == circuit_breaker){
//                 answer<<to_string(*device)<<" ";
//             }
//         }
//         answer<<" - "<<Circuit_Breaker__typestring()<<std::endl;

//         for(auto device = devices.begin()
//                 ; device != devices.end()
//                 ; device++){
//             if(device->second == remote_switch){
//                 answer<<to_string(*device)<<" ";
//             }
//         }
//         answer<<" - "<<Remote_Switch__typestring()<<std::endl;


//         for(auto line = lines.begin()
//                 ; line != lines.end()
//                 ; line++){
//             answer<<to_string(*line)<<" ";
//         }
//         answer<<" - "<<Line__typestring()<<std::endl;
        
        
//         answer<<to_string(Side::side1)<<" "
//               <<to_string(Side::side2)<<" - "<<Side__typestring()<<std::endl;

        answer<<")"<<std::endl;
        
        return answer.str();
    }
    
    
}


#endif
