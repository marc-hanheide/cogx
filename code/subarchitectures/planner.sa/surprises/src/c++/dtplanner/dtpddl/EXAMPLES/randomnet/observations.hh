#ifndef OBSERVATIONS_HH
#define OBSERVATIONS_HH

namespace DTPDDL
{
    string observation()
    {
        ostringstream answer;
        
    
        answer<<"(:observe observe-powered-lines "<<std::endl;
        answer<<":parameters () "<<std::endl;
        answer<<":execution (or ";

        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(device->second == circuit_breaker){
                answer<<"(basic-close-circuit-breaker-"
                      <<to_string(*device)<<")"<<std::endl;
            }
        }
        
        answer<<" ) "<<std::endl;
        answer<<":precondition () "<<std::endl;
         
         answer<<":effect (and  ";

         
        for(auto line = lines.begin()
                ; line != lines.end()
                ; line++){
            answer<<"(when "<<Powered__predicate(to_string(*line))<<" "
                  <<O_Powered__predicate(to_string(*line))<<" )";
        }
        
        answer<<") "<<std::endl;
        answer<<") "<<std::endl;
        
        return answer.str();
    }

    string percepts()
    {
        ostringstream answer;

        answer<<"(:percepts "<<std::endl;
        answer<<O_Powered__predicate()<<std::endl;
        answer<<" ) "<<std::endl;
        
        return answer.str();
    }
    
    
}

#endif
