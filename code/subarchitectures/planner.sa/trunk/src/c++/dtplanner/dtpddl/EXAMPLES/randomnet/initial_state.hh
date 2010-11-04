#ifndef INITIAL_STATE_HH
#define INITIAL_STATE_HH


namespace DTPDDL
{

    string stochastic_faulty()
    {
        ostringstream answer;

        UNRECOVERABLE_ERROR("UNIMPLEMENTED");
        
        return answer.str();
    }
    
    
    string deterministic_fault()
    {
        ostringstream answer;

        for(auto line = faulty_lines.begin()
                ; line != faulty_lines.end()
                ; line++){
            answer<<Faulty__predicate(to_string(*line));
        }
        
        
        return answer.str();
    }    
    
    string initial_state()
    {
        ostringstream answer;
        answer<<"(:init "<<std::endl;

        if(INITIALLY__POWERED__lines.size() == 0){
            std::cerr<<"Warning, no initially powered lines."<<std::endl;
        }
        
        
        for(auto line = INITIALLY__POWERED__lines.begin()
                ; line != INITIALLY__POWERED__lines.end()
                ; line++){
            answer<<Powered__predicate(to_string(*line));
        }

        
        for(auto source = INITIALLY__SOURCE__remote_switch__side.begin()
                ; source != INITIALLY__SOURCE__remote_switch__side.end()
                ; source++){

            for(auto p = source->second.begin()
                    ; p != source->second.end()
                    ; p++){
                answer<<Source__predicate(to_string(source->first),
                                          to_string(*p));
            }
        }

        for(auto device = devices.begin()
                ; device != devices.end()
                ; device++){
            if(closed_devices.find(*device) != closed_devices.end()){
                answer<<Closed__predicate(to_string(*device))<<std::endl;
            } else {
                answer<<Open__predicate(to_string(*device))<<std::endl;
            }
        }
        
//         answer<<stochastic_fault();
        answer<<deterministic_fault();

        
        answer<<")"<<std::endl;
        return answer.str();
        
    }

    
}


#endif
