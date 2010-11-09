#ifndef INITIAL_STATE_HH
#define INITIAL_STATE_HH


namespace DTPDDL
{
    template<typename T>
    set<set<T>> saturate(set<set<T>> input)
    {
        set<set<T>>  answer;
        
        for(auto in = input.begin()
                ; in != input.end()
                ; in ++){
            answer.insert(*in);

            for(auto _in = in
                    ; _in != input.end()
                    ; _in ++){
                if(in == _in) continue;
                set<T> n = *in;
                n.insert(_in->begin(), _in->end());
                answer.insert(n);
            }
        }

        if(answer != input){
            auto _answer = saturate(answer);
            _answer.insert(set<T>());

            return _answer;
        }
        
       return answer; 
    }
    
    
    string stochastic_fault()
    {

        int fake_fault_probability = 10;
        if(command_Line_Arguments.got_guard("--fake-faults")){
            fake_fault_probability = command_Line_Arguments.get_int();
        } else {
            cerr<<"No probability given for the personal"<<std::endl
                <<"belief in a good line being faulty."<<std::endl
                <<"Using :: "<<fake_fault_probability<<std::endl
                <<"--fake-faults switch would specify an integer between 0 and 100"<<std::endl;
        }
        
        ostringstream answer;

        set<Line_Id> fake_faulty;

        for(auto line = lines.begin()
                ; line != lines.end()
                ; line++){
            if(INITIALLY__POWERED__lines.find(*line) == INITIALLY__POWERED__lines.end()){
                if(faulty_lines.find(*line) == faulty_lines.end()){
                    if( ( random() % 100 ) < fake_fault_probability){
                        fake_faulty.insert(*line);
                    }
                }
            }
        }
        
        set<Line_Id> fault_atoms;
        fault_atoms.insert(fault_atoms.begin(), fault_atoms.end());
        fault_atoms.insert(faulty_lines.begin(), faulty_lines.end());
        
        set<set<Line_Id>> all_belief_atoms;
        for(auto line = fault_atoms.begin()
                ; line != fault_atoms.end()
                ; line++){
            set<Line_Id> tmp;
            tmp.insert(*line);
            all_belief_atoms.insert(tmp);
        }
        
        all_belief_atoms = saturate(all_belief_atoms);

        double state_mass = 1.0 / static_cast<double>(all_belief_atoms.size());

        if(all_belief_atoms.size()){ 
            answer<<" (probabilistic ";
            for(auto atom = all_belief_atoms.begin()
                    ; atom != all_belief_atoms.end()
                    ; atom++){
                answer<<state_mass<<" (and ";

                for(auto p = atom->begin()
                        ; p != atom->end()
                        ; p++){   
                    answer<<Faulty__predicate(to_string(*p));
                }
            
                answer<<" )"<<std::endl;
            }
            answer<<")"<<std::endl;
        }
        
        
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

        if(command_Line_Arguments.is_argument("deterministic")){
            answer<<deterministic_fault();
        } else {
            answer<<stochastic_fault();
        }
        
// //         answer<<stochastic_fault();
//         answer<<deterministic_fault();

        
        answer<<")"<<std::endl;
        return answer.str();
        
    }

    
}


#endif
