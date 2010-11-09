#ifndef PREFIX_HH
#define PREFIX_HH

namespace DTPDDL
{
    string domain_name()
    {
        ostringstream _domain_name;
        _domain_name<<"PSR-DOMAIN-CB"<<number_of_circuit_breakers<<"-L"<<number_of_lines<<"-SD"<<number_of_switches;
        return _domain_name.str();
    }
    
    string problem_name()
    {
        ostringstream _domain_name;
        _domain_name<<"PSR-PROBLEM-CB"<<number_of_circuit_breakers<<"-L"<<number_of_lines<<"-SD"<<number_of_switches;
        return _domain_name.str();
    }
    
    
    string domain_postfix()
    {
        return ")";/*END OF DOMAIN DEFINITION*/
    }
    
    string domain_prefix()
    {
        ostringstream answer;
        
        answer<<"(define (domain "<<domain_name()<<")"<<std::endl;
        answer<<""<<std::endl;
        answer<<"(:requirements "<<std::endl;
        answer<<";;    IPC6 elements -- Planning off the ground"<<std::endl;
        answer<<":typing ;; Keywords :: \":types\""<<std::endl;
        answer<<":strips "<<std::endl;
        answer<<":equality"<<std::endl;
        answer<<":fluents "<<std::endl;
        answer<<"   "<<std::endl;
        answer<<";;    Uncertainty track at IPC6"<<std::endl;
        answer<<":probabilistic-effects"<<std::endl;
        answer<<"   "<<std::endl;
        answer<<";;    PDDL syntactic sugar"<<std::endl;
        answer<<";;    Keywords :: "<<std::endl;
        answer<<":universal-effects  "<<std::endl;
        answer<<":conditional-effects  "<<std::endl;
        answer<<"   "<<std::endl;
        answer<<";;    DTP-ELEMENT"<<std::endl;
        answer<<";;    Keywords :: \":percepts\", \":observe\", and \":execution\""<<std::endl;
        answer<<":partial-observability "<<std::endl;
        answer<<""<<std::endl;
        answer<<"    )"<<std::endl;
        
        return answer.str();
    }


    string problem_postfix()
    {
        
        ostringstream answer;

        answer<<"(:metric maximize (reward) )"<<std::endl;

        answer<<"(:goal (and "<<std::endl;
        if(soft_goal){
            for(auto line = lines.begin()
                    ; line != lines.end()
                    ; line++){
                answer<<"(preference 1 "<<Powered__predicate(to_string(*line))<<" )"<<std::endl;
            }
        } else {
            for(auto line = lines.begin()
                    ; line != lines.end()
                    ; line++){
                answer<<""<<Powered__predicate(to_string(*line))<<std::endl;
            }
        }
        
        
        answer<<"))"<<std::endl;

        
        answer<<")"<<std::endl;/*END OF PROBLEM DEFINITION*/
        
        return answer.str();
    }
    
    
    string problem_prefix()
    {
        ostringstream answer;
        
        answer<<"(define (problem "<<problem_name()<<" ) "<<std::endl;
        answer<<"(:domain "<<domain_name()<<" ) "<<std::endl;

        return answer.str();
    }
}


#endif
