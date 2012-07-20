#ifndef TYPES_HH
#define TYPES_HH


namespace DTPDDL
{
    string Device__typestring()
    {
        return "Device";
    }
    
    string Circuit_Breaker__typestring()
    {
        return "CircuitBreaker";
    }
    
    string Remote_Switch__typestring()
    {
        return "RemoteSwitch";
    }

    string Line__typestring()
    {
        return "Line";
    }
    
    string Side__typestring()
    {
        return "Side";
    }
    
    
    string types()
    {
        ostringstream answer;
        
        answer<<"(:types  "<<std::endl;

        answer<<Device__typestring()<<" - object"<<std::endl;
        answer<<Circuit_Breaker__typestring()<<" "
              <<Remote_Switch__typestring()
              <<" - "<<Device__typestring()<<std::endl;
        answer<<Line__typestring()<<" - object"<<std::endl;
        answer<<Side__typestring()<<" - object"<<std::endl;
        
        answer<<")";

        return answer.str();
    }
}


#endif
