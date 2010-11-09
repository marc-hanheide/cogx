#ifndef PREDICATES_HH
#define PREDICATES_HH


namespace DTPDDL
{
    
    string Faulty__predicate()
    {
        ostringstream answer;
        answer<<"(Faulty ?x - "<<Line__typestring()<<")"<<std::endl;
        return answer.str();
    }
    
    string Source__predicate()
    {
        ostringstream answer;
        answer<<"(Source ?x1 - "<<Device__typestring()
              <<" ?x2 - "<<Side__typestring()<<" )"<<std::endl;
        return answer.str();
    }
    
    string Powered__predicate()
    {
        ostringstream answer;
        answer<<"(Powered ?x - "<<Line__typestring()<<")"<<std::endl;
        return answer.str();
    }
    
    string O_Powered__predicate()
    {
        ostringstream answer;
        answer<<"(O-Powered ?x - "<<Line__typestring()<<")"<<std::endl;
        return answer.str();
    }
    
    string Open__predicate()
    {
        ostringstream answer;
        answer<<"(Open ?x - "<<Device__typestring()<<")"<<std::endl;
        return answer.str();
    }
    
    string Closed__predicate()
    {
        ostringstream answer;
        answer<<"(Closed ?x - "<<Device__typestring()<<")"<<std::endl;
        return answer.str();
    }




    
    
    string Faulty__predicate(const string& str)
    {
        ostringstream answer;
        answer<<"(Faulty "<<str<<")";
        return answer.str();
    }
    
    string Source__predicate(const string& str1, const string& str2)
    {
        ostringstream answer;
        answer<<"(Source "<<str1<<" "<<str2<<")";
        return answer.str();
    }
    
    string Powered__predicate(const string& str)
    {
        ostringstream answer;
        answer<<"(Powered "<<str<<")";
        return answer.str();
    }
    
    string O_Powered__predicate(const string& str)
    {
        ostringstream answer;
        answer<<"(O-Powered "<<str<<")";
        return answer.str();
    }
    
    string Open__predicate(const string& str)
    {
        ostringstream answer;
        answer<<"(Open "<<str<<")";
        return answer.str();
    }
    
    string Closed__predicate(const string& str)
    {
        ostringstream answer;
        answer<<"(Closed "<<str<<")";
        return answer.str();
    }


    string predicates()
    {
        ostringstream answer;

        answer<<"(:predicates "<<std::endl;
        answer<<Closed__predicate()<<std::endl;
        answer<<Open__predicate()<<std::endl;
        answer<<Faulty__predicate()<<std::endl;
        answer<<Source__predicate()<<std::endl;
        answer<<Powered__predicate()<<std::endl;
        answer<<" ) "<<std::endl;
        
        return answer.str();
    }
    
}


#endif
