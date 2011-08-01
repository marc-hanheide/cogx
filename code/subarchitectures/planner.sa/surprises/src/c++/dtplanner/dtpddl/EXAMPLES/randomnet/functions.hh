#ifndef FUNCTIONS_HH
#define FUNCTIONS_HH


namespace DTPDDL
{
    string functions()
    {
        ostringstream answer;

        answer<<"(:functions (reward) - number )"<<std::endl;
        
        return answer.str();
    }
    
}


#endif
