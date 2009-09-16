
#ifndef test_harness__CAST_SCAT_ICE
#define test_harness__CAST_SCAT_ICE

module testHarnessCASTSCAT
{    
    ["c++:type:vector<string>"] sequence<string> OptionalMemberDesignator;

    /*Test recursion.*/
    class computeFibonacci
    {   
        int ithNumberIsAnArgument;
        int answerIsAReturn;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
    };
    
    /*Testing procedure calling.*/
    class helloWorld
    {
        ["c++:type:std::string"] string thingToPrintIsAnArgument;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
    };   
}
;

#endif
