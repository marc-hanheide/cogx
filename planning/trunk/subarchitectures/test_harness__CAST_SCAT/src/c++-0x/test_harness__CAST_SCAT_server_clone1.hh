#ifndef TEST_HARNESS__CAST_SCAT_SERVER_CLONE1_HH
#define TEST_HARNESS__CAST_SCAT_SERVER_CLONE1_HH




#include <cast/architecture.hpp>

//#include <HelloWorldData.hpp>

#include "testHarnessCASTSCAT.hpp"


#include "/data/private/grettonc/CogX/SVN/cogx/code/planning/trunk/include/CAST_SCAT/cast_scat.hh"

using CAST_SCAT::Designator;
using CAST_SCAT::Designators;




class test_harness__CAST_SCAT_server_clone1 :
    public CAST_SCAT::procedure_implementation<test_harness__CAST_SCAT_server_clone1>,
    public CAST_SCAT::procedure_call<>
{
public:
    typedef CAST_SCAT::procedure_implementation<test_harness__CAST_SCAT_server_clone1> Implement;
    typedef CAST_SCAT::procedure_call<> Call;

    /* argc and argv*/
    void configure(const std::map<std::string,std::string>& );
    
    //HelloWriter();
    test_harness__CAST_SCAT_server_clone1 (const Designator& = "");

    
    void implement__computeFibonacci(testHarnessCASTSCAT::computeFibonacciPtr&);
    void implement__helloWorld(testHarnessCASTSCAT::helloWorldPtr&);
    
    void runComponent();
protected:
    std::string different_subarchitecture_name;
    void start();
};



#endif
