#ifndef TEST_HARNESS__CAST_SCAT_CLIENT_CLONE2_HH
#define TEST_HARNESS__CAST_SCAT_CLIENT_CLONE2_HH


#include <cast/architecture.hpp>

//#include <HelloWorldData.hpp>

#include "testHarnessCASTSCAT.hpp"


#include "/data/private/grettonc/CogX/SVN/cogx/code/planning/trunk/include/CAST_SCAT/cast_scat.hh"

using CAST_SCAT::Designator;
using CAST_SCAT::Designators;


class test_harness__CAST_SCAT_client_clone2 :
    public CAST_SCAT::procedure_implementation<test_harness__CAST_SCAT_client_clone2, CAST_SCAT::Locality::Local>,
    public CAST_SCAT::procedure_call<CAST_SCAT::_recover_address, CAST_SCAT::Locality::Local>
{
public:
    typedef CAST_SCAT::procedure_implementation<test_harness__CAST_SCAT_client_clone2, CAST_SCAT::Locality::Local> Implement;
    typedef CAST_SCAT::procedure_call<CAST_SCAT::_recover_address, CAST_SCAT::Locality::Local> Call;

    /* argc and argv*/
    void configure(const std::map<std::string,std::string>& );
    
    //HelloWriter();
    test_harness__CAST_SCAT_client_clone2 (const Designator& = "");

    
    void implement__computeFibonacci(testHarnessCASTSCAT::computeFibonacciPtr&);
    
    void runComponent();

    void eachThread__test2();
protected:
    std::string different_subarchitecture_name;
    void start();
};




#endif
