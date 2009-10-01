#include "test_harness__CAST_SCAT_server_clone1.hh"



extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new test_harness__CAST_SCAT_server_clone1();
  }
}

test_harness__CAST_SCAT_server_clone1::test_harness__CAST_SCAT_server_clone1(const Designator&designator)
    :Implement(designator)
{
}


void test_harness__CAST_SCAT_server_clone1::configure(const std::map<std::string,std::string>& arguments)
{
    auto _subarchitecture_name = arguments.find("--different-subarchitecture-name");
    if(arguments.end() != _subarchitecture_name){
        different_subarchitecture_name
        = std::move(_subarchitecture_name->second);
    } else {
        different_subarchitecture_name = getSubarchitectureID();
        CAST__WARNING("There was no CAST-architecture-string specified for the subsystem. This is problematic, as we are required to know where the requests for implementations should be sent. At the moment, we have recovered with :: "<<different_subarchitecture_name<<". that should be the name of our subarchitecture."<<std::endl);
        
    }
}

void test_harness__CAST_SCAT_server_clone1::runComponent()
{
}

void test_harness__CAST_SCAT_server_clone1::implement__computeFibonacci(testHarnessCASTSCAT::computeFibonacciPtr& input)
{
    int n = input->ithNumberIsAnArgument;

    
    if(1 == n){
        
        CAST__VERBOSER(199, " ** Computed Fibonacci of :: "<<n<<" := "<<n);
        input->answerIsAReturn = n;
        
        return;
    }
    
    auto _one_back =
        call
        <testHarnessCASTSCAT::computeFibonacci> /* SLICE name of call.*/
        (n-1 /* Input */
         , 0 /* Return value */
         , Designators());
    
    auto one_back = _one_back->answerIsAReturn;
    
    auto two_back = 0;
    if(0 != (n-2)){
        
        auto _two_back =
            call
            <testHarnessCASTSCAT::computeFibonacci> /* SLICE name of call.*/
            (n-2 /* Input */
             , 0 /* Return value */
             , Designators());

        two_back = _two_back->answerIsAReturn;
    }
    
    input->answerIsAReturn = one_back + two_back;
    
    CAST__VERBOSER(199, " ## Computed Fibonacci of :: "<<n<<" := "<<input->answerIsAReturn);
    return;
}

void test_harness__CAST_SCAT_server_clone1::implement__helloWorld(testHarnessCASTSCAT::helloWorldPtr& input)
{
    CAST__VERBOSER(199, "Hello world!\n");
}

    

void test_harness__CAST_SCAT_server_clone1::start()
{
    implement<testHarnessCASTSCAT::helloWorld>(&test_harness__CAST_SCAT_server_clone1::implement__helloWorld);
    
    implement<testHarnessCASTSCAT::computeFibonacci>(&test_harness__CAST_SCAT_server_clone1::implement__computeFibonacci);
    
}


