#include "test_harness__CAST_SCAT_client_clone1.hh"


extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new test_harness__CAST_SCAT_client_clone1();
  }
}

test_harness__CAST_SCAT_client_clone1::test_harness__CAST_SCAT_client_clone1(const Designator&designator)
    :Implement(designator)
{
}


void test_harness__CAST_SCAT_client_clone1::configure(const std::map<std::string,std::string>& arguments)
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

void* unlikely_to_clash_with_anything__0095591_1(void* arg)
{
    auto thing = static_cast<test_harness__CAST_SCAT_client_clone1*>(arg);

    thing->eachThread__test2();

    return 0;
}

void test_harness__CAST_SCAT_client_clone1::eachThread__test2()
{
    auto result =
        call_to_subarchitecture
        <testHarnessCASTSCAT::helloWorld> /* SLICE name of call.*/
        (different_subarchitecture_name 
         , std::string("") 
         , Designators());

}

void test_harness__CAST_SCAT_client_clone1::runComponent()
{
    /* Upper bound on counting tests.*/
    const auto MAX = 3;
    
    CAST__VERBOSER(1000, "Test one :hello-world: 1a -- STARTING");
    
    auto result =
        call_to_subarchitecture
        <testHarnessCASTSCAT::helloWorld> /* SLICE name of call.*/
        (different_subarchitecture_name 
         , std::string("") 
         , Designators());

    CAST__VERBOSER(1000, "Test one :hello-world: 1a -- DONE");


    CAST__VERBOSER(1000, "Test one :hello-world: 1b -- STARTING");
    
    for(auto i = 0; i < MAX; i++){
        auto result =
            call_to_subarchitecture
            <testHarnessCASTSCAT::helloWorld> /* SLICE name of call.*/
            (different_subarchitecture_name 
             , std::string("") 
             , Designators());
    }
    
    CAST__VERBOSER(1000, "Test one :hello-world: 1b -- DONE");
    
    
    
    CAST__VERBOSER(1000, "Test two :thread-safety: 2 -- STARTING");

    
    std::vector<pthread_t> threads(MAX);
    std::vector<pthread_attr_t> thread_attributes(MAX);
    
    for(auto i = 0; i < MAX; i ++){
        pthread_attr_init(&thread_attributes[i]);
        pthread_create(&threads[i],0,unlikely_to_clash_with_anything__0095591_1, this);
    }

    for(auto i = 0; i < MAX; i++){
        pthread_join(threads[i], 0);
        pthread_attr_destroy(&thread_attributes[i]);
    }
    
    CAST__VERBOSER(1000, "All threads have executed and are.. done... :: ");
    
    
    CAST__VERBOSER(1000, "Test two :thread-safety: 2 -- DONE");


    CAST__VERBOSER(1000, "Test three :Fibonacci-served: 3a -- STARTING");
    
    auto _answer5 =
        call_to_subarchitecture
        <testHarnessCASTSCAT::computeFibonacci> /* SLICE name of call.*/
        (different_subarchitecture_name 
         , 5 /* Input */
         , 0 /* Return value */
         , Designators());

    int answer5 = _answer5->answerIsAReturn;
    
    CAST__VERBOSER(1000, "Fibonacci number 5 is (expecting 5) ... :: "<<answer5);
    
    CAST__VERBOSER(1000, "Test three :Fibonacci-served: 3a -- DONE");
    
    CAST__VERBOSER(1000, "Test three :Fibonacci-served: 3b -- STARTING");
    
    auto _answer7 =
        call_to_subarchitecture
        <testHarnessCASTSCAT::computeFibonacci> /* SLICE name of call.*/
        (different_subarchitecture_name 
         , 15 /* Input */
         , 0 /* Return value */
         , Designators());

    int answer7 = _answer7->answerIsAReturn;
    
    CAST__VERBOSER(1000, "Fibonacci number 15 is (expecting 610) ... :: "<<answer7);
    
    CAST__VERBOSER(1000, "Test three :Fibonacci-served: 3b -- DONE");
}

void test_harness__CAST_SCAT_client_clone1::implement__computeFibonacci(testHarnessCASTSCAT::computeFibonacciPtr& input)
{
}


void test_harness__CAST_SCAT_client_clone1::start()
{
}


