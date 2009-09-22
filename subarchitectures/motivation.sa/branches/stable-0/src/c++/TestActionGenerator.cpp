#include "TestActionGenerator.hpp"
#include <autogen/Execution.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>


extern "C" {
  cast::interfaces::CASTComponentPtr 
  newComponent() {
    return new execution::TestActionGenerator();
  }
}

namespace execution {

  using namespace cast;
  using namespace slice;
  using namespace slice::actions;


  void TestActionGenerator::start() {
    //just a test for recent bugs
    addChangeFilter(createLocalTypeFilter<Action>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<TestActionGenerator>(this,
									  &TestActionGenerator::actionOverwrite));
    
    
  }

  void TestActionGenerator::actionOverwrite(const cast::cdl::WorkingMemoryChange &_wmc) {
    println("seen action overwrite");
  }


  void TestActionGenerator::runComponent() {

//     sleepComponent(10000);
//     println("triggering");
//     GoToPlacePtr gtp = new GoToPlace();
//     gtp->placeID = 2;
//     gtp->status = PENDING;
//     gtp->success = TRIINDETERMINATE;
//     addToWorkingMemory(newDataID(), gtp);
  }

}
