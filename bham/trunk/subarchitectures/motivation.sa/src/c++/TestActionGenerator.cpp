#include "TestActionGenerator.hpp"
#include <autogen/Execution.hpp>


extern "C" {
  cast::interfaces::CASTComponentPtr 
  newComponent() {
    return new execution::TestActionGenerator();
  }
}

namespace execution {

  using namespace slice;
  using namespace slice::actions;

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
