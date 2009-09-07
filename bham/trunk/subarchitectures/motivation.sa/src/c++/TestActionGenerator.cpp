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

  void TestActionGenerator::runComponent() {
    PrintMessagePtr pm = new PrintMessage();
    pm->message = "Oh no, oh my.";
    pm->status = autogen::Planner::PENDING;
    pm->success = TRIINDETERMINATE;
    addToWorkingMemory(newDataID(), pm);
  }

}
