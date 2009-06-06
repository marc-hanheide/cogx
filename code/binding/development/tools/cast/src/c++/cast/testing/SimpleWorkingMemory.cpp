#include "SimpleWorkingMemory.hpp"

namespace cast {
namespace testing {
  using namespace std;

SimpleWorkingMemory::SimpleWorkingMemory(const string & _id) : 
  SubarchitectureWorkingMemory(_id) 
{
  setSendXarchChangeNotifications(true);
}

} // namespace testing
} // namespace cast

extern "C" {
  FrameworkProcess* newComponent(const std::string &_id) {
    return new cast::testing::SimpleWorkingMemory(_id);
  }
}
