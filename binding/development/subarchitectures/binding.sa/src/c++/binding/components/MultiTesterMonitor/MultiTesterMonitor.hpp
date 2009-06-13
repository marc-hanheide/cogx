#ifndef MULTI_TESTER_MONITOR_H_
#define MULTI_TESTER_MONITOR_H_

#include "binding/components/TesterMonitor/TesterMonitor.hpp"

namespace Binding {

/// a monitor which creates proxies and then does some simple basic
/// sanity testing.
class MultiTesterMonitor : 
    public TesterMonitor
{
protected:
  
public:
  MultiTesterMonitor(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  
  virtual void runComponent();
  /// only does sth if postConditionTester is true
  virtual void testCompleteness();
  
  /// if true, this component will test the postcondition
  bool postConditionTester;
  /// each multitester needs an instance number (so that they can
  /// write slightly different things)
  unsigned int instance;
  /// the total number of testers (primarily important information for the postConditionTester)
  unsigned int noOfMultiTesters;
  
  /// returns the number of moniitors that think they are ready for post condition testing
  static unsigned int& readyCount() {
    static unsigned int n(0);
    return n;
  }
}; // MultiTesterMonitor
} // namespace Binding

#endif // MULTI_TESTER_MONITOR_H_

