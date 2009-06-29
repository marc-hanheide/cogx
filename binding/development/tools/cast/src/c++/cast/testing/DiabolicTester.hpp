#ifndef DIABOLIC_TESTER_HPP_
#define DIABOLIC_TESTER_HPP_

#include <cast/testing/AbstractTester.hpp>

#include <map>
#include <set>
#include <string>

namespace cast {

namespace testing {
/// a component which writes discrefs to diabolic, and checks the
/// resulting proxies etc.
class DiabolicTester : 
    public PrivilegedManagedProcess {

public:
  DiabolicTester(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  
  void taskAdopted(const std::string& _taskID){}
  void taskRejected(const std::string& _taskID){}
  void runComponent();
  
  /// exits and signals success
  void successExit() const {
    const_cast<DiabolicTester&>(*this).sleepProcess(2000); // sleep for a while to see if anything crashes
    std::cout << "\nSUCCESS" << std::endl;
    sleep(1);
    ::exit(cast::cdl::testing::CAST_TEST_PASS);
  }
  /// exits and signals failure
  void failExit() const {
    std::cout << "\nFAILURE" << std::endl;
    abort();
  }  

  void dummyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void dummyOverwritten(const cast::cdl::WorkingMemoryChange & _wmc);

private:
  /// reader or writer mode... the writer will write a lot, and the readers read a lot
  enum {
    READER,
    WRITER
  } m_mode;
  
  /// whether or not to use locking (instead of blocking overwrites)
  bool m_locking;
  
  /// number of entries to write (default 1000) in the reader, \p m_n is used to decide when to reload data
  unsigned int m_n;
  /// the ID of the constantly overwritten dummy
  std::string m_dummyID;
  
  /// the cache which is used to load the dummy (is allocated at first add)
  std::auto_ptr<cast::CachedCASTData<cast::cdl::testing::TestDummyStruct> > m_dummyCache;
}; // DiabolicTester

} // namespace testing
} // namespace cast

#endif // TESTER_MONITOR_H_

