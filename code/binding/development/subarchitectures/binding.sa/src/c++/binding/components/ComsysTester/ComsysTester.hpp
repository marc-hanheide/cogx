#ifndef COMSYS_TESTER_HPP_
#define COMSYS_TESTER_HPP_

#include <map>
#include <set>
#include <string>
#include <comsys/idl/ComsysEssentials.hh>
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Predicates.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/idl/BindingFeatures.hh>
#include <binding/BindingException.hpp>
#include <binding/feature-utils/AbstractFeature.hpp>
#include <binding/abstr/AbstractBinder.hpp>
#include <cast/core/CASTCore.hpp>
#include <balt/interface/BALTTimer.hpp>
#include "binding/feature-specialization/helpers/SalienceHelper.hpp"


namespace Binding {

/// a component which writes discrefs to comsys, and checks the
/// resulting proxies etc.
class ComsysTester : 
    public AbstractBinder {
  /// the test number
  int m_test;
  
  /// the latest registered status of the binder
  BindingData::BinderStatus m_status;
    /// the id of the status
  std::string m_statusID;

  std::string m_comsysID;
  /// all proxies currently on WM (as far as this component knows)
  std::set<std::string> m_proxiesOnWM;
  /// all proxies signalled as added
  std::set<std::string> m_addSignalledProxyIDs;

  BindingGraphHandler m_handler;

  bool m_testFinished;

public:
  ComsysTester(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  
protected:
  void bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
  void statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void somethingInterestingUpdated(const cast::cdl::WorkingMemoryChange & _wmc);

  void taskAdopted(const std::string& _taskID){}
  void taskRejected(const std::string& _taskID){}
  void runComponent();
  
  /// exits and signals success
/// exits and signals success
  void successExit() {
    const_cast<ComsysTester&>(*this).sleepProcess(100); // sleep for a while and test completeness again (10 times)
    if(m_retest++ < 10) {
      std::cout << "retesting #" << m_retest << " at time "
		<< BALTTimer::getBALTTime() << std::endl;
      testCompleteness();
    }
    addToWorkingMemory(newDataID(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    sleepProcess(12000); // sleep for a while to allow dotviewer to finish
    std::cout << "\nSUCCESS" << std::endl;
    ::exit(cast::cdl::testing::CAST_TEST_PASS);
  }
  
  /// exits and signals failure
  void failExit() {
    log("\nFAILURE\n");
    addToWorkingMemory(newDataID(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    sleepProcess(2000); // sleep for a while to allow dotviewer to finish
    //::exit(cast::cdl::testing::CAST_TEST_FAIL);
    abort();
  }
  
  /// tests if the binding is now complete and exits with a success
  /// signal if so.
  void testCompleteness();
  /// returns true if all added proxies are bound
  bool allBound();

private:
  /// writes the sentence to comsys's WM
  void passToComsys(const std::string& _sentence);

  unsigned int m_retest;  

  std::string m_dotTitle;

  void  trigger_dot() {
    BindingData::TriggerDotViewerWithTitle* trigger = new BindingData::TriggerDotViewerWithTitle;
    trigger->m_title = CORBA::string_dup(m_dotTitle.c_str());
    addToWorkingMemory(newDataID(), trigger, cast::cdl::BLOCKING);
    addToWorkingMemory(newDataID(), "motiv.sa", trigger, cast::cdl::BLOCKING);
  }
  
}; // ComsysTester
} // namespace Binding

#endif // TESTER_MONITOR_H_

