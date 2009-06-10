#ifndef TESTER_MONITOR_H_
#define TESTER_MONITOR_H_

#include <map>
#include <set>
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Predicates.hpp>
#include <binding/idl/BindingData.hh>
#include <binding/idl/BindingFeatures.hh>
#include <binding/idl/BindingQueries.hh>
#include <binding/BindingException.hpp>
#include <binding/feature-utils/AbstractFeature.hpp>
#include <binding/abstr/AbstractMonitor.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <cast/core/CASTCore.hpp>


namespace Binding {

/// a monitor which creates proxies and then does some simple basic
/// sanity testing.
class TesterMonitor : 
    public AbstractMonitor
    //public AbstractBindingWMRepresenter 
{
protected:
   /// the test number
  int m_test;
  /// the proxies that are stored and need to be checked before exiting
  std::set<std::string> m_proxyIDs;
  /// the proxies which have been signalled as added
  std::set<std::string> m_addSignalledProxyIDs;
  /// the proxies which have been signalled as overwritten
  std::set<std::string> m_overwriteSignalledProxyIDs;

  /// the latest registered status of the binder
  BindingData::BinderStatus m_status;
  /// the id of the status
  std::string m_statusID;
  
  /// the IDs of the queries, in the order they were stated
  std::vector<std::string> m_basicQueryIDs;
  /// the IDs of the queries, in the order they were stated
  std::vector<std::string> m_advancedQueryIDs;
  /// stores the answered queries 
  std::map<std::string,BindingQueries::BasicQuery> m_basicQueryAnswers;
  /// stores the answered queries 
  std::map<std::string,BindingQueries::AdvancedQuery> m_advancedQueryAnswers;
  
  /// set to true when all proxies involved in the test have been
  /// created (default is true, as in most cases the test will not
  /// abort until really finished anyway)
  bool m_testFinished;

  BindingGraphHandler m_handler;
  
public:
  TesterMonitor(const std::string &_id);

  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  
protected:
  void bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  void statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc);  
  /// just calls testCompleteness
  void somethingInterestingUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  /// when a BasicQuery is overwritten, it is an answer to be analysed
  void basicQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc);
  void advancedQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc);

  void taskAdopted(const std::string& _taskID){}
  void taskRejected(const std::string& _taskID){}
  virtual void runComponent();
  
  /// exits and signals success
  void successExit() {
    const_cast<TesterMonitor&>(*this).sleepProcess(100); // sleep for a while and test completeness again (10 times)
    if(m_retest++ < 10) {
      std::cout << "retesting #" << m_retest << std::endl;
      testCompleteness();
    }
    addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    const_cast<TesterMonitor&>(*this).sleepProcess(8000); // sleep for a while to allow dotviewer etc to finish
    std::cout << "\nSUCCESS" << std::endl;
    ::exit(cast::cdl::testing::CAST_TEST_PASS);
  }
  /// exits and signals failure
  void failExit() {
    
    addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    const_cast<TesterMonitor&>(*this).sleepProcess(8000); // sleep for a while to allow dotviewer to finish
    std::cout << "\nFAILURE" << std::endl;
    //::exit(cast::cdl::testing::CAST_TEST_FAIL);
    abort();
  }
  
  /// tests if the binding is now complete and exits with a success
  /// signal if so.
  virtual void testCompleteness();
  /// returns true if all added proxies are bound
  virtual bool allBound();
  virtual bool allBound(const std::set<std::string>&);
  /// returns true if proxies have been added, and all of them are
  /// deleted
  bool binderEmptied();

  /// exits only after all proxies in the set are bound
  void awaitBinding(const ProxySet& _proxies);
  void awaitBinding(const std::set<std::string>& _proxies);

protected:
  /// returns id of relation
  std::string addTwoProxiesAndOneRelation(const std::string& _concept1 = "test_concept1", 
					  const std::string& _concept2 = "test_concept2", 
					  const std::string& _relation_label = "test_label");
  unsigned int m_retest;
  /// number of times a BinderStatus was added/updated
  unsigned int m_statusUpdates;
  /// number of times a stable BinderStatus was added/updated
  unsigned int m_statusStableUpdates;
}; // TesterMonitor
} // namespace Binding

#endif // TESTER_MONITOR_H_

