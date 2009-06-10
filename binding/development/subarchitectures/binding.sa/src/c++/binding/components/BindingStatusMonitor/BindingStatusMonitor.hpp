#ifndef BINDING_BINDING_STATUS_MONITOR_H_
#define BINDING_BINDING_STATUS_MONITOR_H_


#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include <ctime>
#include <ext/hash_set>
#include "binding/abstr/AbstractBinder.hpp"
#include "binding/idl/BindingData.hh"

namespace Binding {

//typedef std::set<std::string> StringSet;
typedef __gnu_cxx::hash_set<std::string> StringSet;


/// When a Monitor tells the binding sbarch that it should bind a set
/// of proxies, the BindingStatusMonitor creates scoring tasks for
/// the BindingJudge.
class BindingStatusMonitor : public AbstractBinder {
public:
  BindingStatusMonitor(const std::string &_id);
  virtual ~BindingStatusMonitor();

  virtual void runComponent(){
    m_stateID = newDataID();
    
    m_lastStableTime = clock();
    addToWorkingMemory(m_stateID,
		       //BindingLocalOntology::BINDER_STATUS_TYPE, 
		       _newState());
    lockEntry(m_stateID,cast::cdl::LOCKED_OD);
  };
  void start();
  
  void scoringTaskAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingTaskAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void scoringTaskRemoved(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingTaskRemoved(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  //void bindingUnionUpdatedOrAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  void bindingProxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
  
protected:
  int m_scoringTasks;
  int m_bindingTasks;
  
  std::string m_stateID;

  clock_t m_lastStableTime;
  clock_t m_totalUnstableTime;
  bool m_lastValue;
  
  //virtual void workingMemoryChanged(const cast::cdl::WorkingMemoryChangeList & _wmcl){};
  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  virtual void configure(std::map<std::string,std::string>& _config);
private:
  void _updateState();
  BindingData::BinderStatus* _newState(); 
  
  // IDs of unbound proxies
  StringSet m_unbound;
  // IDs of bound proxies
  StringSet m_bound;
};

} // namespace Binding

#endif // BINDING_BINDING_STATUS_MONITOR_H_
