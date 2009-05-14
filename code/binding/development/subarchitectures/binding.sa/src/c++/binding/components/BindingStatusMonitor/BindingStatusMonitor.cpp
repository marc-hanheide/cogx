#include "binding/utils/BindingUtils.hpp"
#include "BindingStatusMonitor.hpp"
#include "binding/feature-utils/Features.hpp"
#include "binding/utils/BindingUtils.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/assign.hpp>


using namespace std;
using namespace boost;
using namespace boost::assign;
using namespace cast;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
     return new Binding::BindingStatusMonitor(_id);
  }
}

namespace Binding {

BindingStatusMonitor::BindingStatusMonitor(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  m_scoringTasks(0),
  m_bindingTasks(0),
  m_totalUnstableTime(0),
  m_lastValue(true)
{ 
  m_queueBehaviour = cdl::QUEUE;
  m_stateID = "WRONG_ID! you added a proxy before the binder got properly initialized.";
}

void
BindingStatusMonitor::start() {

  AbstractBinder::start();

  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingTask>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::bindingTaskAdded));
  
		  
  addChangeFilter(createLocalTypeFilter<BindingData::ScoringTask>(cdl::ADD),
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::scoringTaskAdded));

  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingTask>(cdl::DELETE), 
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::bindingTaskRemoved));

  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::bindingProxyAdded));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::bindingProxyUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,
									 &BindingStatusMonitor::bindingProxyDeleted));
  
  addChangeFilter(createLocalTypeFilter<BindingData::ScoringTask>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<BindingStatusMonitor>(this,									 
									 &BindingStatusMonitor::scoringTaskRemoved));
}

void 
BindingStatusMonitor::scoringTaskAdded(const cdl::WorkingMemoryChange & _wmc) 
{
  m_scoringTasks++;
  _updateState();
}

void 
BindingStatusMonitor::bindingTaskAdded(const cdl::WorkingMemoryChange & _wmc) 
{
  m_bindingTasks++;
  _updateState();
}

void 
BindingStatusMonitor::scoringTaskRemoved(const cdl::WorkingMemoryChange & _wmc)
{
  m_scoringTasks--;
  assert(m_scoringTasks >= 0);
  _updateState();
}

void 
BindingStatusMonitor::bindingTaskRemoved(const cdl::WorkingMemoryChange & _wmc) 
{
  m_bindingTasks--;
  assert(m_bindingTasks >= 0);
  _updateState();
}

void 
BindingStatusMonitor::bindingProxyAdded(const cdl::WorkingMemoryChange & _wmc)
{
  unsigned int size(m_unbound.size());
  m_unbound.insert(string(_wmc.m_address.m_id));
  assert(size != m_unbound.size());
  _updateState();
}

/*void 
BindingStatusMonitor::bindingUnionUpdatedOrAdded(const cdl::WorkingMemoryChange & _wmc)
{
  
  const LBindingUnion& binding_union(m_unionLocalCache[string(_wmc.m_address.m_id)]);
    
  unsigned int size(m_unbound.size());
  for(unsigned int i = 0 ; i < binding_union->m_proxyIDs.length() ; ++i) {
    string id(binding_union->m_proxyIDs[i]);
    m_unbound.erase(id);
    m_bound.insert(id);
  }
  if(size != m_unbound.size()) {
    _updateState();
  }
}
*/
void 
BindingStatusMonitor::bindingProxyUpdated(const cdl::WorkingMemoryChange & _wmc)
{
      
  unsigned int size(m_unbound.size());
  
  string id(_wmc.m_address.m_id);
  try {
    if(m_proxyLocalCache[id]->m_proxyState == BindingData::BOUND) {
      m_unbound.erase(id);
      m_bound.insert(id);
    }
    if(size != m_unbound.size()) {
      _updateState();
    }
  }
  catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in BindingStatusMonitor: " + string(_e.what()));    
  }
}

void 
BindingStatusMonitor::bindingProxyDeleted(const cdl::WorkingMemoryChange & _wmc)
{
//  unsigned int size(m_unbound.size());
  m_unbound.erase(string(_wmc.m_address.m_id));
  m_bound.erase(string(_wmc.m_address.m_id));
  //if(size != m_unbound.size()) {
  _updateState();
  //}  
}
  

BindingData::BinderStatus*
BindingStatusMonitor::_newState() { 
  BindingData::BinderStatus* state = new BindingData::BinderStatus();
  if(//m_bindingTasks == 0 && m_scoringTasks == 0 && 
     m_unbound.empty()) {
    state->m_stable = true;
    m_lastValue = true;
  }  else {
    if(m_lastValue)
      m_lastStableTime = clock();
    state->m_stable = false;
    m_lastValue = false;
  }

  state->m_scoringTasks   = m_scoringTasks;
  state->m_bindingTasks   = m_bindingTasks;
  state->m_unboundProxies = m_unbound.size();
  state->m_boundProxies   = m_bound.size();

//#define SLOW_STATUS_MONITORING
#ifdef SLOW_STATUS_MONITORING
  vector<shared_ptr<const CASTData<BindingData::BindingTask> > > bindingtasks;
  getWorkingMemoryEntries(BindingLocalOntology::BINDING_TASK_TYPE,
			  0,
			  bindingtasks);
  vector<shared_ptr<const CASTData<BindingData::ScoringTask> > > scoringtasks;
  getWorkingMemoryEntries(BindingLocalOntology::SCORING_TASK_TYPE,
			  0,
			  scoringtasks);

  
  log("   scoringTasks: " + lexical_cast<string>(m_scoringTasks) + "(" + 
      lexical_cast<string>(scoringtasks.size()) + ")");
  log("   bindingTasks: " + lexical_cast<string>(m_bindingTasks)+ "(" + 
      lexical_cast<string>(bindingtasks.size()) + ")");
  log("unbound proxies: " + lexical_cast<string>(state->m_unboundProxies));
  log("  bound proxies: " + lexical_cast<string>(state->m_boundProxies));

  log("         stable: " + lexical_cast<string>(state->m_stable));
#else // NDEBUG

  log("   scoringTasks: " + lexical_cast<string>(m_scoringTasks));
  log("   bindingTasks: " + lexical_cast<string>(m_bindingTasks));
  {
    stringstream str;
    for(StringSet::const_iterator i = m_unbound.begin() ; i != m_unbound.end() ; ++i) {
      str << *i << ", ";
    }
    log("unbound proxies: " + lexical_cast<string>(state->m_unboundProxies) + " (" + str.str() + ")");
  }
  {
    stringstream str;
    for(StringSet::const_iterator i = m_bound.begin() ; i != m_bound.end() ; ++i) {
      str << *i << ", ";
    }
    log("  bound proxies: " + lexical_cast<string>(state->m_boundProxies) + " (" + str.str() + ")");
  }

  log("         stable: " + lexical_cast<string>(state->m_stable));
#endif
  if(state->m_stable) {
    unsigned int current_time = clock();
    time_t dt = current_time - m_lastStableTime;
    m_totalUnstableTime += dt;
    log("              T: " + lexical_cast<string>((double)(dt) / (double)CLOCKS_PER_SEC));
    log("  sum binding T: " + lexical_cast<string>((double)(m_totalUnstableTime) / (double)CLOCKS_PER_SEC));

    //nah: if not logging, then just print a 1-liner
    if(!m_bLogOutput) {
      println("stable with %i bound proxies",static_cast<int>(state->m_boundProxies));
    }

  }


  return state;
}

void
BindingStatusMonitor::_updateState() 
{  
  try {
    // to pass the consistency check, the data must be loaded before
    // it's written. It's not strictly necessary for any other purpose
    // in this particular case.
//    getWorkingMemoryEntry<BindingData::BinderStatus>(m_stateID); 
    
    BindingData::BinderStatus* state = _newState();
    overwriteWorkingMemory(m_stateID,
			   //BindingLocalOntology::BINDER_STATUS_TYPE, 
			   state);
//			   cdl::BLOCKING);
    
  } catch (DoesNotExistOnWMException& _e ) {
    cerr << "caught this in BindingStatusMonitor::_updateState: " << _e.what() << endl 
	 << "while trying lo load binder status on address \""<< m_stateID << "\"";
    abort();
  }

}

BindingStatusMonitor::~BindingStatusMonitor() {

}

void 
BindingStatusMonitor::configure(map<string, string>& _config)
{
  AbstractBinder::configure(_config);
} 




} // namespace Binding 
