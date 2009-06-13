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
  scoringTasks(0),
  bindingTasks(0),
  totalUnstableTime(0),
  lastValue(true)
{ 
  queueBehaviour = cdl::QUEUE;
  stateID = "WRONG_ID! you added a proxy before the binder got properly initialized.";
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
  scoringTasks++;
  _updateState();
}

void 
BindingStatusMonitor::bindingTaskAdded(const cdl::WorkingMemoryChange & _wmc) 
{
  bindingTasks++;
  _updateState();
}

void 
BindingStatusMonitor::scoringTaskRemoved(const cdl::WorkingMemoryChange & _wmc)
{
  scoringTasks--;
  assert(scoringTasks >= 0);
  _updateState();
}

void 
BindingStatusMonitor::bindingTaskRemoved(const cdl::WorkingMemoryChange & _wmc) 
{
  bindingTasks--;
  assert(bindingTasks >= 0);
  _updateState();
}

void 
BindingStatusMonitor::bindingProxyAdded(const cdl::WorkingMemoryChange & _wmc)
{
  unsigned int size(unbound.size());
  unbound.insert(string(_wmc.address.id));
  assert(size != unbound.size());
  _updateState();
}

/*void 
BindingStatusMonitor::bindingUnionUpdatedOrAdded(const cdl::WorkingMemoryChange & _wmc)
{
  
  const LBindingUnion& binding_union(unionLocalCache[string(_wmc.address.id)]);
    
  unsigned int size(unbound.size());
  for(unsigned int i = 0 ; i < binding_union->proxyIDs.length() ; ++i) {
    string id(binding_union->proxyIDs[i]);
    unbound.erase(id);
    bound.insert(id);
  }
  if(size != unbound.size()) {
    _updateState();
  }
}
*/
void 
BindingStatusMonitor::bindingProxyUpdated(const cdl::WorkingMemoryChange & _wmc)
{
      
  unsigned int size(unbound.size());
  
  string id(_wmc.address.id);
  try {
    if(proxyLocalCache[id]->proxyState == BindingData::BOUND) {
      unbound.erase(id);
      bound.insert(id);
    }
    if(size != unbound.size()) {
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
//  unsigned int size(unbound.size());
  unbound.erase(string(_wmc.address.id));
  bound.erase(string(_wmc.address.id));
  //if(size != unbound.size()) {
  _updateState();
  //}  
}
  

BindingData::BinderStatus*
BindingStatusMonitor::_newState() { 
  BindingData::BinderStatus* state = new BindingData::BinderStatus();
  if(//bindingTasks == 0 && scoringTasks == 0 && 
     unbound.empty()) {
    state->stable = true;
    lastValue = true;
  }  else {
    if(lastValue)
      lastStableTime = clock();
    state->stable = false;
    lastValue = false;
  }

  state->scoringTasks   = scoringTasks;
  state->bindingTasks   = bindingTasks;
  state->unboundProxies = unbound.size();
  state->boundProxies   = bound.size();

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

  
  log("   scoringTasks: " + lexical_cast<string>(scoringTasks) + "(" + 
      lexical_cast<string>(scoringtasks.size()) + ")");
  log("   bindingTasks: " + lexical_cast<string>(bindingTasks)+ "(" + 
      lexical_cast<string>(bindingtasks.size()) + ")");
  log("unbound proxies: " + lexical_cast<string>(state->unboundProxies));
  log("  bound proxies: " + lexical_cast<string>(state->boundProxies));

  log("         stable: " + lexical_cast<string>(state->stable));
#else // NDEBUG

  log("   scoringTasks: " + lexical_cast<string>(scoringTasks));
  log("   bindingTasks: " + lexical_cast<string>(bindingTasks));
  {
    stringstream str;
    for(StringSet::const_iterator i = unbound.begin() ; i != unbound.end() ; ++i) {
      str << *i << ", ";
    }
    log("unbound proxies: " + lexical_cast<string>(state->unboundProxies) + " (" + str.str() + ")");
  }
  {
    stringstream str;
    for(StringSet::const_iterator i = bound.begin() ; i != bound.end() ; ++i) {
      str << *i << ", ";
    }
    log("  bound proxies: " + lexical_cast<string>(state->boundProxies) + " (" + str.str() + ")");
  }

  log("         stable: " + lexical_cast<string>(state->stable));
#endif
  if(state->stable) {
    unsigned int current_time = clock();
    time_t dt = current_time - lastStableTime;
    totalUnstableTime += dt;
    log("              T: " + lexical_cast<string>((double)(dt) / (double)CLOCKS_PER_SEC));
    log("  sum binding T: " + lexical_cast<string>((double)(totalUnstableTime) / (double)CLOCKS_PER_SEC));

    //nah: if not logging, then just print a 1-liner
    if(!bLogOutput) {
      println("stable with %i bound proxies",static_cast<int>(state->boundProxies));
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
//    getWorkingMemoryEntry<BindingData::BinderStatus>(stateID); 
    
    BindingData::BinderStatus* state = _newState();
    overwriteWorkingMemory(stateID,
			   //BindingLocalOntology::BINDER_STATUS_TYPE, 
			   state);
//			   cdl::BLOCKING);
    
  } catch (DoesNotExistOnWMException& _e ) {
    cerr << "caught this in BindingStatusMonitor::_updateState: " << _e.what() << endl 
	 << "while trying lo load binder status on address \""<< stateID << "\"";
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
