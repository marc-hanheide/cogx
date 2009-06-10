
#include "MultiTesterMonitor.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/idl/BindingQueries.hh"
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
//#include <boost/assign/std/map.hpp>
//#include <boost/assign.hpp>
#include "cast/architecture/ChangeFilterFactory.hpp"

namespace Binding {
  using namespace boost;
  using namespace boost::assign;
  using namespace std;
  using namespace cast;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::MultiTesterMonitor(_id);
  }
}


MultiTesterMonitor::MultiTesterMonitor(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  TesterMonitor(_id)
{
  setReceiveXarchChangeNotifications(true);
  m_queueBehaviour = cdl::QUEUE;
}

static const unsigned int MANY_PROXIES = 24;
static const unsigned int FEW_PROXIES = 6;

void
MultiTesterMonitor::start() {    
  TesterMonitor::start();
}

void 
MultiTesterMonitor::configure(map<string,string> & _config) {
  TesterMonitor::configure(_config);
  map<string,string>::const_iterator itr = _config.find("--postConditionTester");
  m_postConditionTester = true;
  if(itr == _config.end()) {
    m_postConditionTester = false;
  }

  itr = _config.find("--noOfMultiTesters");
  if(itr == _config.end()) {
    cerr << "Error in MultiTesterMonitor: --noOfMultiTesters N must be given";
    abort();
  }
  m_noOfMultiTesters = lexical_cast<unsigned int>(itr->second);
  assert(m_noOfMultiTesters >= 0);
//  cout << "m_noOfMultiTesters " << m_noOfMultiTesters <<  " " << itr->second << endl;
  
  itr = _config.find("--instance");
  if(itr == _config.end()) {
    cerr << "Error in MultiTesterMonitor: --instance N must be given";
    abort();
  }
  m_instance = lexical_cast<unsigned int>(itr->second);
  assert(m_instance >= 0);
  assert(m_instance < m_noOfMultiTesters);
  
}
  
void MultiTesterMonitor::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  m_sourceID = subarchitectureID();
  switch(m_test) {
  case 0: 
    {
      log("testing to add a single basic proxy with one concept feature that should bind with the proxies of all other MultiTesterMonitor");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      readyCount()++;
    }
    break;
  case 1: 
    {
      log("testing to add a single basic proxy with one concept feature that should NOT bind with the proxies of any other MultiTesterMonitor, then, after some time the proxy is updated such that it should bind with all other proxies.");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      string c = "updated_test_concept_" + lexical_cast<string>(m_instance);
      concept.m_concept = CORBA::string_dup(c.c_str());
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      m_proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      sleepProcess(5000);
      // let every secon monitor sleep a little extra
      if(m_instance % 2)
	sleepProcess(50 * m_instance);
      log("will update proxy now");    
      changeExistingProxy(proxyID,list_of(typeName<BindingFeatures::Concept>()));
      concept.m_concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      log("finished updating");
      awaitBinding(m_proxyIDs);
      readyCount()++;
    }
    break;
  case 2: 
    {
      log("testing to add a single basic proxy with one concept feature that should bind with the proxies of all other MultiTesterMonitor, then, after some time the proxy is updated such that it should bind with no other proxy.");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      m_proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      sleepProcess(5000);
      // let every second monitor sleep a little extra
      if(m_instance % 2)
	sleepProcess(500 * m_instance);
      log("will update proxy now");
      changeExistingProxy(proxyID,list_of(typeName<BindingFeatures::Concept>()));
      string c = "updated_test_concept_" + lexical_cast<string>(m_instance);
      concept.m_concept = CORBA::string_dup(c.c_str());
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      log("finished updating");
      awaitBinding(m_proxyIDs);
      sleepProcess(10000);
      readyCount()++;
    }
    break;
  case 3: 
    {
      log("adding a proxy, then wait for it to bind to at least m_noOfMultiTesters other proxies, then add a MakeProxyUnavailable for the first proxy in the union");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      static string static_proxyID = proxyID; // will in the end only have one correct proxyID
      m_proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      sleepProcess(5000);
      log("will request removal of a proxy now");
      /*      const LBindingProxy& proxy (m_proxyLocalCache[proxyID]);
	      cout << "proxy loaded" << endl;
	      assert(!proxy.bindingUnionID().empty());
	      const LBindingUnion& binding_union(m_unionLocalCache[proxy.bindingUnionID()]);
	      cout << "union loaded" << endl;
      */
      BindingQueries::MakeProxyUnavailable* make_unavailable = new BindingQueries::MakeProxyUnavailable;
      cout << "alocated" << endl;
      make_unavailable->m_proxyID = CORBA::string_dup(static_proxyID.c_str());
      cout << "assigned" << endl;
      cout << "proxyID = " << static_proxyID << endl;
      cout << "proxyID = " << make_unavailable->m_proxyID << endl;
      log(string("the proxy requested for removal:") + string(make_unavailable->m_proxyID));
      addToWorkingMemory(newDataID(), bindingSA(), make_unavailable, cast::cdl::BLOCKING);
      cout << "added" << endl;
      sleepProcess(5000);
      readyCount()++;
      sleepProcess(500);
    }
    break;
  default:
    cerr << "incorrect test number: " << m_test << endl;
    failExit();
  }
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(5000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
}
  
void 
MultiTesterMonitor::testCompleteness() {
  if(!m_postConditionTester)
    return;
  // make sure we're loading fresh data
  m_handler.purgeAllLoadedData();
  try {
    switch(m_test) {
    case 0: 
      {
	cout << "readyCount: " << readyCount() << endl;
	cout << "m_noOfMultiTesters: " << m_noOfMultiTesters << endl;
	cout << "m_addSignalledProxyIDs.size(): " << m_addSignalledProxyIDs.size() << endl;
	
	if(readyCount() == m_noOfMultiTesters && 
	   m_addSignalledProxyIDs.size() == m_noOfMultiTesters && allBound(m_addSignalledProxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == m_noOfMultiTesters);
	  assert(unions.size() == 1);
	  successExit();
	}
      }
      break;
    case 1:
      {
	if(readyCount() == m_noOfMultiTesters && 
	   m_addSignalledProxyIDs.size() == m_noOfMultiTesters && allBound(m_addSignalledProxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == m_noOfMultiTesters);
	  assert(unions.size() == 1);
	  successExit();
	}
      }
      break;        
    case 2:
      {
	if(readyCount() == m_noOfMultiTesters && 
	   m_addSignalledProxyIDs.size() == m_noOfMultiTesters && allBound(m_addSignalledProxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == m_noOfMultiTesters);
	  assert(unions.size() == m_noOfMultiTesters);
	  successExit();
	}
      }
      break;        
    case 3:
      {
	cout << "readyCount: " << readyCount() << endl;
	cout << "m_noOfMultiTesters: " << m_noOfMultiTesters << endl;
	cout << "m_addSignalledProxyIDs.size(): " << m_addSignalledProxyIDs.size() << endl;
	if(readyCount() == m_noOfMultiTesters && 
	   m_addSignalledProxyIDs.size() == m_noOfMultiTesters) {
	  ProxySet proxies = m_handler.allProxiesFromWM();
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == m_noOfMultiTesters - 1);
	  assert(unions.size() == 1);
	  successExit();
	}
      }
      break;        
    default:
      log("incorrect test number");
      failExit();
    } 
  }
  catch(const BindingException & _e) {
    log(string("Caught this while testing") + _e.what());
  }
  catch(const DoesNotExistOnWMException & _e) {
    log(string("Caught this while testing") + _e.what());
  }  
  // reset the retesting if successExit was not called.
  m_retest = 0;  
  log("MultiTesterMonitor::testCompleteness() just failed");
}

} // namespace Binding
