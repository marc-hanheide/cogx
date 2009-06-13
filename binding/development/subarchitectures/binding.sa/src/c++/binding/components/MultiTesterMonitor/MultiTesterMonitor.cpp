
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
  queueBehaviour = cdl::QUEUE;
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
  postConditionTester = true;
  if(itr == _config.end()) {
    postConditionTester = false;
  }

  itr = _config.find("--noOfMultiTesters");
  if(itr == _config.end()) {
    cerr << "Error in MultiTesterMonitor: --noOfMultiTesters N must be given";
    abort();
  }
  noOfMultiTesters = lexical_cast<unsigned int>(itr->second);
  assert(noOfMultiTesters >= 0);
//  cout << "noOfMultiTesters " << noOfMultiTesters <<  " " << itr->second << endl;
  
  itr = _config.find("--instance");
  if(itr == _config.end()) {
    cerr << "Error in MultiTesterMonitor: --instance N must be given";
    abort();
  }
  instance = lexical_cast<unsigned int>(itr->second);
  assert(instance >= 0);
  assert(instance < noOfMultiTesters);
  
}
  
void MultiTesterMonitor::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  sourceID = subarchitectureID();
  switch(test) {
  case 0: 
    {
      log("testing to add a single basic proxy with one concept feature that should bind with the proxies of all other MultiTesterMonitor");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
      awaitBinding(proxyIDs);
      readyCount()++;
    }
    break;
  case 1: 
    {
      log("testing to add a single basic proxy with one concept feature that should NOT bind with the proxies of any other MultiTesterMonitor, then, after some time the proxy is updated such that it should bind with all other proxies.");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      string c = "updated_test_concept_" + lexical_cast<string>(instance);
      concept.concept = CORBA::string_dup(c.c_str());
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(proxyIDs);
      sleepProcess(5000);
      // let every secon monitor sleep a little extra
      if(instance % 2)
	sleepProcess(50 * instance);
      log("will update proxy now");    
      changeExistingProxy(proxyID,list_of(typeName<BindingFeatures::Concept>()));
      concept.concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      log("finished updating");
      awaitBinding(proxyIDs);
      readyCount()++;
    }
    break;
  case 2: 
    {
      log("testing to add a single basic proxy with one concept feature that should bind with the proxies of all other MultiTesterMonitor, then, after some time the proxy is updated such that it should bind with no other proxy.");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(proxyIDs);
      sleepProcess(5000);
      // let every second monitor sleep a little extra
      if(instance % 2)
	sleepProcess(500 * instance);
      log("will update proxy now");
      changeExistingProxy(proxyID,list_of(typeName<BindingFeatures::Concept>()));
      string c = "updated_test_concept_" + lexical_cast<string>(instance);
      concept.concept = CORBA::string_dup(c.c_str());
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      log("finished updating");
      awaitBinding(proxyIDs);
      sleepProcess(10000);
      readyCount()++;
    }
    break;
  case 3: 
    {
      log("adding a proxy, then wait for it to bind to at least noOfMultiTesters other proxies, then add a MakeProxyUnavailable for the first proxy in the union");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      string proxyID = storeCurrentProxy();
      static string static_proxyID = proxyID; // will in the end only have one correct proxyID
      proxyIDs.insert(proxyID);
      bindNewProxies();
      awaitBinding(proxyIDs);
      sleepProcess(5000);
      log("will request removal of a proxy now");
      /*      const LBindingProxy& proxy (proxyLocalCache[proxyID]);
	      cout << "proxy loaded" << endl;
	      assert(!proxy.bindingUnionID().empty());
	      const LBindingUnion& binding_union(unionLocalCache[proxy.bindingUnionID()]);
	      cout << "union loaded" << endl;
      */
      BindingQueries::MakeProxyUnavailable* make_unavailable = new BindingQueries::MakeProxyUnavailable;
      cout << "alocated" << endl;
      make_unavailable->proxyID = CORBA::string_dup(static_proxyID.c_str());
      cout << "assigned" << endl;
      cout << "proxyID = " << static_proxyID << endl;
      cout << "proxyID = " << make_unavailable->proxyID << endl;
      log(string("the proxy requested for removal:") + string(make_unavailable->proxyID));
      addToWorkingMemory(newDataID(), bindingSA(), make_unavailable, cast::cdl::BLOCKING);
      cout << "added" << endl;
      sleepProcess(5000);
      readyCount()++;
      sleepProcess(500);
    }
    break;
  default:
    cerr << "incorrect test number: " << test << endl;
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
  if(!postConditionTester)
    return;
  // make sure we're loading fresh data
  handler.purgeAllLoadedData();
  try {
    switch(test) {
    case 0: 
      {
	cout << "readyCount: " << readyCount() << endl;
	cout << "noOfMultiTesters: " << noOfMultiTesters << endl;
	cout << "addSignalledProxyIDs.size(): " << addSignalledProxyIDs.size() << endl;
	
	if(readyCount() == noOfMultiTesters && 
	   addSignalledProxyIDs.size() == noOfMultiTesters && allBound(addSignalledProxyIDs)) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == noOfMultiTesters);
	  assert(unions.size() == 1);
	  successExit();
	}
      }
      break;
    case 1:
      {
	if(readyCount() == noOfMultiTesters && 
	   addSignalledProxyIDs.size() == noOfMultiTesters && allBound(addSignalledProxyIDs)) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == noOfMultiTesters);
	  assert(unions.size() == 1);
	  successExit();
	}
      }
      break;        
    case 2:
      {
	if(readyCount() == noOfMultiTesters && 
	   addSignalledProxyIDs.size() == noOfMultiTesters && allBound(addSignalledProxyIDs)) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == noOfMultiTesters);
	  assert(unions.size() == noOfMultiTesters);
	  successExit();
	}
      }
      break;        
    case 3:
      {
	cout << "readyCount: " << readyCount() << endl;
	cout << "noOfMultiTesters: " << noOfMultiTesters << endl;
	cout << "addSignalledProxyIDs.size(): " << addSignalledProxyIDs.size() << endl;
	if(readyCount() == noOfMultiTesters && 
	   addSignalledProxyIDs.size() == noOfMultiTesters) {
	  ProxySet proxies = handler.allProxiesFromWM();
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == noOfMultiTesters - 1);
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
  retest = 0;  
  log("MultiTesterMonitor::testCompleteness() just failed");
}

} // namespace Binding
