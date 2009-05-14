
#include "TesterMonitor.hpp"
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
    return new Binding::TesterMonitor(_id);
  }
}


TesterMonitor::TesterMonitor(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  AbstractMonitor(_id),
//  AbstractBindingWMRepresenter(dynamic_cast<WorkingMemoryReaderProcess&>(*this)),
  m_test(-1),
  m_testFinished(true),
  m_handler(*this),
  m_retest(0),
  m_statusUpdates(0),
  m_statusStableUpdates(0)
{
  setReceiveXarchChangeNotifications(true);
  m_queueBehaviour = cdl::QUEUE;
}

static const unsigned int MANY_PROXIES = 24;
static const unsigned int FEW_PROXIES = 6;

void
TesterMonitor::start() {    
  AbstractMonitor::start();
  
  //addChangeFilter(BindingLocalOntology::BINDING_PROXY_TYPE, 
  //cdl::ADD, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<TesterMonitor>(this,
  //&TesterMonitor::bindingProxyAdded));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::bindingProxyAdded));
  //addChangeFilter(BindingGlobalOntology::BINDING_PROXY_TYPE, 
  //cdl::OVERWRITE, 
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<TesterMonitor>(this,
  //&TesterMonitor::bindingProxyUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::bindingProxyUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::BindingUnion>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::bindingUnionUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingUnion>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BindingUnion>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::bindingUnionUpdated));
  
  //addChangeFilter(BindingGlobalOntology::BINDER_STATUS_TYPE,
  //cdl::ADD,
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<TesterMonitor>(this,
  //&TesterMonitor::statusUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BinderStatus>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::statusUpdated));
  //addChangeFilter(BindingGlobalOntology::BINDER_STATUS_TYPE,
  //cdl::OVERWRITE,
  //cdl::ALL_SA, //only look at sa-local changes
  //new MemberFunctionChangeReceiver<TesterMonitor>(this,
  //&TesterMonitor::statusUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::BinderStatus>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::statusUpdated));
  
  addChangeFilter(createGlobalTypeFilter<BindingData::Ambiguity>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::TriggerDotViewer>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::TriggerDotViewerWithTitle>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::Ambiguity>(cdl::ADD),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::ProxyPorts>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::somethingInterestingUpdated));
  
  addChangeFilter(createGlobalTypeFilter<BindingQueries::BasicQuery>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::basicQueryAnswered));

  addChangeFilter(createGlobalTypeFilter<BindingQueries::AdvancedQuery>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<TesterMonitor>(this,
								  &TesterMonitor::advancedQueryAnswered));
}

void 
TesterMonitor::configure(map<string,string> & _config) {
  AbstractMonitor::configure(_config);
  map<string,string>::const_iterator itr = _config.find("--test");
  if(itr == _config.end()) {
    cerr << "Error in TesterMonitor: --test N must be given";
    abort();
  }
  m_test = lexical_cast<int>(itr->second);
  assert(m_test >= 0);
  cout << "bindingSA: " << getBindingSA() << endl;
  dynamic_cast<AbstractBindingWMRepresenter*>(this)->setBindingSubarchID(getBindingSA());
}

void TesterMonitor::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  m_sourceID = subarchitectureID();
  m_testFinished = false;
  switch(m_test) {
  case 0: 
    {
      log("testing to add a single basic proxy with no features");
      startNewBasicProxy();
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    break;
  case 1:
    {
      m_testFinished = false;
      log("testing to add two proxies that should NOT bind, then add a couple of queries too");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.m_colour = CORBA::string_dup("blue");
	addFeatureToCurrentProxy(colour);
	m_proxyIDs.insert(storeCurrentProxy());
      }
/*      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::OtherSourceID other;
	other.m_otherSourceID = CORBA::string_dup("binding.sa");
	addFeatureToCurrentProxy(other, BindingFeaturesCommon::NEGATIVE);
	m_proxyIDs.insert(storeCurrentProxy());
      }*/
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept, BindingFeaturesCommon::NEGATIVE); 
	m_proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      {
	BindingFeatures::Concept* query_concept = new BindingFeatures::Concept;
	query_concept->m_concept = CORBA::string_dup("test_concept");
	query_concept->m_parent.m_truthValue = BindingFeaturesCommon::POSITIVE;
	query_concept->m_parent.m_immediateProxyID = CORBA::string_dup("");;
	string id(newDataID());
	addToWorkingMemory(id,bindingSubarchID(),query_concept);
	BindingQueries::BasicQuery* query = new BindingQueries::BasicQuery;
	query->m_parameters.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
	query->m_proxyID = CORBA::string_dup(m_proxyIDs.begin()->c_str());	
	query->m_featurePointer.m_type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	query->m_featurePointer.m_address = CORBA::string_dup(id.c_str());
	query->m_featurePointer.m_immediateProxyID = CORBA::string_dup("");
	query->m_answer = cast::cdl::triIndeterminate;
	query->m_processed = false;
	string qid(newDataID());
	addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
	m_basicQueryIDs.push_back(qid);
      }
      {
	BindingFeatures::Concept* query_concept = new BindingFeatures::Concept;
	query_concept->m_concept = CORBA::string_dup("test_concept");
	query_concept->m_parent.m_truthValue = BindingFeaturesCommon::POSITIVE;
	query_concept->m_parent.m_immediateProxyID = CORBA::string_dup("");;
	string id(newDataID());
	addToWorkingMemory(id,bindingSubarchID(),query_concept);
	BindingData::FeaturePointer ptr;
	ptr.m_type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	ptr.m_address = CORBA::string_dup(id.c_str());
	BindingQueries::AdvancedQuery* query = new BindingQueries::AdvancedQuery;
	query->m_parameters.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
	query->m_featurePointer.m_type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	query->m_featurePointer.m_address = CORBA::string_dup(id.c_str());
	query->m_featurePointer.m_immediateProxyID = CORBA::string_dup("");	
	query->m_hasTheFeatureProxyIDs.length(0);
	query->m_hasTheFeatureUnionIDs.length(0);
	query->m_matchingProxyIDs.length(0);
	query->m_matchingUnionIDs.length(0);
	query->m_nonMatchingProxyIDs.length(0);
	query->m_nonMatchingUnionIDs.length(0);
	query->m_processed = false;
	
	string qid(newDataID());
	addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
	m_advancedQueryIDs.push_back(qid);
      }
      sleepProcess(2000);
      m_testFinished = true;
    }
    break;
  case 2:
    {
      log("testing to add two proxies that should bind");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
	//bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept); 
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
    }
    break;
  case 3:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation between them, no bindings");
      addTwoProxiesAndOneRelation();
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      m_testFinished = true;
    }
    break;
  case 4:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation between them (twice, and the relation should bind as a consequence)");
      addTwoProxiesAndOneRelation();
      sleepProcess(3000);
      log("------- time to add more proxies");
      addTwoProxiesAndOneRelation();
      bindNewProxies();
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      m_testFinished = true;
    }
    break;
  case 5:
    {
      log("testing a single unbounded group proxy");
      startNewGroupProxy(0);
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      log("about to bind the proxy");
      bindNewProxies();
    }
    break;
  case 6:
    {
      log("testing to add two proxies that should bind (with a delay in between)");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
      sleepProcess(1000);
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept); 
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
    }
    break;
  case 7:
    {
      log("adds a number of proxies that should not bind, and then delete all of them");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept" + lexical_cast<string>(i)).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
	bindNewProxies();
      }
      sleepProcess(1000);
      for(set<string>::const_iterator itr = m_proxyIDs.begin();
	  itr != m_proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 8:
    {
      log("Add a number of proxies that all should bind, and call bindNewProxies only once");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
    }
    break;
  case 9:
    {
      m_testFinished = false;
      log("Add two sets of proxies that all should bind within the sets, and call bindNewProxies only once");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string(string("test_concept") + ((i%2)?"_1":"_2")).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
      awaitBinding(m_proxyIDs);
    }
    m_testFinished = true;
    break;
  case 10:
    {
      log("Add three sets of proxies that all should bind within the sets, and call bindNewProxies only once");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string(string("test_concept_") + lexical_cast<string>(i%3 + 1)).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
    }
    break;
  case 11:
    {
      log("Add many proxies that should bind, bind them incrementally, then delete them after a while");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      bindNewProxies();
      }
      sleepProcess(5000);
      for(set<string>::const_iterator itr = m_proxyIDs.begin();
	  itr != m_proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 12:
    {
      log("Add many proxies that should bind, bind them all at once, then delete them after a while");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
      sleepProcess(5000);
      for(set<string>::const_iterator itr = m_proxyIDs.begin();
	  itr != m_proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 13:
    {
      log("Add many proxies that should bind, bind them incrementally, then delete them right away");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
	bindNewProxies();
      }
      for(set<string>::const_iterator itr = m_proxyIDs.begin();
	  itr != m_proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 14:
    {
      log("Add many proxies that should bind, bind them all at once, then delete them right away");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
      for(set<string>::const_iterator itr = m_proxyIDs.begin();
	  itr != m_proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 15:
    {
      m_testFinished = false;
      log("for ten times in a row: Add few proxies that should bind, bind them incremently, and delete them right away (in a mixed order)");
      for(unsigned int j = 0 ; j < 10 ; ++j) {
	log(string("gonna create ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	deque<string> ids;
	for(unsigned int i = 0 ; i < FEW_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  m_proxyIDs.insert(id);
	  if((i+j)%2) 
	    ids.push_back(id);
	  else
	    ids.push_front(id);
	  bindNewProxies();
	}
	log(string("gonna delete ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	for(deque<string>::const_iterator itr = ids.begin();
	    itr != ids.end();
	    ++itr) {
	  deleteExistingProxy(*itr);
	}
      }
      m_testFinished = true;
    }
  case 16:
    {
      m_testFinished = false;
      log("for ten times in a row: Add few proxies that should bind, bind them incremently, wait a bit and then delete them (in a mixed order)");
      for(unsigned int j = 0 ; j < 10 ; ++j) {
	log(string("gonna create ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	deque<string> ids;
	for(unsigned int i = 0 ; i < FEW_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  m_proxyIDs.insert(id);
	  if((i+j)%2) 
	    ids.push_back(id);
	  else
	    ids.push_front(id);
	  bindNewProxies();
	}
	sleepProcess(2000);
	log(string("gonna delete ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	for(deque<string>::const_iterator itr = ids.begin();
	    itr != ids.end();
	    ++itr) {
	  deleteExistingProxy(*itr);
	}
      }
      m_testFinished = true;
    }
    break;
  case 17:
    {
      m_testFinished = false;
      log("testing to add a single basic proxy and then update it after some delay");
      string id;
      {
	//addTwoProxiesAndOneRelation();
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id = storeCurrentProxy();
	m_proxyIDs.insert(id);
	bindNewProxies();
      }
      sleepProcess(3000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING); 
      log("Will now change the proxy");      
      {
	// now change it
	changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
	cout << "loaded proxy with: \"changeExistingProxy\"" << endl;
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("updated_test_concept");
	addFeatureToCurrentProxy(concept);
	cout << "added update concept feature" << endl;
	string id2 = storeCurrentProxy();
	cout << "stored updated proxy" << endl;
	assert(id == id2);
	bindNewProxies();
	cout << "called bindNewProxies()" << endl;
      }
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(4000);
      m_testFinished = true;
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    }
    break;  
  case 18:
    {
      m_testFinished = false;
      log("testing to add a single basic proxy and then update with no changes after some delay");
      string id;
      {
	//addTwoProxiesAndOneRelation();
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id = storeCurrentProxy();
	m_proxyIDs.insert(id);
	bindNewProxies();
      }
      sleepProcess(1000);
      {
	changeExistingProxy(id);
	string id2 = storeCurrentProxy();
	assert(id == id2);
	bindNewProxies();
      }
      m_testFinished = true;
    }
    break;
  case 19:
    {
      m_testFinished = false;
      log("testing to add many proxies (that should bind) and then update them after they are bound such that they don't bind anymore");
      {
	for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  m_proxyIDs.insert(id);
	}
	bindNewProxies();
      }
      sleepProcess(2000);
      // make sure all proxies are first bound
      ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	m_handler.reloadAllLoadedData();
	proxies = m_handler.loadProxies(m_proxyIDs);
      }
      log("Now the proxies will be updated");
      UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
      assert(unions.size() == 1);
      {
	unsigned int nr = 0;
	// now change them
	foreach(string id,m_proxyIDs) {
	  changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup((string("updated_test_concept_") + lexical_cast<string>(nr++)).c_str());
	  addFeatureToCurrentProxy(concept);
	  string id2 = storeCurrentProxy();
	  assert(id == id2);
	}
	bindNewProxies();
      }
      m_testFinished = true;
    }
    break;
  case 20:
    {
      m_testFinished = false;
      log("testing to add many proxies (that should NOT bind) and then update them after they are bound such that they DO bind");
      {
	for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup((string("test_concept_") + lexical_cast<string>(i)).c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  m_proxyIDs.insert(id);
	}
	bindNewProxies();
      }
      sleepProcess(2000);
      // make sure all proxies are first bound
      ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	m_handler.reloadAllLoadedData();
	proxies = m_handler.loadProxies(m_proxyIDs);
      }
      log("Now the proxies will be updated");
      UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
      assert(unions.size() == MANY_PROXIES);
      {
	// now change them
	foreach(string id,m_proxyIDs) {
	  changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
	  BindingFeatures::Concept concept;
	  concept.m_concept = CORBA::string_dup(string("updated_test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id2 = storeCurrentProxy();
	  assert(id == id2);
	}
	bindNewProxies();
	sleepProcess(10000);
      }
      m_testFinished = true;
    }
    break;
  case 21:
    {
      log("adding two proxies that should not bind, then add a third one that is ambiguous, then check that there isa disambiguation struct on WM");
      {
	startNewBasicProxy(); // proxy blue
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.m_colour = CORBA::string_dup("blue");
	addFeatureToCurrentProxy(colour);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy(); // proxy red
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.m_colour = CORBA::string_dup("red");
	addFeatureToCurrentProxy(colour);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();
      ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	sleepProcess(10);
	m_handler.reloadAllLoadedData();
	proxies = m_handler.loadProxies(m_proxyIDs);
      }
      {
	startNewBasicProxy(); // proxy with unknown colour
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      m_testFinished = true;   
      bindNewProxies();  
    }
    break;
  case 22:
    {
      log("testing to add two proxies that should NOT bind (due to OtherProxyID)");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::OtherSourceID other;
	other.m_otherSourceID = CORBA::string_dup(subarchitectureID().c_str());
	addFeatureToCurrentProxy(other, BindingFeaturesCommon::NEGATIVE);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();
    }
    break;
  case 23:
    {
      log("testing to add three proxies of which two should bind, and a third not (all due to ExistingProxyID)");
      string id1;
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id1 = storeCurrentProxy();
	m_proxyIDs.insert(id1);
      }
      {
	startNewBasicProxy();
	BindingFeatures::ExistingProxyID existing;
	existing.m_existingProxyID = CORBA::string_dup(id1.c_str());
	addFeatureToCurrentProxy(existing);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::ExistingProxyID existing;
	existing.m_existingProxyID = CORBA::string_dup(id1.c_str());
	addFeatureToCurrentProxy(existing, BindingFeaturesCommon::NEGATIVE);
	m_proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();      
    }
    break;  
  case 24:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation, then bind, then delete the relation");
      string rel_id = addTwoProxiesAndOneRelation();
      bindNewProxies();      
      // make sure all proxies are first bound
      ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	m_handler.reloadAllLoadedData();
	proxies = m_handler.loadProxies(m_proxyIDs);
      }
      deleteExistingProxy(rel_id);      
      m_proxyIDs.erase(rel_id);
      awaitBinding(m_proxyIDs);
      sleepProcess(10000);
      m_testFinished = true;
    }
    break;
  case 25:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation, then bind, then update one of the related proxies");
      string rel_id = addTwoProxiesAndOneRelation();
      bindNewProxies();      
      // make sure all proxies are first bound
      ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	m_handler.reloadAllLoadedData();
	proxies = m_handler.loadProxies(m_proxyIDs);
      }
      const string& id(*m_proxyIDs.begin());
      changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("updated_test_concept");
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      m_testFinished = true;
      awaitBinding(m_proxyIDs);
    }
    break;  
  case 26:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      bindNewProxies();      
      // make sure all proxies are first bound
      awaitBinding(m_proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","test_concept_not2","test_label");
      m_testFinished = true;
      awaitBinding(m_proxyIDs);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(4000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
    }
    break;
  case 27:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones (zip up)");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      // make sure all proxies are first bound
      awaitBinding(m_proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","","test_label");
      awaitBinding(m_proxyIDs);
      m_testFinished = true;      
    }
    break;
  case 28:
    {
      m_testFinished = false;
      log("testing to add two proxies and two relations, then remove the relations, add new proxy and add a relation to it instead");
      string id1,id2,id3;
      string rel_id1,rel_id2;
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept1");
	addFeatureToCurrentProxy(concept);
	id1 = storeCurrentProxy();
	m_proxyIDs.insert(id1);
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept2");
	addFeatureToCurrentProxy(concept); 
	id2 = storeCurrentProxy();
	m_proxyIDs.insert(id2);
      }
      bindNewProxies();
      rel_id1 = addSimpleRelation(id1,id2,"rel_label1");
      rel_id2 = addSimpleRelation(id2,id1,"rel_label1");
      m_proxyIDs.insert(rel_id1);
      m_proxyIDs.insert(rel_id2);
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      log("BEFORE DELETION");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      deleteExistingProxy(rel_id1);
      deleteExistingProxy(rel_id2);
      m_proxyIDs.erase(rel_id1);
      m_proxyIDs.erase(rel_id2);
      awaitBinding(m_proxyIDs);
      sleepProcess(1000);
      log("AFTER DELETION OF RELATIONS");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000); // sleep for a while to allow dotviewer to finish
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("test_concept3");
	addFeatureToCurrentProxy(concept); 
	id3 = storeCurrentProxy();
	m_proxyIDs.insert(id3);
      }
      rel_id1 = addSimpleRelation(id1,id3,"rel_label2");
      rel_id2 = addSimpleRelation(id3,id1,"rel_label2");
      bindNewProxies();
      m_proxyIDs.insert(rel_id1);
      m_proxyIDs.insert(rel_id2);
      log("AFTER ADDITION OF NEW RELAIONS");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000); // sleep for a while to allow dotviewer to finish
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      m_testFinished = true;      
    }
    break;  
  case 29:
    {
      m_testFinished = false;
      log("testing to add two proxies and one relations, and then remove the relation");
      string id;
      string rel_id;
      {
	startNewBasicProxy();
	id = storeCurrentProxy();
	m_proxyIDs.insert(id);
      }
      bindNewProxies();
      rel_id = addSimpleRelation(id,id,"rel_label");
      m_proxyIDs.insert(rel_id);
      bindNewProxies();
      awaitBinding(m_proxyIDs);
      const LBindingProxy& da_proxy(m_proxyLocalCache[id]);
      typedef vector<shared_ptr<const CASTData<BindingData::BindingUnion> > > UnionPtrs;
      UnionPtrs unions;
      getWorkingMemoryEntries<BindingData::BindingUnion>(0,
							 unions);
      
      typedef vector<shared_ptr<const CASTData<BindingData::BindingProxy> > > ProxyPtrs;
      ProxyPtrs proxies;
      getWorkingMemoryEntries<BindingData::BindingProxy>(0,
							 proxies);
      
      assert(proxies.size() == 2);
      assert(unions.size() == 2);      
      assert(existsOnWorkingMemory(id));
      assert(existsOnWorkingMemory(da_proxy.bindingUnionID()));
      log("BEFORE DELETION");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      deleteExistingProxy(rel_id);
      m_proxyIDs.erase(rel_id);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      awaitBinding(m_proxyIDs);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      log("AFTER DELETION OF RELATIONS");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000); // sleep for a while to allow dotviewer to finish
      assert(existsOnWorkingMemory(id));
      assert(existsOnWorkingMemory(da_proxy.bindingUnionID()));
      getWorkingMemoryEntries<BindingData::BindingUnion>(0,
							 unions);      
      getWorkingMemoryEntries<BindingData::BindingProxy>(0,
							 proxies);
      assert(proxies.size() == 1);
      assert(unions.size() == 1);      
      assert(!existsOnWorkingMemory(rel_id));
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      m_testFinished = true;      
    }
    break;
  case 30:
    {
      m_testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones (the zip-up, cf. test 26)");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      bindNewProxies();      
      // make sure all proxies are first bound
      awaitBinding(m_proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","","test_label");
      m_testFinished = true;
      awaitBinding(m_proxyIDs);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(4000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
    }
    break;
  case 31:
    {
      m_testFinished = false;
      log("testing to add three proxies and a pair of relations. Nothing should bind");
      {
	string id1,id2,id3;
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("concept1");
	addFeatureToCurrentProxy(concept);	
	id1 = storeCurrentProxy();
	m_proxyIDs.insert(id1);
	bindNewProxies();
	startNewBasicProxy();
	concept.m_concept = CORBA::string_dup("concept2"); 
	addFeatureToCurrentProxy(concept); 
	id2 = storeCurrentProxy();
	m_proxyIDs.insert(id2);
	bindNewProxies();
	startNewRelationProxy();	      
	BindingFeatures::RelationLabel label;	
	label.m_label = CORBA::string_dup("label");
	addFeatureToCurrentProxy(label); 
	addOutPortToCurrentProxy(id1, "from");
	addOutPortToCurrentProxy(id2, "to");
	string rel_id = storeCurrentProxy();
	m_proxyIDs.insert(rel_id);	
	bindNewProxies();
	startNewBasicProxy();
	concept.m_concept = CORBA::string_dup("concept3"); 
	addFeatureToCurrentProxy(concept); 
	id3 = storeCurrentProxy();
	m_proxyIDs.insert(id3);
	bindNewProxies();
	startNewRelationProxy();
	addFeatureToCurrentProxy(label); 
	addOutPortToCurrentProxy(id3, "from");
	addOutPortToCurrentProxy(id2, "to");
	rel_id = storeCurrentProxy();
	m_proxyIDs.insert(rel_id);	
      }
      bindNewProxies();
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      m_testFinished = true;
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
    }
    break;
  default:
    cerr << "incorrect test number: " << m_test << endl;
    failExit();
  }
  m_testFinished = true;
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(5000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
}
  
void 
TesterMonitor::bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  //const LBindingProxy& binding_proxy(m_proxyLocalCache[id]);
  m_addSignalledProxyIDs.insert(id);
}

void 
TesterMonitor::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  m_overwriteSignalledProxyIDs.insert(id);
  try {
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);      
  } catch(const DoesNotExistOnWMException& _e) {
    cerr << "Caught this in TesterMonitor::bindingProxyUpdated: " + string(_e.what());
    abort();
  }
  testCompleteness();
}

void 
TesterMonitor::bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in TesterMonitor::bindingUnionUpdated: " + string(_e.what()));
    abort();
  }
  testCompleteness();
}

void 
TesterMonitor::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    m_statusUpdates++;
    if(m_statusID.empty())
      m_statusID = _wmc.m_address.m_id;
    else
      assert(m_statusID == string(_wmc.m_address.m_id));
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
    if(m_status.m_stable)
      m_statusStableUpdates++;    
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in TesterMonitor::statusUpdated: " + string(_e.what()));
    abort();
  }
  testCompleteness();
}

void 
TesterMonitor::somethingInterestingUpdated(const cast::cdl::WorkingMemoryChange &) {
  testCompleteness();
}

  
void 
TesterMonitor::testCompleteness() {
  // make sure we're loading fresh data
  m_handler.purgeAllLoadedData();
  try {
    switch(m_test) {
    case 0: 
      {
	assert(m_addSignalledProxyIDs.size() <= 1);
	if(allBound()) {
	  successExit();
	}
      }
      break;  
    case 1: 
      {
	assert(m_addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies, proxyCheck(BestListCountChecker<>(1))));
	  assert(true_for_all(proxies, proxyCheck(NonMatchingListCountChecker<>(1))));
	  assert(unions.size() == 2);
	  if(!m_basicQueryAnswers.empty() &&
	     !m_advancedQueryAnswers.empty() &&
	     true_for_all(unions, 
			  UnionProxyCountCheck(1) && 
			  !UnionIsBoundToAllOfProxies(proxies) &&
			  UnionIsBoundToOneOfProxies(proxies))) {
	    // some testing of the tests
	    assert(m_basicQueryAnswers.begin()->second.m_answer == cast::cdl::triTrue);
	    assert(m_advancedQueryAnswers.begin()->second.m_hasTheFeatureProxyIDs.length() == 2);
	    assert(m_advancedQueryAnswers.begin()->second.m_hasTheFeatureUnionIDs.length() == 2);
	    assert(m_advancedQueryAnswers.begin()->second.m_matchingProxyIDs.length() == 1);
	    assert(m_advancedQueryAnswers.begin()->second.m_matchingUnionIDs.length() == 1);
	    
	    assert(m_advancedQueryAnswers.begin()->second.m_nonMatchingProxyIDs.length() == 1);
	    assert(m_advancedQueryAnswers.begin()->second.m_nonMatchingUnionIDs.length() == 1);
	    ProxySet matching_proxies = m_handler.loadProxies(m_advancedQueryAnswers.begin()->second.m_matchingProxyIDs);
	    assert(true_for_all(matching_proxies,
				hasFeature<ProxyPtr,BindingFeatures::Colour>(1)));
	    assert(true_for_all(unions,
				//ConsistencyCheck<UnionPtr>(*this) && 
				HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::Concept>()) &&
				HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::ThisProxyID>()) &&
				TypeCheck<UnionPtr>(BindingData::BASIC)
				)
		   );
	    shared_ptr<AbstractPredicate<UnionPtr> > checker(HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::Concept>()).clone());
	    assert(true_for_some(unions,
				 *checker));
	    checker = (*checker && HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::ThisProxyID>())).clone();
	    assert(true_for_some(unions,
				 *checker));	    
	    checker = clone_predicate(*checker && 
				      UnionIsBoundToProxy(*m_overwriteSignalledProxyIDs.begin()) &&
				      TypeCheck<UnionPtr>(BindingData::BASIC));
	    
	    assert(true_for_some(unions,
				 *checker));
	    
	    assert(true_for_all(unions,
				*checker == !(!*checker)));
	    
	    checker = clone_predicate(!*checker);
	    assert(true_for_some(unions,
	    			 !*checker));

	    
	    assert(true_for_some(unions,
				 //ConsistencyCheck<UnionPtr>(*this) && 
				 HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::Concept>()) &&
				 HasFeatureCheck<UnionPtr>(cast::typeName<BindingFeatures::ThisProxyID>()) &&
				 UnionIsBoundToProxy(*m_overwriteSignalledProxyIDs.begin()) &&
				 TypeCheck<UnionPtr>(BindingData::BASIC)
				 )
		   );
	    
	    shared_ptr<AbstractPredicate<ProxyPtr > > 
	      checker3(TruePredicate<ProxyPtr>().clone());
	    
	    assert(true_for_all(proxies,TruePredicate<ProxyPtr>()));
	    assert(true_for_all(proxies,!(!TruePredicate<ProxyPtr>())));
	    assert(true_for_all(proxies,*checker3));
	    assert(true_for_all(proxies,!(!*checker3)));
	    checker3 = clone_predicate(!(*checker3));
	    	    
	    shared_ptr<AbstractPredicate<ProxyPtr> > 
	      checker4((TruePredicate<ProxyPtr>() || !TruePredicate<ProxyPtr>()).clone());
	    	    
	    assert(true_for_all(proxies,!*checker3));
	    checker3 = clone_predicate(!(*checker3));
	    assert(true_for_all(proxies,*checker3));
	    
	    
	    shared_ptr<AbstractPredicate<ProxyPtr > > 
	      checker2(HasFeatureCheck<ProxyPtr>(cast::typeName<BindingFeatures::Concept>()).clone());
	    
	    // clone ore clone_predicate can be used:
	    checker2 = 
	      (*checker2 &&
	       hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
	       !ProxyStateChecker(BindingData::REPROCESSED) &&
	       !ProxyStateChecker(BindingData::NEW)).clone();
	    
	    assert(true_for_all(proxies, 
				*checker2));
	    assert(true_for_all(proxies,
				*checker2 == !(!*checker2)));
		   
	    checker2 = clone_predicate(!(*checker2));
	    checker2 = clone_predicate(!(*checker2));
	    assert(true_for_all(proxies, *checker2));
	    checker2 = clone_predicate(!(*checker2));
	    assert(true_for_all(proxies, !*checker2));
	    assert(true_for_all(proxies,
				//ConsistencyCheck<ProxyPtr>(*this) &&
				HasFeatureCheck<ProxyPtr>(cast::typeName<BindingFeatures::Concept>()) &&
				hasFeature<ProxyPtr,BindingFeatures::Concept>(1) && // exactly one concept feature
				hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
				!ProxyStateChecker(BindingData::REPROCESSED) &&
				!ProxyStateChecker(BindingData::NEW) &&
				!ProxyStateChecker(BindingData::UPDATED) &&
				ProxyStateChecker(BindingData::BOUND) &&
				TypeCheck<ProxyPtr>(BindingData::BASIC)
				)
		   );	  
	    successExit();
	  }
	}
      }
      break;
    case 2: 
      {
	assert(m_addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(2))) {
	    assert(unions.size() == 1);
	    successExit();
	  }
	}
      }
      break;
    case 3: 
      {
	assert(m_addSignalledProxyIDs.size() <= 3);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(1))) {
	    // testing of test operators
	    proxies = subset(proxies,
			     TypeCheck<ProxyPtr>(BindingData::BASIC)); // remove all that are not basic
	    // alt:
	    proxies = proxies | TypeCheck<ProxyPtr>(BindingData::BASIC);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    proxies = m_handler.extractProxiesFromProxies(proxies, InPortsExtractor<ProxyPtr>()); // should now only contain the one relation
	    assert(proxies.size() == 1);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(true_for_all(proxies,
				TypeCheck<ProxyPtr>(BindingData::RELATION) ->* 
				hasFeature<ProxyPtr,BindingFeatures::RelationLabel>()));	  
	    proxies = m_handler.extractProxiesFromProxies(proxies, OutPortsExtractor<ProxyPtr>());  // should now contain only the related proxies
	    assert(proxies.size() == 2);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    ProxySet one_proxy; one_proxy.insert(*proxies.begin()); // store only one basic proxy
	    one_proxy += one_proxy; // should be a noop.
	    assert(one_proxy.size() == 1);
	    // get the relation proxy again
	    proxies = 
	      m_handler.extractProxiesFromProxies(one_proxy,
						  InPortsExtractor<ProxyPtr>());
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(true_for_all(one_proxy,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    assert(proxies.size() == 1);
	    // get all proxies via the out ports, one by one
	    proxies = 
	      m_handler.extractProxiesFromProxies(one_proxy,
						  InPortsExtractor<ProxyPtr>() || OutPortsExtractor<ProxyPtr>());
	    cout << proxies << endl;
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(proxies.size() == 1);
	    proxies += // obs += here... 
	      m_handler.extractProxiesFromProxies(proxies,
						  InPortsExtractor<ProxyPtr>() || OutPortsExtractor<ProxyPtr>());
	    assert(proxies.size() == 3);
	    
	    // now, extract over all in and outports recursively instead
	    proxies = 
	      m_handler.extractProxiesFromProxies(one_proxy,
						  RecursiveExtractor<ProxyPtr>(OutPortsExtractor<ProxyPtr>()||InPortsExtractor<ProxyPtr>(),
									       m_handler.loadProxyFctPtr()));
	    //should contain all proxies
	    cout << proxies << endl;
	    assert(proxies.size() == 3);
	    assert(true_for_all(unions,UnionIsBoundToOneOfProxies(proxies)));
	    assert(unions.size() == 3);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)    ->* InportsCountCheck<ProxyPtr>(1)));
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION) ->* OutportsCountCheck<ProxyPtr>(2)));
	    assert(true_for_all(unions,TypeCheck<UnionPtr>(BindingData::BASIC)     ->* InportsCountCheck<UnionPtr>(1)));
	    assert(true_for_all(unions,TypeCheck<UnionPtr>(BindingData::RELATION)  ->* OutportsCountCheck<UnionPtr>(2)));

	    successExit();
	  }
	}
      }
      break;
    case 4: 
      {
	assert(m_addSignalledProxyIDs.size() <= 6);
	if(m_testFinished && allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(2))) { // each union has two proxies
	    assert(unions.size() == 3);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)    ->* InportsCountCheck<ProxyPtr>(1)));
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION) ->* OutportsCountCheck<ProxyPtr>(2)));
	    assert(true_for_all(unions,TypeCheck<UnionPtr>(BindingData::BASIC)     ->* InportsCountCheck<UnionPtr>(2)));
	    assert(true_for_all(unions,TypeCheck<UnionPtr>(BindingData::RELATION)  ->* OutportsCountCheck<UnionPtr>(4)));
	    successExit();
	  }
	}
      }
      break;
    case 5: 
      {
	assert(m_addSignalledProxyIDs.size() <= 2);
	if(!m_proxyIDs.empty() && 
	   m_status.m_stable &&
	   m_overwriteSignalledProxyIDs.size() > 1 &&
	   m_overwriteSignalledProxyIDs == m_addSignalledProxyIDs) 
	  {
	    ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	    UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	    if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND)) &&
	       true_for_all(unions, UnionProxyCountCheck(1))) { // each union has one proxy
	      // test 2 ways of extracting bound unions
	      ProxySet proxies2 = m_handler.extractProxiesFromUnions(unions);
	      ProxySet proxies3 = m_handler.extractProxiesFromProxies(proxies, BoundProxyFromProxyExtractor());
	      assert(proxies2 == proxies3);
	      assert(unions.size() == 2);
	      assert(true_for_all(proxies,
				  TypeCheck<ProxyPtr>(BindingData::GROUP) ->* 				 
				  hasFeature<ProxyPtr,BindingFeatures::Group>())
		     );
	      assert(true_for_all(proxies,
				  hasFeature<ProxyPtr,BindingFeatures::Group>() ->*
				  TypeCheck<ProxyPtr>(BindingData::GROUP))
		     );
	      assert(true_for_all(proxies,
				  hasFeature<ProxyPtr,BindingFeatures::Group>() ==
				  TypeCheck<ProxyPtr>(BindingData::GROUP))
		     );
	      assert(true_for_all(unions,
				  TypeCheck<UnionPtr>(BindingData::GROUP) ->* // if a group, then it must have a group feature
				  hasFeature<UnionPtr,BindingFeatures::Group>())
		     );
	      assert(true_for_some(proxies,TypeCheck<ProxyPtr>(BindingData::GROUP)));
	      assert(true_for_some(unions,TypeCheck<UnionPtr>(BindingData::GROUP)));
	      
	      // lets focus on only the singulars:
	      proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	      assert(true_for_all(proxies, FreeSingular(m_handler))); // in this case they (it) should be free
	      proxies = m_handler.extractProxiesFromProxies(proxies, GroupProxyFromSingularExtractor<ProxyPtr>());
	      assert(proxies.size() == 1);
	      assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::GROUP)));
	      proxies = m_handler.extractProxiesFromProxies(proxies, SingularProxiesFromGroupExtractor<ProxyPtr>());
	      assert(true_for_all(proxies, 
				  hasFeature<ProxyPtr,BindingFeatures::Singular>() && 
				  FreeSingular(m_handler))); 
	      
	      // lets focus on only the singulars, but this time via the unions:
	      unions = unions | hasFeature<UnionPtr,BindingFeatures::Singular>();
	      proxies = m_handler.extractProxiesFromUnions(unions, GroupProxyFromSingularExtractor<UnionPtr>());
	      assert(proxies.size() == 1);
	      assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::GROUP)));
	      
	      successExit();
	    }
	  }
      }
      break;
    case 6: 
      {
	assert(m_addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(2))) {
	    assert(unions.size() == 1);
	    successExit();
	  }
	}
      }
      break;
    case 7: 
      {
	if(binderEmptied()) {
	  successExit();
	}
      }
      break;
    case 8: 
      {
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(unions.size() != 1) {
	    log("failure: not exactly one union");
	    failExit();
	  }
	  if(true_for_all(unions, UnionProxyCountCheck(MANY_PROXIES))) {
	    successExit();
	  }
	}
      }
      break;
    case 9:
      {
	if(m_testFinished && allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(unions.size() != 2) {
	    log("failure: not exactly two unions");
	    failExit();
	  }
	  if(true_for_all(unions, UnionProxyCountCheck(MANY_PROXIES / 2))) {
	    successExit();
	  }
	}
      }
      break;
    case 10:
      {
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  if(unions.size() != 3) {
	    log("failure: not exactly three unions");
	    failExit();
	  }
	  if(true_for_all(unions, UnionProxyCountCheck(MANY_PROXIES / 3))) {
	    successExit();
	  }
	}
      }
      break;
    case 11: 
    case 12: 
    case 13: 
    case 14: 
      {
	if(binderEmptied()) {
	  successExit();
	}
      }
      break;
    case 15: 
    case 16:
      {
	if(m_testFinished && binderEmptied()) {
	  successExit();
	}
      }
      break;
    case 17:
      {
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))));	       	  
	  assert(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))));
	  successExit();
	}
      }
      break;  
    case 18:
      {
	if(m_testFinished && allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("test_concept"))));	       
	  assert(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("test_concept"))));	       
	  successExit();
	}
      }
      break;
    case 19:
      {
	if(m_testFinished && allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == proxies.size());
	  assert(proxies.size() == MANY_PROXIES);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept_\\d+"))));	       
	  if(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept_\\d+"))))
	    successExit();
	}
      }
      break;
    case 20:
      {
	if(m_testFinished && allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 1);
	  assert(proxies.size() == MANY_PROXIES);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))));	       
	  if(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))))
	    successExit();
	}
      }
      break;
    case 21:
      {
	typedef vector<shared_ptr<const CASTData<BindingData::Ambiguity> > > IssuePtrs;
	IssuePtrs issues;
	getWorkingMemoryEntries<BindingData::Ambiguity>(0,
							issues);
	if(m_testFinished && allBound() && issues.size() > 0) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  assert(issues.size() == 1);
	  const BindingData::Ambiguity& issue(*(issues[0]->getData()));
	  //assert(issue.m_missingProxyFeatures.length() == 1);
	  assert(string(issue.m_missingProxyFeatures[0]) == typeName<BindingFeatures::Colour>());
	  successExit();
	}
      }
      break;
    case 22:
      {
	assert(m_addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  successExit();
	}
      }
      break;
    case 23:
      {
	assert(m_addSignalledProxyIDs.size() <= 3);
	if(allBound()) {
	  ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  assert(true_for_all(unions,hasFeature<UnionPtr,BindingFeatures::Concept>(1))); // only one concept per union if all is correct
	  assert(true_for_all(unions,hasFeature<UnionPtr,BindingFeatures::ExistingProxyID>(1)));
	  successExit();
	}
      }
      break;
    case 24:
      {
	assert(m_addSignalledProxyIDs.size() <= 3);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == 2);
	  assert(unions.size() == 2);
	  assert(true_for_all(proxies,InportsCountCheck<ProxyPtr>(0)));
	  cout << unions << endl;
	  assert(true_for_all(unions,InportsCountCheck<UnionPtr>(0)));
	  successExit();
	}
      }
      break;
    case 25:
      {
	assert(m_addSignalledProxyIDs.size() <= 3);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == 3);
	  assert(unions.size() == 3);
	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			       ->* InportsCountCheck<ProxyPtr>(1)));
	  assert(true_for_some(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("update.*"))));
	  assert(true_for_some(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("update.*"))));
	  successExit();
	}
      }
      break;
    case 26:
      {
	assert(m_addSignalledProxyIDs.size() <= 6);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 6);
	  assert(unions.size() == 5);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			       ->* InportsCountCheck<ProxyPtr, greater_equal<unsigned int> >(1)));
	  
 	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			       && ProxyStateChecker(BindingData::BOUND))
			      ->* ProxyBindingsCountCheckViaUnion<>(2, m_handler))
		 );
 	  assert(true_for_all(proxies,
			      ProxyBindingsCountCheckViaUnion<>(2, m_handler) == ProxyBindingsCountCheck<>(2))
		 );
	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			       && ProxyStateChecker(BindingData::BOUND))
			      ->* ProxyBindingsCountCheck<>(2))
		 );
	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher(".*2")) 
			       ->* ProxyBindingsCountCheck<>(1)))
		 );
	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::RELATION) 
			      ->* ProxyBindingsCountCheck<>(1))
		 );
	  successExit();
	}
      }
      break;    
    case 27:
      {
	assert(m_addSignalledProxyIDs.size() <= 6);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 6);
	  assert(unions.size() == 3);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			       ->* InportsCountCheck<ProxyPtr, greater_equal<unsigned int> >(1)));

	  assert(true_for_all(proxies,
			      featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			      ->* ProxyBindingsCountCheck<>(2))
		 );
	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher(".*2")) 
			       ->* ProxyBindingsCountCheck<>(1)))
		 );
	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::RELATION) 
			      ->* ProxyBindingsCountCheck<>(1))
		 );
	  successExit();
	}
      }
      break;
    case 28:
      {
	assert(m_addSignalledProxyIDs.size() <= 8);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 6);
	  //assert(unions.size() == 3);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			      ->* InportsCountCheck<ProxyPtr, greater_equal<unsigned int> >(1)));

	  assert(true_for_all(proxies,
			      featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			      ->* ProxyBindingsCountCheck<>(2))
		 );
	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher(".*2")) 
			       ->* ProxyBindingsCountCheck<>(1)))
		 );
	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::RELATION) 
			      ->* ProxyBindingsCountCheck<>(1))
		 );
	  successExit();
	}
      }
      break;
    case 29:
      {
	assert(m_addSignalledProxyIDs.size() <= 3);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 1);
	  //assert(unions.size() == 3);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			      ->* InportsCountCheck<ProxyPtr>(0)));

	  successExit();
	}
      }
      break;
    case 30:
      {
	assert(m_addSignalledProxyIDs.size() <= 6);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 6);
	  assert(unions.size() == 3);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			       ->* InportsCountCheck<ProxyPtr, greater_equal<unsigned int> >(1)));

	  assert(true_for_all(proxies,
			      featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			      ->* ProxyBindingsCountCheck<>(2))
		 );
	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher(".*2")) 
			       ->* ProxyBindingsCountCheck<>(1)))
		 );
	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::RELATION) 
			      ->* ProxyBindingsCountCheck<>(1))
		 );
	  successExit();
	}
      }
      break;    
    case 31:
      {
	assert(m_addSignalledProxyIDs.size() <= 6);
	if(m_testFinished && allBound(m_proxyIDs)) {
	  ProxySet proxies = m_handler.loadProxies(m_proxyIDs);
	  UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 5);
	  assert(unions.size() == 5);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			      ->* proxyCheck(BestListCountChecker<>(1))));

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			      ->* proxyCheck(NonMatchingListCountChecker<>(4))));
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
  log("TesterMonitor::testCompleteness() just failed");
}
  
bool
TesterMonitor::binderEmptied() 
{
  return 
    !m_proxyIDs.empty() &&
    m_status.m_stable && 
    m_status.m_unboundProxies == 0 &&
    m_status.m_boundProxies == 0;
}
  
bool
TesterMonitor::allBound() 
{  
  bool result = false;
  if ( !m_proxyIDs.empty() && 
       m_status.m_stable &&
       m_overwriteSignalledProxyIDs == m_proxyIDs &&
       m_overwriteSignalledProxyIDs == m_addSignalledProxyIDs) {
    bool consistency_checked = false;
    while(!consistency_checked) {
      result = false;
      ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
      if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker(""))) { // return true after some assertions
	// some consistency checks:
	assert(true_for_all(proxies,
			    hasFeature<ProxyPtr,BindingFeatures::SourceID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::CreationTime>() &&
			    (TypeCheck<ProxyPtr>(BindingData::GROUP) == hasFeature<ProxyPtr,BindingFeatures::Group>())
			    )
	       );
	assert(true_for_all(proxies,
			    featureCheck<ProxyPtr,BindingFeatures::ThisProxyID>(ThisProxyIDConsistencyChecker<>()))
	       );
	// make sure all ThisProxyID points correctly (well... not a dead certain test, but well)
	ProxySet proxies2 = m_handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = m_handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	m_handler.reloadAllLoadedData();
      }
    }
  }
  return result;
}
  
bool
TesterMonitor::allBound(const set<string>& _proxyIDs) 
{  
  bool result = false;
  if ( !_proxyIDs.empty() && 
       m_status.m_stable) {
    bool consistency_checked = false;
    ProxySet proxies;
    while(!consistency_checked) {
      result = false;
      proxies = m_handler.loadProxies(_proxyIDs);
      if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker(""))) { // return true after some assertions
	// some consistency checks:
	assert(true_for_all(proxies,
			    hasFeature<ProxyPtr,BindingFeatures::SourceID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::CreationTime>() &&
			    (TypeCheck<ProxyPtr>(BindingData::GROUP) == hasFeature<ProxyPtr,BindingFeatures::Group>())
			    )
	       );
	assert(true_for_all(proxies,
			    featureCheck<ProxyPtr,BindingFeatures::ThisProxyID>(ThisProxyIDConsistencyChecker<>()))
	       );
	// make sure all ThisProxyID points correctly (well... not a dead certain test, but well)
	ProxySet proxies2 = m_handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = m_handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	m_handler.reloadAllLoadedData();
      }
    }
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(0, m_handler) == ProxyBindingsCountCheck<>(0))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(1, m_handler) == ProxyBindingsCountCheck<>(1))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(2, m_handler) == ProxyBindingsCountCheck<>(2))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(3, m_handler) == ProxyBindingsCountCheck<>(3))
	   );
  }
  return result;
}

string
TesterMonitor::addTwoProxiesAndOneRelation(const string& _concept1, 
					   const string& _concept2, 
					   const string& _relation_label) {
  string id1,id2;
  string rel_id;
  {
    startNewBasicProxy();
    if(!_concept1.empty()) {
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup(_concept1.c_str());
      addFeatureToCurrentProxy(concept);
    }
    id1 = storeCurrentProxy();
    m_proxyIDs.insert(id1);
  }
  {
    startNewBasicProxy();
    if(!_concept2.empty()) {
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup(_concept2.c_str()); 
      addFeatureToCurrentProxy(concept); 
    }
    id2 = storeCurrentProxy();
    m_proxyIDs.insert(id2);
  }
  {
    startNewRelationProxy();
    BindingFeatures::RelationLabel label;
    label.m_label = CORBA::string_dup(_relation_label.c_str());
    addFeatureToCurrentProxy(label); 
    addOutPortToCurrentProxy(id1, "from");
    addOutPortToCurrentProxy(id2, "to");
    rel_id = storeCurrentProxy();
    m_proxyIDs.insert(rel_id);
    bindNewProxies();
  }
  return rel_id;
}

void 
TesterMonitor::basicQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc)
{
  shared_ptr<const BindingQueries::BasicQuery> query(loadBindingDataFromWM<BindingQueries::BasicQuery>(_wmc));
  m_basicQueryAnswers[string(_wmc.m_address.m_id)] = *query;
  testCompleteness();
}
void 

TesterMonitor::advancedQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc)
{
  shared_ptr<const BindingQueries::AdvancedQuery> query(loadBindingDataFromWM<BindingQueries::AdvancedQuery>(_wmc));
  m_advancedQueryAnswers[string(_wmc.m_address.m_id)] = *query;
  /*  cout << "Advanced query answer received " << _wmc.m_address.m_id << endl;
      cout << "P2 : " << query->m_hasTheFeatureProxyIDs.length() << endl;
      cout << "U2 : " << query->m_hasTheFeatureUnionIDs.length() << endl;*/
  testCompleteness();
}

void
TesterMonitor::awaitBinding(const set<string>& _proxies) {
  awaitBinding(m_handler.loadProxies(_proxies));
}

void
TesterMonitor::awaitBinding(const ProxySet& _proxies) 
{
  ProxySet proxies = _proxies;
  while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
    log("all not bound");
    sleepProcess(1000);
    m_handler.reloadAllLoadedData();
    std::set<string> ids;
    insert_iterator<set<string> > inserter = std::inserter(ids,ids.begin());
    foreach(ProxySet::value_type prox , proxies) {
      inserter = prox.first;
    }
    proxies = m_handler.loadProxies(ids);
  }
}
  
} // namespace Binding
