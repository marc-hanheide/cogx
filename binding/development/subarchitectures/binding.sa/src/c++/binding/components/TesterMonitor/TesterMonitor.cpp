
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
  test(-1),
  testFinished(true),
  handler(*this),
  retest(0),
  statusUpdates(0),
  statusStableUpdates(0)
{
  setReceiveXarchChangeNotifications(true);
  queueBehaviour = cdl::QUEUE;
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
  test = lexical_cast<int>(itr->second);
  assert(test >= 0);
  cout << "bindingSA: " << getBindingSA() << endl;
  dynamic_cast<AbstractBindingWMRepresenter*>(this)->setBindingSubarchID(getBindingSA());
}

void TesterMonitor::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  sourceID = subarchitectureID();
  testFinished = false;
  switch(test) {
  case 0: 
    {
      log("testing to add a single basic proxy with no features");
      startNewBasicProxy();
      proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    break;
  case 1:
    {
      testFinished = false;
      log("testing to add two proxies that should NOT bind, then add a couple of queries too");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.colour = CORBA::string_dup("blue");
	addFeatureToCurrentProxy(colour);
	proxyIDs.insert(storeCurrentProxy());
      }
/*      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::OtherSourceID other;
	other.otherSourceID = CORBA::string_dup("binding.sa");
	addFeatureToCurrentProxy(other, BindingFeaturesCommon::NEGATIVE);
	proxyIDs.insert(storeCurrentProxy());
      }*/
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept, BindingFeaturesCommon::NEGATIVE); 
	proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();
      awaitBinding(proxyIDs);
      {
	BindingFeatures::Concept* query_concept = new BindingFeatures::Concept;
	query_concept->concept = CORBA::string_dup("test_concept");
	query_concept->parent.truthValue = BindingFeaturesCommon::POSITIVE;
	query_concept->parent.immediateProxyID = CORBA::string_dup("");;
	string id(newDataID());
	addToWorkingMemory(id,bindingSubarchID(),query_concept);
	BindingQueries::BasicQuery* query = new BindingQueries::BasicQuery;
	query->parameters.boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
	query->proxyID = CORBA::string_dup(proxyIDs.begin()->c_str());	
	query->featurePointer.type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	query->featurePointer.address = CORBA::string_dup(id.c_str());
	query->featurePointer.immediateProxyID = CORBA::string_dup("");
	query->answer = BindingData::INDETERMINATETB;
	query->processed = false;
	string qid(newDataID());
	addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
	basicQueryIDs.push_back(qid);
      }
      {
	BindingFeatures::Concept* query_concept = new BindingFeatures::Concept;
	query_concept->concept = CORBA::string_dup("test_concept");
	query_concept->parent.truthValue = BindingFeaturesCommon::POSITIVE;
	query_concept->parent.immediateProxyID = CORBA::string_dup("");;
	string id(newDataID());
	addToWorkingMemory(id,bindingSubarchID(),query_concept);
	BindingData::FeaturePointer ptr;
	ptr.type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	ptr.address = CORBA::string_dup(id.c_str());
	BindingQueries::AdvancedQuery* query = new BindingQueries::AdvancedQuery;
	query->parameters.boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
	query->featurePointer.type = CORBA::string_dup(typeName<BindingFeatures::Concept>().c_str());
	query->featurePointer.address = CORBA::string_dup(id.c_str());
	query->featurePointer.immediateProxyID = CORBA::string_dup("");	
	query->hasTheFeatureProxyIDs.length(0);
	query->hasTheFeatureUnionIDs.length(0);
	query->matchingProxyIDs.length(0);
	query->matchingUnionIDs.length(0);
	query->nonMatchingProxyIDs.length(0);
	query->nonMatchingUnionIDs.length(0);
	query->processed = false;
	
	string qid(newDataID());
	addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
	advancedQueryIDs.push_back(qid);
      }
      sleepProcess(2000);
      testFinished = true;
    }
    break;
  case 2:
    {
      log("testing to add two proxies that should bind");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	proxyIDs.insert(storeCurrentProxy());
	//bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept); 
	proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
    }
    break;
  case 3:
    {
      testFinished = false;
      log("testing to add two proxies and a relation between them, no bindings");
      addTwoProxiesAndOneRelation();
      bindNewProxies();
      awaitBinding(proxyIDs);
      testFinished = true;
    }
    break;
  case 4:
    {
      testFinished = false;
      log("testing to add two proxies and a relation between them (twice, and the relation should bind as a consequence)");
      addTwoProxiesAndOneRelation();
      sleepProcess(3000);
      log("------- time to add more proxies");
      addTwoProxiesAndOneRelation();
      bindNewProxies();
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      testFinished = true;
    }
    break;
  case 5:
    {
      log("testing a single unbounded group proxy");
      startNewGroupProxy(0);
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("test_concept");
      addFeatureToCurrentProxy(concept);
      proxyIDs.insert(storeCurrentProxy());
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
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
      sleepProcess(1000);
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept); 
	proxyIDs.insert(storeCurrentProxy());
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
	concept.concept = CORBA::string_dup(string("test_concept" + lexical_cast<string>(i)).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
	bindNewProxies();
      }
      sleepProcess(1000);
      for(set<string>::const_iterator itr = proxyIDs.begin();
	  itr != proxyIDs.end();
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
	concept.concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
      }
      bindNewProxies();
    }
    break;
  case 9:
    {
      testFinished = false;
      log("Add two sets of proxies that all should bind within the sets, and call bindNewProxies only once");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup(string(string("test_concept") + ((i%2)?"_1":"_2")).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
      }
      bindNewProxies();
      awaitBinding(proxyIDs);
    }
    testFinished = true;
    break;
  case 10:
    {
      log("Add three sets of proxies that all should bind within the sets, and call bindNewProxies only once");
      for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup(string(string("test_concept_") + lexical_cast<string>(i%3 + 1)).c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
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
	concept.concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
      bindNewProxies();
      }
      sleepProcess(5000);
      for(set<string>::const_iterator itr = proxyIDs.begin();
	  itr != proxyIDs.end();
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
	concept.concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
      }
      bindNewProxies();
      sleepProcess(5000);
      for(set<string>::const_iterator itr = proxyIDs.begin();
	  itr != proxyIDs.end();
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
	concept.concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
	bindNewProxies();
      }
      for(set<string>::const_iterator itr = proxyIDs.begin();
	  itr != proxyIDs.end();
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
	concept.concept = CORBA::string_dup(string("test_concept").c_str());
	addFeatureToCurrentProxy(concept);
	string id = storeCurrentProxy();
	proxyIDs.insert(id);
      }
      bindNewProxies();
      for(set<string>::const_iterator itr = proxyIDs.begin();
	  itr != proxyIDs.end();
	  ++itr) {
	deleteExistingProxy(*itr);
      }
    }
    break;
  case 15:
    {
      testFinished = false;
      log("for ten times in a row: Add few proxies that should bind, bind them incremently, and delete them right away (in a mixed order)");
      for(unsigned int j = 0 ; j < 10 ; ++j) {
	log(string("gonna create ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	deque<string> ids;
	for(unsigned int i = 0 ; i < FEW_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  proxyIDs.insert(id);
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
      testFinished = true;
    }
  case 16:
    {
      testFinished = false;
      log("for ten times in a row: Add few proxies that should bind, bind them incremently, wait a bit and then delete them (in a mixed order)");
      for(unsigned int j = 0 ; j < 10 ; ++j) {
	log(string("gonna create ") + lexical_cast<string>(FEW_PROXIES) +" proxies");
	deque<string> ids;
	for(unsigned int i = 0 ; i < FEW_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  proxyIDs.insert(id);
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
      testFinished = true;
    }
    break;
  case 17:
    {
      testFinished = false;
      log("testing to add a single basic proxy and then update it after some delay");
      string id;
      {
	//addTwoProxiesAndOneRelation();
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id = storeCurrentProxy();
	proxyIDs.insert(id);
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
	concept.concept = CORBA::string_dup("updated_test_concept");
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
      testFinished = true;
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
    }
    break;  
  case 18:
    {
      testFinished = false;
      log("testing to add a single basic proxy and then update with no changes after some delay");
      string id;
      {
	//addTwoProxiesAndOneRelation();
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id = storeCurrentProxy();
	proxyIDs.insert(id);
	bindNewProxies();
      }
      sleepProcess(1000);
      {
	changeExistingProxy(id);
	string id2 = storeCurrentProxy();
	assert(id == id2);
	bindNewProxies();
      }
      testFinished = true;
    }
    break;
  case 19:
    {
      testFinished = false;
      log("testing to add many proxies (that should bind) and then update them after they are bound such that they don't bind anymore");
      {
	for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup(string("test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  proxyIDs.insert(id);
	}
	bindNewProxies();
      }
      sleepProcess(2000);
      // make sure all proxies are first bound
      ProxySet proxies = handler.loadProxies(proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	handler.reloadAllLoadedData();
	proxies = handler.loadProxies(proxyIDs);
      }
      log("Now the proxies will be updated");
      UnionSet unions = handler.extractUnionsFromProxies(proxies);
      assert(unions.size() == 1);
      {
	unsigned int nr = 0;
	// now change them
	foreach(string id,proxyIDs) {
	  changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup((string("updated_test_concept_") + lexical_cast<string>(nr++)).c_str());
	  addFeatureToCurrentProxy(concept);
	  string id2 = storeCurrentProxy();
	  assert(id == id2);
	}
	bindNewProxies();
      }
      testFinished = true;
    }
    break;
  case 20:
    {
      testFinished = false;
      log("testing to add many proxies (that should NOT bind) and then update them after they are bound such that they DO bind");
      {
	for(unsigned int i = 0 ; i < MANY_PROXIES; ++i) {
	  startNewBasicProxy();
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup((string("test_concept_") + lexical_cast<string>(i)).c_str());
	  addFeatureToCurrentProxy(concept);
	  string id = storeCurrentProxy();
	  proxyIDs.insert(id);
	}
	bindNewProxies();
      }
      sleepProcess(2000);
      // make sure all proxies are first bound
      ProxySet proxies = handler.loadProxies(proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	handler.reloadAllLoadedData();
	proxies = handler.loadProxies(proxyIDs);
      }
      log("Now the proxies will be updated");
      UnionSet unions = handler.extractUnionsFromProxies(proxies);
      assert(unions.size() == MANY_PROXIES);
      {
	// now change them
	foreach(string id,proxyIDs) {
	  changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
	  BindingFeatures::Concept concept;
	  concept.concept = CORBA::string_dup(string("updated_test_concept").c_str());
	  addFeatureToCurrentProxy(concept);
	  string id2 = storeCurrentProxy();
	  assert(id == id2);
	}
	bindNewProxies();
	sleepProcess(10000);
      }
      testFinished = true;
    }
    break;
  case 21:
    {
      log("adding two proxies that should not bind, then add a third one that is ambiguous, then check that there isa disambiguation struct on WM");
      {
	startNewBasicProxy(); // proxy blue
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.colour = CORBA::string_dup("blue");
	addFeatureToCurrentProxy(colour);
	proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy(); // proxy red
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.colour = CORBA::string_dup("red");
	addFeatureToCurrentProxy(colour);
	proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();
      ProxySet proxies = handler.loadProxies(proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	sleepProcess(10);
	handler.reloadAllLoadedData();
	proxies = handler.loadProxies(proxyIDs);
      }
      {
	startNewBasicProxy(); // proxy with unknown colour
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	proxyIDs.insert(storeCurrentProxy());
      }
      testFinished = true;   
      bindNewProxies();  
    }
    break;
  case 22:
    {
      log("testing to add two proxies that should NOT bind (due to OtherProxyID)");
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::OtherSourceID other;
	other.otherSourceID = CORBA::string_dup(subarchitectureID().c_str());
	addFeatureToCurrentProxy(other, BindingFeaturesCommon::NEGATIVE);
	proxyIDs.insert(storeCurrentProxy());
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
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	id1 = storeCurrentProxy();
	proxyIDs.insert(id1);
      }
      {
	startNewBasicProxy();
	BindingFeatures::ExistingProxyID existing;
	existing.existingProxyID = CORBA::string_dup(id1.c_str());
	addFeatureToCurrentProxy(existing);
	proxyIDs.insert(storeCurrentProxy());
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::ExistingProxyID existing;
	existing.existingProxyID = CORBA::string_dup(id1.c_str());
	addFeatureToCurrentProxy(existing, BindingFeaturesCommon::NEGATIVE);
	proxyIDs.insert(storeCurrentProxy());
      }
      bindNewProxies();      
    }
    break;  
  case 24:
    {
      testFinished = false;
      log("testing to add two proxies and a relation, then bind, then delete the relation");
      string rel_id = addTwoProxiesAndOneRelation();
      bindNewProxies();      
      // make sure all proxies are first bound
      ProxySet proxies = handler.loadProxies(proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	handler.reloadAllLoadedData();
	proxies = handler.loadProxies(proxyIDs);
      }
      deleteExistingProxy(rel_id);      
      proxyIDs.erase(rel_id);
      awaitBinding(proxyIDs);
      sleepProcess(10000);
      testFinished = true;
    }
    break;
  case 25:
    {
      testFinished = false;
      log("testing to add two proxies and a relation, then bind, then update one of the related proxies");
      string rel_id = addTwoProxiesAndOneRelation();
      bindNewProxies();      
      // make sure all proxies are first bound
      ProxySet proxies = handler.loadProxies(proxyIDs);
      while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
	log("all not bound");
	sleepProcess(1000);
	handler.reloadAllLoadedData();
	proxies = handler.loadProxies(proxyIDs);
      }
      const string& id(*proxyIDs.begin());
      changeExistingProxy(id,list_of(typeName<BindingFeatures::Concept>()));
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup("updated_test_concept");
      addFeatureToCurrentProxy(concept);
      storeCurrentProxy();
      bindNewProxies();
      testFinished = true;
      awaitBinding(proxyIDs);
    }
    break;  
  case 26:
    {
      testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      bindNewProxies();      
      // make sure all proxies are first bound
      awaitBinding(proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","test_concept_not2","test_label");
      testFinished = true;
      awaitBinding(proxyIDs);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(4000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
    }
    break;
  case 27:
    {
      testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones (zip up)");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      // make sure all proxies are first bound
      awaitBinding(proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","","test_label");
      awaitBinding(proxyIDs);
      testFinished = true;      
    }
    break;
  case 28:
    {
      testFinished = false;
      log("testing to add two proxies and two relations, then remove the relations, add new proxy and add a relation to it instead");
      string id1,id2,id3;
      string rel_id1,rel_id2;
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept1");
	addFeatureToCurrentProxy(concept);
	id1 = storeCurrentProxy();
	proxyIDs.insert(id1);
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept2");
	addFeatureToCurrentProxy(concept); 
	id2 = storeCurrentProxy();
	proxyIDs.insert(id2);
      }
      bindNewProxies();
      rel_id1 = addSimpleRelation(id1,id2,"rel_label1");
      rel_id2 = addSimpleRelation(id2,id1,"rel_label1");
      proxyIDs.insert(rel_id1);
      proxyIDs.insert(rel_id2);
      bindNewProxies();
      awaitBinding(proxyIDs);
      log("BEFORE DELETION");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      deleteExistingProxy(rel_id1);
      deleteExistingProxy(rel_id2);
      proxyIDs.erase(rel_id1);
      proxyIDs.erase(rel_id2);
      awaitBinding(proxyIDs);
      sleepProcess(1000);
      log("AFTER DELETION OF RELATIONS");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000); // sleep for a while to allow dotviewer to finish
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("test_concept3");
	addFeatureToCurrentProxy(concept); 
	id3 = storeCurrentProxy();
	proxyIDs.insert(id3);
      }
      rel_id1 = addSimpleRelation(id1,id3,"rel_label2");
      rel_id2 = addSimpleRelation(id3,id1,"rel_label2");
      bindNewProxies();
      proxyIDs.insert(rel_id1);
      proxyIDs.insert(rel_id2);
      log("AFTER ADDITION OF NEW RELAIONS");
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000); // sleep for a while to allow dotviewer to finish
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      testFinished = true;      
    }
    break;  
  case 29:
    {
      testFinished = false;
      log("testing to add two proxies and one relations, and then remove the relation");
      string id;
      string rel_id;
      {
	startNewBasicProxy();
	id = storeCurrentProxy();
	proxyIDs.insert(id);
      }
      bindNewProxies();
      rel_id = addSimpleRelation(id,id,"rel_label");
      proxyIDs.insert(rel_id);
      bindNewProxies();
      awaitBinding(proxyIDs);
      const LBindingProxy& da_proxy(proxyLocalCache[id]);
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
      proxyIDs.erase(rel_id);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      awaitBinding(proxyIDs);
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
      testFinished = true;      
    }
    break;
  case 30:
    {
      testFinished = false;
      log("testing to add two proxies and a relation, then another two proxies and a relation of which one should match one of the first ones (the zip-up, cf. test 26)");
      string rel_id = addTwoProxiesAndOneRelation("test_concept1","test_concept2","test_label");
      bindNewProxies();      
      // make sure all proxies are first bound
      awaitBinding(proxyIDs);
      addTwoProxiesAndOneRelation("test_concept1","","test_label");
      testFinished = true;
      awaitBinding(proxyIDs);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(4000);
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(2000);
    }
    break;
  case 31:
    {
      testFinished = false;
      log("testing to add three proxies and a pair of relations. Nothing should bind");
      {
	string id1,id2,id3;
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.concept = CORBA::string_dup("concept1");
	addFeatureToCurrentProxy(concept);	
	id1 = storeCurrentProxy();
	proxyIDs.insert(id1);
	bindNewProxies();
	startNewBasicProxy();
	concept.concept = CORBA::string_dup("concept2"); 
	addFeatureToCurrentProxy(concept); 
	id2 = storeCurrentProxy();
	proxyIDs.insert(id2);
	bindNewProxies();
	startNewRelationProxy();	      
	BindingFeatures::RelationLabel label;	
	label.label = CORBA::string_dup("label");
	addFeatureToCurrentProxy(label); 
	addOutPortToCurrentProxy(id1, "from");
	addOutPortToCurrentProxy(id2, "to");
	string rel_id = storeCurrentProxy();
	proxyIDs.insert(rel_id);	
	bindNewProxies();
	startNewBasicProxy();
	concept.concept = CORBA::string_dup("concept3"); 
	addFeatureToCurrentProxy(concept); 
	id3 = storeCurrentProxy();
	proxyIDs.insert(id3);
	bindNewProxies();
	startNewRelationProxy();
	addFeatureToCurrentProxy(label); 
	addOutPortToCurrentProxy(id3, "from");
	addOutPortToCurrentProxy(id2, "to");
	rel_id = storeCurrentProxy();
	proxyIDs.insert(rel_id);	
      }
      bindNewProxies();
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
      testFinished = true;
      addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);
      sleepProcess(1000);
    }
    break;
  default:
    cerr << "incorrect test number: " << test << endl;
    failExit();
  }
  testFinished = true;
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
  sleepProcess(5000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
}
  
void 
TesterMonitor::bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.address.id);
  //const LBindingProxy& binding_proxy(proxyLocalCache[id]);
  addSignalledProxyIDs.insert(id);
}

void 
TesterMonitor::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.address.id);
  overwriteSignalledProxyIDs.insert(id);
  try {
    status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), statusID);      
  } catch(const DoesNotExistOnWMException& _e) {
    cerr << "Caught this in TesterMonitor::bindingProxyUpdated: " + string(_e.what());
    abort();
  }
  testCompleteness();
}

void 
TesterMonitor::bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), statusID);  
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in TesterMonitor::bindingUnionUpdated: " + string(_e.what()));
    abort();
  }
  testCompleteness();
}

void 
TesterMonitor::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  try {
    statusUpdates++;
    if(statusID.empty())
      statusID = _wmc.address.id;
    else
      assert(statusID == string(_wmc.address.id));
    status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), statusID);  
    if(status.stable)
      statusStableUpdates++;    
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
  handler.purgeAllLoadedData();
  try {
    switch(test) {
    case 0: 
      {
	assert(addSignalledProxyIDs.size() <= 1);
	if(allBound()) {
	  successExit();
	}
      }
      break;  
    case 1: 
      {
	assert(addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies, proxyCheck(BestListCountChecker<>(1))));
	  assert(true_for_all(proxies, proxyCheck(NonMatchingListCountChecker<>(1))));
	  assert(unions.size() == 2);
	  if(!basicQueryAnswers.empty() &&
	     !advancedQueryAnswers.empty() &&
	     true_for_all(unions, 
			  UnionProxyCountCheck(1) && 
			  !UnionIsBoundToAllOfProxies(proxies) &&
			  UnionIsBoundToOneOfProxies(proxies))) {
	    // some testing of the tests
	    assert(basicQueryAnswers.begin()->second.answer == BindingData::TRUETB);
	    assert(advancedQueryAnswers.begin()->second.hasTheFeatureProxyIDs.length() == 2);
	    assert(advancedQueryAnswers.begin()->second.hasTheFeatureUnionIDs.length() == 2);
	    assert(advancedQueryAnswers.begin()->second.matchingProxyIDs.length() == 1);
	    assert(advancedQueryAnswers.begin()->second.matchingUnionIDs.length() == 1);
	    
	    assert(advancedQueryAnswers.begin()->second.nonMatchingProxyIDs.length() == 1);
	    assert(advancedQueryAnswers.begin()->second.nonMatchingUnionIDs.length() == 1);
	    ProxySet matching_proxies = handler.loadProxies(advancedQueryAnswers.begin()->second.matchingProxyIDs);
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
				      UnionIsBoundToProxy(*overwriteSignalledProxyIDs.begin()) &&
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
				 UnionIsBoundToProxy(*overwriteSignalledProxyIDs.begin()) &&
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
	assert(addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(2))) {
	    assert(unions.size() == 1);
	    successExit();
	  }
	}
      }
      break;
    case 3: 
      {
	assert(addSignalledProxyIDs.size() <= 3);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  if(true_for_all(unions, UnionProxyCountCheck(1))) {
	    // testing of test operators
	    proxies = subset(proxies,
			     TypeCheck<ProxyPtr>(BindingData::BASIC)); // remove all that are not basic
	    // alt:
	    proxies = proxies | TypeCheck<ProxyPtr>(BindingData::BASIC);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    proxies = handler.extractProxiesFromProxies(proxies, InPortsExtractor<ProxyPtr>()); // should now only contain the one relation
	    assert(proxies.size() == 1);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(true_for_all(proxies,
				TypeCheck<ProxyPtr>(BindingData::RELATION) ->* 
				hasFeature<ProxyPtr,BindingFeatures::RelationLabel>()));	  
	    proxies = handler.extractProxiesFromProxies(proxies, OutPortsExtractor<ProxyPtr>());  // should now contain only the related proxies
	    assert(proxies.size() == 2);
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    ProxySet one_proxy; one_proxy.insert(*proxies.begin()); // store only one basic proxy
	    one_proxy += one_proxy; // should be a noop.
	    assert(one_proxy.size() == 1);
	    // get the relation proxy again
	    proxies = 
	      handler.extractProxiesFromProxies(one_proxy,
						  InPortsExtractor<ProxyPtr>());
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(true_for_all(one_proxy,TypeCheck<ProxyPtr>(BindingData::BASIC)));
	    assert(proxies.size() == 1);
	    // get all proxies via the out ports, one by one
	    proxies = 
	      handler.extractProxiesFromProxies(one_proxy,
						  InPortsExtractor<ProxyPtr>() || OutPortsExtractor<ProxyPtr>());
	    cout << proxies << endl;
	    assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::RELATION)));
	    assert(proxies.size() == 1);
	    proxies += // obs += here... 
	      handler.extractProxiesFromProxies(proxies,
						  InPortsExtractor<ProxyPtr>() || OutPortsExtractor<ProxyPtr>());
	    assert(proxies.size() == 3);
	    
	    // now, extract over all in and outports recursively instead
	    proxies = 
	      handler.extractProxiesFromProxies(one_proxy,
						  RecursiveExtractor<ProxyPtr>(OutPortsExtractor<ProxyPtr>()||InPortsExtractor<ProxyPtr>(),
									       handler.loadProxyFctPtr()));
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
	assert(addSignalledProxyIDs.size() <= 6);
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	assert(addSignalledProxyIDs.size() <= 2);
	if(!proxyIDs.empty() && 
	   status.stable &&
	   overwriteSignalledProxyIDs.size() > 1 &&
	   overwriteSignalledProxyIDs == addSignalledProxyIDs) 
	  {
	    ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	    UnionSet unions = handler.extractUnionsFromProxies(proxies);
	    if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND)) &&
	       true_for_all(unions, UnionProxyCountCheck(1))) { // each union has one proxy
	      // test 2 ways of extracting bound unions
	      ProxySet proxies2 = handler.extractProxiesFromUnions(unions);
	      ProxySet proxies3 = handler.extractProxiesFromProxies(proxies, BoundProxyFromProxyExtractor());
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
	      assert(true_for_all(proxies, FreeSingular(handler))); // in this case they (it) should be free
	      proxies = handler.extractProxiesFromProxies(proxies, GroupProxyFromSingularExtractor<ProxyPtr>());
	      assert(proxies.size() == 1);
	      assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::GROUP)));
	      proxies = handler.extractProxiesFromProxies(proxies, SingularProxiesFromGroupExtractor<ProxyPtr>());
	      assert(true_for_all(proxies, 
				  hasFeature<ProxyPtr,BindingFeatures::Singular>() && 
				  FreeSingular(handler))); 
	      
	      // lets focus on only the singulars, but this time via the unions:
	      unions = unions | hasFeature<UnionPtr,BindingFeatures::Singular>();
	      proxies = handler.extractProxiesFromUnions(unions, GroupProxyFromSingularExtractor<UnionPtr>());
	      assert(proxies.size() == 1);
	      assert(true_for_all(proxies,TypeCheck<ProxyPtr>(BindingData::GROUP)));
	      
	      successExit();
	    }
	  }
      }
      break;
    case 6: 
      {
	assert(addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	if(testFinished && binderEmptied()) {
	  successExit();
	}
      }
      break;
    case 17:
      {
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))));	       	  
	  assert(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("updated_test_concept"))));
	  successExit();
	}
      }
      break;  
    case 18:
      {
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(true_for_all(proxies,featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("test_concept"))));	       
	  assert(true_for_all(unions,featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("test_concept"))));	       
	  successExit();
	}
      }
      break;
    case 19:
      {
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	if(testFinished && allBound() && issues.size() > 0) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  assert(issues.size() == 1);
	  const BindingData::Ambiguity& issue(*(issues[0]->getData()));
	  //assert(issue.missingProxyFeatures.length() == 1);
	  assert(string(issue.missingProxyFeatures[0]) == typeName<BindingFeatures::Colour>());
	  successExit();
	}
      }
      break;
    case 22:
      {
	assert(addSignalledProxyIDs.size() <= 2);
	if(allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  successExit();
	}
      }
      break;
    case 23:
      {
	assert(addSignalledProxyIDs.size() <= 3);
	if(allBound()) {
	  ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(unions.size() == 2);
	  assert(true_for_all(unions,hasFeature<UnionPtr,BindingFeatures::Concept>(1))); // only one concept per union if all is correct
	  assert(true_for_all(unions,hasFeature<UnionPtr,BindingFeatures::ExistingProxyID>(1)));
	  successExit();
	}
      }
      break;
    case 24:
      {
	assert(addSignalledProxyIDs.size() <= 3);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	assert(addSignalledProxyIDs.size() <= 3);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
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
	assert(addSignalledProxyIDs.size() <= 6);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
	  assert(proxies.size() == 6);
	  assert(unions.size() == 5);

	  assert(true_for_all(proxies,
			      TypeCheck<ProxyPtr>(BindingData::BASIC) 
			       ->* InportsCountCheck<ProxyPtr, greater_equal<unsigned int> >(1)));
	  
 	  assert(true_for_all(proxies,
			      (featureCheck<ProxyPtr,BindingFeatures::Concept,ConceptRegexMatcher >(ConceptRegexMatcher(".*1")) 
			       && ProxyStateChecker(BindingData::BOUND))
			      ->* ProxyBindingsCountCheckViaUnion<>(2, handler))
		 );
 	  assert(true_for_all(proxies,
			      ProxyBindingsCountCheckViaUnion<>(2, handler) == ProxyBindingsCountCheck<>(2))
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
	assert(addSignalledProxyIDs.size() <= 6);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
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
	assert(addSignalledProxyIDs.size() <= 8);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
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
	assert(addSignalledProxyIDs.size() <= 3);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
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
	assert(addSignalledProxyIDs.size() <= 6);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
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
	assert(addSignalledProxyIDs.size() <= 6);
	if(testFinished && allBound(proxyIDs)) {
	  ProxySet proxies = handler.loadProxies(proxyIDs);
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  
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
  retest = 0;  
  log("TesterMonitor::testCompleteness() just failed");
}
  
bool
TesterMonitor::binderEmptied() 
{
  return 
    !proxyIDs.empty() &&
    status.stable && 
    status.unboundProxies == 0 &&
    status.boundProxies == 0;
}
  
bool
TesterMonitor::allBound() 
{  
  bool result = false;
  if ( !proxyIDs.empty() && 
       status.stable &&
       overwriteSignalledProxyIDs == proxyIDs &&
       overwriteSignalledProxyIDs == addSignalledProxyIDs) {
    bool consistency_checked = false;
    while(!consistency_checked) {
      result = false;
      ProxySet proxies = handler.loadProxies(overwriteSignalledProxyIDs);
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
	ProxySet proxies2 = handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	handler.reloadAllLoadedData();
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
       status.stable) {
    bool consistency_checked = false;
    ProxySet proxies;
    while(!consistency_checked) {
      result = false;
      proxies = handler.loadProxies(_proxyIDs);
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
	ProxySet proxies2 = handler.extractProxiesFromProxies(proxies,ProxyFromThisProxyIDExtractor<ProxyPtr>());
	assert(proxies == proxies2);
	// have a look at the singulars:
	proxies = proxies | hasFeature<ProxyPtr,BindingFeatures::Singular>();
	// and make sure they're all part of groups
	proxies = handler.extractProxiesFromProxies(proxies,GroupProxyFromSingularExtractor<ProxyPtr>());
	assert(true_for_all(proxies, TypeCheck<ProxyPtr>(BindingData::GROUP)));
	result = true;
      }
      if(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this))) {
	consistency_checked = true;
      } else {
	handler.reloadAllLoadedData();
      }
    }
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(0, handler) == ProxyBindingsCountCheck<>(0))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(1, handler) == ProxyBindingsCountCheck<>(1))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(2, handler) == ProxyBindingsCountCheck<>(2))
	   );
    assert(true_for_all(proxies,
			ProxyBindingsCountCheckViaUnion<>(3, handler) == ProxyBindingsCountCheck<>(3))
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
      concept.concept = CORBA::string_dup(_concept1.c_str());
      addFeatureToCurrentProxy(concept);
    }
    id1 = storeCurrentProxy();
    proxyIDs.insert(id1);
  }
  {
    startNewBasicProxy();
    if(!_concept2.empty()) {
      BindingFeatures::Concept concept;
      concept.concept = CORBA::string_dup(_concept2.c_str()); 
      addFeatureToCurrentProxy(concept); 
    }
    id2 = storeCurrentProxy();
    proxyIDs.insert(id2);
  }
  {
    startNewRelationProxy();
    BindingFeatures::RelationLabel label;
    label.label = CORBA::string_dup(_relation_label.c_str());
    addFeatureToCurrentProxy(label); 
    addOutPortToCurrentProxy(id1, "from");
    addOutPortToCurrentProxy(id2, "to");
    rel_id = storeCurrentProxy();
    proxyIDs.insert(rel_id);
    bindNewProxies();
  }
  return rel_id;
}

void 
TesterMonitor::basicQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc)
{
  shared_ptr<const BindingQueries::BasicQuery> query(loadBindingDataFromWM<BindingQueries::BasicQuery>(_wmc));
  basicQueryAnswers[string(_wmc.address.id)] = *query;
  testCompleteness();
}
void 

TesterMonitor::advancedQueryAnswered(const cast::cdl::WorkingMemoryChange & _wmc)
{
  shared_ptr<const BindingQueries::AdvancedQuery> query(loadBindingDataFromWM<BindingQueries::AdvancedQuery>(_wmc));
  advancedQueryAnswers[string(_wmc.address.id)] = *query;
  /*  cout << "Advanced query answer received " << _wmc.address.id << endl;
      cout << "P2 : " << query->hasTheFeatureProxyIDs.length() << endl;
      cout << "U2 : " << query->hasTheFeatureUnionIDs.length() << endl;*/
  testCompleteness();
}

void
TesterMonitor::awaitBinding(const set<string>& _proxies) {
  awaitBinding(handler.loadProxies(_proxies));
}

void
TesterMonitor::awaitBinding(const ProxySet& _proxies) 
{
  ProxySet proxies = _proxies;
  while(!true_for_all(proxies,ProxyStateChecker(BindingData::BOUND))) {
    log("all not bound");
    sleepProcess(1000);
    handler.reloadAllLoadedData();
    std::set<string> ids;
    insert_iterator<set<string> > inserter = std::inserter(ids,ids.begin());
    foreach(ProxySet::value_type prox , proxies) {
      inserter = prox.first;
    }
    proxies = handler.loadProxies(ids);
  }
}
  
} // namespace Binding
