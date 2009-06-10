#include "ComaTester.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/utils/QueryUtils.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
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
    return new Binding::ComaTester(_id);
  }
}


ComaTester::ComaTester(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  TesterMonitor(_id)
{
  setReceiveXarchChangeNotifications(true);
  m_queueBehaviour = cdl::QUEUE;
}

static const unsigned int MANY_PROXIES = 24;
static const unsigned int FEW_PROXIES = 6;

void
ComaTester::start() {    
  TesterMonitor::start();
}

void 
ComaTester::configure(map<string,string> & _config) {
  TesterMonitor::configure(_config);
}

void ComaTester::runComponent() {
  sleepProcess(1000); // sleep for a second to allow the rest to be properly started
  m_sourceID = subarchitectureID();
  switch(m_test) {
  case 0: 
    {
      m_testFinished = false;
      string thing_id, ball_id;
      {
	log("testing to add two proxies (blue) 'thing' and 'ball' which should match according to coma");
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("thing");
	addFeatureToCurrentProxy(concept);
	BindingFeatures::Colour colour;
	colour.m_colour = CORBA::string_dup("blue");
	addFeatureToCurrentProxy(colour);
	thing_id = storeCurrentProxy();
	m_proxyIDs.insert(thing_id);
	bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("ball");
	addFeatureToCurrentProxy(concept);
	ball_id = storeCurrentProxy();
	m_proxyIDs.insert(ball_id);
	bindNewProxies();
      }
      awaitBinding(m_proxyIDs);
      sleepProcess(1000);
      BindingFeatures::Concept concept;
      
      QueryUtils utils(dynamic_cast<cast::PrivilegedManagedProcess&>(*this),
		       bindingSubarchID());
      string q;

      concept.m_concept = CORBA::string_dup("mug");
      q = utils.askBasicQuery(concept,
			      ball_id);
      m_basicQueryIDs.push_back(q); //0

      concept.m_concept = CORBA::string_dup("mug");
      q = utils.askBasicQuery(concept,
			      ball_id,
			      utils.constructQueryParameters(BindingQueries::EXCLUDE_BOUND));
      m_basicQueryIDs.push_back(q); //1

      concept.m_concept = CORBA::string_dup("ball");      
      q = utils.askBasicQuery(concept,
			      ball_id);
      m_basicQueryIDs.push_back(q);//2

      concept.m_concept = CORBA::string_dup("ball");      
      q = utils.askBasicQuery(concept,
			      ball_id,
			      utils.constructQueryParameters(BindingQueries::EXCLUDE_BOUND));
      m_basicQueryIDs.push_back(q);//3

      concept.m_concept = CORBA::string_dup("thing");      
      q = utils.askBasicQuery(concept,
			      ball_id);
      m_basicQueryIDs.push_back(q);//4

      q = utils.askBasicQuery<BindingFeatures::Concept>(ball_id);
      m_basicQueryIDs.push_back(q);//5
      
      q = utils.askBasicQuery<BindingFeatures::Colour>(ball_id);
      m_basicQueryIDs.push_back(q);//6
      q = utils.askBasicQuery<BindingFeatures::Colour>(ball_id,
						       utils.constructQueryParameters(BindingQueries::EXCLUDE_BOUND));
      m_basicQueryIDs.push_back(q);//7
      
      concept.m_concept = CORBA::string_dup("mug");      
      q = utils.askAdvancedQuery(concept);
      cout << "q: " << q << endl;
      m_advancedQueryIDs.push_back(q); //0

      concept.m_concept = CORBA::string_dup("mug");      
      q = utils.askAdvancedQuery(concept,
				 utils.constructQueryParameters(BindingQueries::EXCLUDE_BOUND));
      cout << "q: " << q << endl;
      m_advancedQueryIDs.push_back(q); //1
      
      concept.m_concept = CORBA::string_dup("thing");      
      q = utils.askAdvancedQuery(concept);
      m_advancedQueryIDs.push_back(q);//2

      concept.m_concept = CORBA::string_dup("ball");      
      q = utils.askAdvancedQuery(concept);
      m_advancedQueryIDs.push_back(q);//3
      
      // ask what proxies has a colour
      q = utils.askAdvancedQuery<BindingFeatures::Colour>();
      m_advancedQueryIDs.push_back(q);//4

      // ask what proxies has a colour (including the union features in the answer)
      q = utils.askAdvancedQuery<BindingFeatures::Colour>(utils.constructQueryParameters(BindingQueries::EXCLUDE_BOUND));
      m_advancedQueryIDs.push_back(q);//5
      
    }
    m_testFinished = true;
    break;
  case 1: 
    log("testing to add six proxies , 'Sofa', 'sofa', 'Couch', 'couch', 'Object' and 'object' which should match according to coma");
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("Sofa");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("sofa");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("Couch");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("couch");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("Object");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("object");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
    }
    bindNewProxies();
    m_testFinished = true;
    break;
  case 2: 
    {
      m_testFinished = false;
      log("testing to add two proxies 'blurb1' and 'blurb2' which should not match according to coma (since they're not in the ontology)");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("blurb1");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    {
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("blurb2");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    m_testFinished = true;
    break;  
  case 3: 
    sleepProcess(5000); // let coma update its stuff first
    {
      m_testFinished = false;
      log("testing to add two proxies 'room' and AreaID=71 which should match according to coma");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("room");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    {
      startNewBasicProxy();
      BindingFeatures::AreaID area;
      area.m_id = 71;
      addFeatureToCurrentProxy(area);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    m_testFinished = true;
    break;
  case 4: 
    sleepProcess(5000); // let coma update its stuff first
    {
      m_testFinished = false;
      log("testing to add two proxies 'room' and AreaID=73 (a corridor) which should not match according to coma");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("room");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    {
      startNewBasicProxy();
      BindingFeatures::AreaID area;
      area.m_id = 73;
      addFeatureToCurrentProxy(area);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    m_testFinished = true;
    break;
  case 5: 
    sleepProcess(5000); // let coma update its stuff first
    {
      m_testFinished = false;
      log("testing to add two proxies 'room' and AreaID=72 (an office) which should  match according to coma");
      startNewBasicProxy();
      BindingFeatures::Concept concept;
      concept.m_concept = CORBA::string_dup("room");
      addFeatureToCurrentProxy(concept);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    {
      startNewBasicProxy();
      BindingFeatures::AreaID area;
      area.m_id = 72;
      addFeatureToCurrentProxy(area);
      m_proxyIDs.insert(storeCurrentProxy());
      bindNewProxies();
    }
    m_testFinished = true;
    break;
  case 6: 
    {
      m_testFinished = false;
      sleepProcess(5000); // let coma update its stuff first
      log("testing to add three proxies AreaID=71, AreaID=74, (rooms) and an 'office' and a 'room' which should not match and match respectively according to coma");
      log("then ask about kitchens...");
      string kitchen_id;
      {
	startNewBasicProxy();
	BindingFeatures::AreaID area;
	area.m_id = 71;
	addFeatureToCurrentProxy(area);
	kitchen_id = storeCurrentProxy(); 
	m_proxyIDs.insert(kitchen_id);
	bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::AreaID area;
	area.m_id = 74;
	addFeatureToCurrentProxy(area);
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("office");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
      {
	startNewBasicProxy();
	BindingFeatures::Concept concept;
	concept.m_concept = CORBA::string_dup("room");
	addFeatureToCurrentProxy(concept);
	m_proxyIDs.insert(storeCurrentProxy());
	bindNewProxies();
      }
      awaitBinding(m_proxyIDs);
      sleepProcess(2000);
      {
	BindingFeatures::Concept concept;
	string q;
	concept.m_concept = CORBA::string_dup("kitchen");      
	QueryUtils utils(dynamic_cast<cast::PrivilegedManagedProcess&>(*this),
			 bindingSubarchID());
	
	q = utils.askBasicQuery(concept,
				kitchen_id);
	m_basicQueryIDs.push_back(q);//0
	
	concept.m_concept = CORBA::string_dup("kitchen");      
	q = utils.askAdvancedQuery(concept);
	m_advancedQueryIDs.push_back(q); //0
	
	m_testFinished = true;
      }
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
  sleepProcess(1000);
  addToWorkingMemory(newDataID(), getBindingSA(), new BindingData::TriggerDotViewer, cast::cdl::BLOCKING);  //testCompleteness();
}
  
void 
ComaTester::bindingProxyAdded(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  //const LBindingProxy& binding_proxy(m_proxyLocalCache[id]);
  m_addSignalledProxyIDs.insert(id);
}

void 
ComaTester::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.m_address.m_id);
  m_overwriteSignalledProxyIDs.insert(id);
  try {
    m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
    testCompleteness();
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComaTester::bindingProxyUpdated: " + string(_e.what()));
    abort();
  }
}

void 
ComaTester::bindingUnionUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
  try {
    testCompleteness();
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComaTester::bindingUnionUpdated: " + string(_e.what()));
    abort();
  }
}

void 
ComaTester::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  if(m_statusID.empty())
    m_statusID = _wmc.m_address.m_id;
  else
    assert(m_statusID == string(_wmc.m_address.m_id));
  m_status = *loadBindingDataFromWM<BindingData::BinderStatus>(getBindingSA(), m_statusID);  
  try {
    testCompleteness();    
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComaTester::statusUpdated: " + string(_e.what()));
    abort();
  }
}
  
void 
ComaTester::testCompleteness() {
  //log("ComaTester::testCompleteness()");
  // make sure we're loading fresh data
  m_handler.purgeAllLoadedData();
  switch(m_test) {
  case 0: 
    {
      assert(m_addSignalledProxyIDs.size() <= 2);
      if(m_testFinished && allBound() && 
	 m_advancedQueryAnswers.size() == m_advancedQueryIDs.size() &&
	 m_basicQueryAnswers.size() == m_basicQueryIDs.size()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 1);
	assert(true_for_all(unions, 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("thing")) && 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("ball"))
			    )
	       );
	{
	  map<string,BindingQueries::BasicQuery>::const_iterator itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[0]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    mug_answer(itr->second);
	  assert(mug_answer.m_answer == cdl::triFalse);
	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[1]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    mug_answer2(itr->second);
	  assert(mug_answer2.m_answer == cdl::triFalse);
	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[2]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    ball_answer(itr->second);
	  assert(ball_answer.m_answer == cdl::triTrue);
	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[3]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    ball_answer2(itr->second);
	  assert(ball_answer2.m_answer == cdl::triTrue);
	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[4]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    thing_answer(itr->second);
	  assert(thing_answer.m_answer == cdl::triTrue);

	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[5]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    concept_answer(itr->second);
	  assert(concept_answer.m_answer == cdl::triTrue);

	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[6]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    colour_answer(itr->second);
	  assert(colour_answer.m_answer == cdl::triTrue);

	  itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[7]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    colour_answer2(itr->second);
	  assert(colour_answer2.m_answer == cdl::triFalse);
	}

	map<string,BindingQueries::AdvancedQuery>::const_iterator itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[0]);
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  mug_answer(itr->second);
	assert(mug_answer.m_processed);
	assert(mug_answer.m_hasTheFeatureProxyIDs.length() == 2);
	assert(mug_answer.m_hasTheFeatureUnionIDs.length() == 1);
	assert(mug_answer.m_matchingProxyIDs.length() == 0);
	assert(mug_answer.m_matchingUnionIDs.length() == 0);
	assert(mug_answer.m_nonMatchingProxyIDs.length() == 2);
	assert(mug_answer.m_nonMatchingUnionIDs.length() == 1);
	ProxySet nonmugs = m_handler.loadProxies(mug_answer.m_nonMatchingProxyIDs);
	assert(true_for_all(nonmugs,
			    featureCheck<ProxyPtr,BindingFeatures::Concept>
			    (ConceptRegexMatcher("thing")) ||
			    featureCheck<ProxyPtr,BindingFeatures::Concept>
			    (ConceptRegexMatcher("ball")))
	       );	
	itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[1]);
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  mug_answer2(itr->second); // answer excludes union features
	assert(mug_answer2.m_hasTheFeatureProxyIDs.length() == 2);
	assert(mug_answer2.m_hasTheFeatureUnionIDs.length() == 1);
	assert(mug_answer2.m_matchingProxyIDs.length() == 1); // matches 'thing'
	assert(mug_answer2.m_matchingUnionIDs.length() == 1);
	assert(mug_answer2.m_nonMatchingProxyIDs.length() == 1);
	assert(mug_answer2.m_nonMatchingUnionIDs.length() == 1);
	nonmugs = m_handler.loadProxies(mug_answer2.m_nonMatchingProxyIDs);
	assert(true_for_all(nonmugs,
			    featureCheck<ProxyPtr,BindingFeatures::Concept>
			    (ConceptRegexMatcher("ball")))
	       );
	// the thing proxy does match mug, and thereby has a colour
	ProxySet mugs(m_handler.loadProxies(mug_answer2.m_matchingProxyIDs));
	assert(true_for_all(mugs, hasFeature<ProxyPtr,BindingFeatures::Colour>()));
	itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[2]);
	ProxySet comparable_to_area_id = m_handler.allProxiesFromWM();
	comparable_to_area_id = comparable_to_area_id | 
	  ComparableToFeature<ProxyPtr>(cast::typeName<BindingFeatures::AreaID>());
	assert(comparable_to_area_id.size() == 4); // i.e all proxies
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  thing_answer(itr->second); 
	assert(thing_answer.m_hasTheFeatureProxyIDs.length() == 2);
	assert(thing_answer.m_hasTheFeatureUnionIDs.length() == 1);
	assert(thing_answer.m_matchingProxyIDs.length() == 2);
	assert(thing_answer.m_matchingUnionIDs.length() == 1);
	assert(thing_answer.m_nonMatchingProxyIDs.length() == 0);
	assert(thing_answer.m_nonMatchingUnionIDs.length() == 0);

	itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[3]);
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  ball_answer(itr->second);
	assert(ball_answer.m_hasTheFeatureProxyIDs.length() == 2);
	assert(ball_answer.m_hasTheFeatureUnionIDs.length() == 1);
	assert(ball_answer.m_matchingProxyIDs.length() == 2); 
	assert(ball_answer.m_matchingUnionIDs.length() == 1);
	assert(ball_answer.m_nonMatchingProxyIDs.length() == 0);
	assert(ball_answer.m_nonMatchingUnionIDs.length() == 0);
	
	itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[4]);
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  colour_answer(itr->second); 
	assert(colour_answer.m_hasTheFeatureProxyIDs.length() == 2);
	assert(colour_answer.m_hasTheFeatureUnionIDs.length() == 1);
	assert(colour_answer.m_matchingProxyIDs.length() == 0); 
	assert(colour_answer.m_matchingUnionIDs.length() == 0);
	assert(colour_answer.m_nonMatchingProxyIDs.length() == 0);
	assert(colour_answer.m_nonMatchingUnionIDs.length() == 0);

	itr = 
	  m_advancedQueryAnswers.find(m_advancedQueryIDs[5]);
	assert(itr !=  m_advancedQueryAnswers.end());
	const BindingQueries::AdvancedQuery& 
	  colour_answer2(itr->second); // answer excludes union features
	assert(colour_answer2.m_hasTheFeatureProxyIDs.length() == 1);
	ProxySet has_colour = 
	  m_handler.loadProxies(colour_answer2.m_hasTheFeatureProxyIDs);
	assert(true_for_all(has_colour,hasFeature<ProxyPtr,BindingFeatures::Colour>()));
	assert(colour_answer2.m_hasTheFeatureUnionIDs.length() == 1);
	assert(colour_answer2.m_matchingProxyIDs.length() == 0); 
	assert(colour_answer2.m_matchingUnionIDs.length() == 0);
	assert(colour_answer2.m_nonMatchingProxyIDs.length() == 0);
	assert(colour_answer2.m_nonMatchingUnionIDs.length() == 0);

	successExit();
      }
    }
    break;  
  case 1: 
    {
      assert(m_addSignalledProxyIDs.size() <= 6);
      if(m_testFinished && allBound()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 1);
	assert(true_for_all(unions, 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("sofa")) && 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("Sofa")) && 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("couch")) &&
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("Couch")) && 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("object")) && 
			    featureCheck<UnionPtr,BindingFeatures::Concept>(ConceptRegexMatcher("Object"))
			    )
	       );
	successExit();
      }
    }
    break;  
  case 2: 
    {
      assert(m_addSignalledProxyIDs.size() <= 2);
      if(m_testFinished && allBound()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 2);
	successExit();
      }
    }
    break;  
  case 3: 
    {
      assert(m_addSignalledProxyIDs.size() <= 2);
      if(m_testFinished && allBound()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 1);
	successExit();
      }
    }
    break;
  case 4: 
    {
      assert(m_addSignalledProxyIDs.size() <= 2);
      if(m_testFinished && allBound()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 2);
	successExit();
      }
    }
    break;
  case 5: 
    {
      assert(m_addSignalledProxyIDs.size() <= 2);
      if(m_testFinished && allBound()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 1);
	successExit();
      }
    }
    break;
  case 6: 
    {
      assert(m_addSignalledProxyIDs.size() <= 4);
      if(m_testFinished && allBound() && 
	 m_advancedQueryAnswers.size() == m_advancedQueryIDs.size() &&
	 m_basicQueryAnswers.size() == m_basicQueryIDs.size()) {
	ProxySet proxies = m_handler.loadProxies(m_overwriteSignalledProxyIDs);
	UnionSet unions = m_handler.extractUnionsFromProxies(proxies);
	assert(unions.size() == 3);
	{
	  map<string,BindingQueries::BasicQuery>::const_iterator itr = 
	    m_basicQueryAnswers.find(m_basicQueryIDs[0]);
	  assert(itr !=  m_basicQueryAnswers.end());
	  const BindingQueries::BasicQuery& 
	    kitchen_answer(itr->second);
	  assert(kitchen_answer.m_answer == cdl::triTrue);
	}
	{	
	  map<string,BindingQueries::AdvancedQuery>::const_iterator itr = 
	    m_advancedQueryAnswers.find(m_advancedQueryIDs[0]);
	  assert(itr !=  m_advancedQueryAnswers.end());
	  const BindingQueries::AdvancedQuery& 
	    kitchen_answer(itr->second);
	  assert(kitchen_answer.m_processed);
	  assert(kitchen_answer.m_hasTheFeatureProxyIDs.length() == 2);
	  assert(kitchen_answer.m_hasTheFeatureUnionIDs.length() == 2);
	  assert(kitchen_answer.m_matchingProxyIDs.length() == 3);
	  assert(kitchen_answer.m_matchingUnionIDs.length() == 2);
	  assert(kitchen_answer.m_nonMatchingProxyIDs.length() == 1);
	  assert(kitchen_answer.m_nonMatchingUnionIDs.length() == 1);
	  ProxySet nonkitchens = m_handler.loadProxies(kitchen_answer.m_nonMatchingProxyIDs);
	  assert(true_for_all(nonkitchens,
			      featureCheck<ProxyPtr,BindingFeatures::Concept>
			      (ConceptRegexMatcher("office")))
		 );
	}
	successExit();
      }
    }
    break;
  default:
    log("incorrect test number");
    failExit();
  }  
}

} // namespace Binding
