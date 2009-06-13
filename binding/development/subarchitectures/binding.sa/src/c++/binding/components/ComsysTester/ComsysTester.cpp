
#include "ComsysTester.hpp"
#include <boost/lexical_cast.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

namespace Binding {
  using namespace boost;
  using namespace std;
  using namespace cast;
  using namespace org::cognitivesystems::comsys::autogen::ComsysEssentials; // wohaa!

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Binding::ComsysTester(_id);
  }
}


ComsysTester::ComsysTester(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  AbstractBinder(_id),
  test(-1),
  handler(*this),
  testFinished(true),
  retest(0)
{
  queueBehaviour = cdl::QUEUE;
}

void
ComsysTester::start() {    
  AbstractBinder::start();
  
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::bindingProxyUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::bindingProxyUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::BindingProxy>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::bindingProxyDeleted));
  addChangeFilter(createLocalTypeFilter<BindingData::BinderStatus>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::statusUpdated));
  addChangeFilter(createLocalTypeFilter<BindingData::BinderStatus>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::statusUpdated));

  addChangeFilter(createGlobalTypeFilter<BindingData::TriggerDotViewer>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::somethingInterestingUpdated));
  addChangeFilter(createGlobalTypeFilter<BindingData::TriggerDotViewerWithTitle>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ComsysTester>(this,
								  &ComsysTester::somethingInterestingUpdated));

  
}

void 
ComsysTester::configure(map<string,string> & _config) {
  AbstractBinder::configure(_config);
  map<string,string>::const_iterator itr = _config.find("--test");
  if(itr == _config.end()) {
    cerr << "Error in ComsysTester: --test N must be given";
    abort();
  }
  test = lexical_cast<int>(itr->second);
  assert(test >= 0);
  itr = _config.find("--comsys");
  if(itr == _config.end()) {
    cerr << "Error in ComsysTester: --comsys ID must be given, where ID is the subarchitecture ID of comsys";
    abort();
  }
  comsysID = itr->second;
}


void 
ComsysTester::bindingProxyUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.address.id);
  addSignalledProxyIDs.insert(id);
  proxiesOnWM.insert(id);
  try {
    status = *loadBindingDataFromWM<BindingData::BinderStatus>(statusID);  
    testCompleteness();
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComsysTester::bindingProxyUpdated: " + string(_e.what()));
    abort();
  }
}

void 
ComsysTester::bindingProxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc) {
  string id(_wmc.address.id);
  
  std::set<std::string>::iterator itr = proxiesOnWM.find(id);
  assert(itr != proxiesOnWM.end());
  proxiesOnWM.erase(itr);
  
  try {
    status = *loadBindingDataFromWM<BindingData::BinderStatus>(statusID);  
    testCompleteness();
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComsysTester::bindingProxyUpdated: " + string(_e.what()));
    abort();
  }
}

void 
ComsysTester::statusUpdated(const cast::cdl::WorkingMemoryChange & _wmc) {
  status = *loadBindingDataFromWM<BindingData::BinderStatus>(_wmc);
  statusID = string(_wmc.address.id);
  try {
    testCompleteness();    
  } catch(const DoesNotExistOnWMException& _e) {
    log("Caught this in ComsysTester: " + string(_e.what()));
    abort();
  }
}

void ComsysTester::runComponent() {
  sleepProcess(3000); // sleep for a second to allow the rest to be properly started
  switch(test) {
  case 0: 
    {
      log("testing a single simple sentence");
      testFinished = false;
      passToComsys("the red triangle");
      sleepProcess(2000);
      testFinished = true;
    }
    break;
  case 1: 
    {
      log("testing two simple sentences with a delay between");
      testFinished = false;
      passToComsys("the red triangle");
      sleepProcess(2000);
      passToComsys("the green thing");
      testFinished = true;
    }
    break;
  case 2: 
    {
      log("testing two simple sentences with no delay between");
      testFinished = false;
      passToComsys("the red triangle");
      passToComsys("the green thing");      
      testFinished = true;
    }
    break;
  case 3: 
    {
      log("testing to add a number of simple sentences with a delay between each");
      log("will fail since all the excess updating before proxies are bound confuses the binder");
      testFinished = false;
      passToComsys("the red triangle");
      sleepProcess(1000);
      passToComsys("the blue circle");      
      sleepProcess(1000);
      passToComsys("the green square");      
      sleepProcess(1000);
      passToComsys("the yellow star");      
      sleepProcess(1000);
      passToComsys("the red thing");      
      sleepProcess(1000);
      passToComsys("the blue thing");      
      sleepProcess(1000);
      passToComsys("the green thing");      
      sleepProcess(1000);
      passToComsys("the yellow thing");       
      sleepProcess(1000);
      passToComsys("the triangle");      
      sleepProcess(1000);
      passToComsys("the circle");      
      sleepProcess(1000);
      passToComsys("the square");      
      sleepProcess(1000);
      passToComsys("the star");      
      testFinished = true;
    }
    break;
  case 4: 
    {
      log("testing to add a number of simple sentences with no delay between each");
      log("will fail since all the excess updating before proxies are bound confuses the binder");
      testFinished = false;
      passToComsys("the red triangle");
      passToComsys("the blue circle");      
      passToComsys("the green square");      
      passToComsys("the yellow star");      
      passToComsys("the red thing");      
      passToComsys("the blue thing");      
      passToComsys("the green thing");      
      passToComsys("the yellow thing");       
      passToComsys("the triangle");      
      passToComsys("the circle");      
      passToComsys("the square");      
      passToComsys("the star");      
      testFinished = true;
    }
    break;
  case 5: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("take the box");
      testFinished = true;
    }
    break;
  case 6: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("now take the box");
      testFinished = true;
    }
    break;
  case 7: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the ball to the left of the box");
      testFinished = true;
    }
    break;
  case 8: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the green ball to the left of the blue box");
      testFinished = true;
    }
    break;
  case 9: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the balls to the left of the box");
      testFinished = true;
    }
    break;
  case 10: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the green ball to the left of the boxes");
      testFinished = true;
    }
    break;
  case 11: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the green things to the left of the red thing");
      testFinished = true;
    }
    break;
  case 12: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the green things to the left of the red things");
      testFinished = true;
    }
    break;
  case 13: 
    {
      log("testing to add single (tricky) imperative sentence");
      testFinished = false;
      passToComsys("put the thing on the thing");
      testFinished = true;
    }
    break;
  case 14: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("put the green thing to the left of the mug onto the box");
      testFinished = true;
    }
    break;
  case 15: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("take the box and put it here");
      testFinished = true;
    }
    break;
  case 16: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("look at the table then take the green mug on it and give it to me");
      testFinished = true;
    }
    break;
  case 17: 
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("go to the kitchen");
      testFinished = true;
    }
    break;  
  case 18:
    {
      log("testing to add single imperative sentence");
      testFinished = false;
      passToComsys("go to the kitchen through the corridor and fetch me a cup of coffee");
      sleepProcess(200);
      passToComsys("and bring me a cookie too");
      testFinished = true;
    }
    break;
  case 19:
    {
      log("testing to add single simple sentence");
      testFinished = false;
      passToComsys("ok that is right");
      testFinished = true;
    }
    break;
  case 20:
    {
      log("testing to add single assertive sentence");
      testFinished = false;
      passToComsys("no that is wrong");
      testFinished = true;
    }
    break;
  case 21:
    {
      log("testing to add single assertive sentence");
      testFinished = false;
      passToComsys("there is a mug on the table");
      testFinished = true;
    }
    break;
  case 22:
    {
      log("testing to add single assertive sentence");
      testFinished = false;
      passToComsys("no this is a mug");
      testFinished = true;
    }
    break;
  case 23:
    {
      log("testing to add single question");
      testFinished = false;
      passToComsys("where is the box now");
      testFinished = true;
    }
    break;
  case 24:
    {
      log("testing to add single question");
      testFinished = false;
      passToComsys("are you there");
      testFinished = true;
    }
    break;  
  case 25:
    {
      log("testing to add single question");
      testFinished = false;
      passToComsys("what is it");
      testFinished = true;
    }
    break;  
  case 26:
    {
      log("testing to add an assertion, and a follow up assertion (delay between)");
      testFinished = false;
      passToComsys("here is the mug");
      sleepProcess(3000);
      passToComsys("it is green");
      testFinished = true;
    }
    break;
  case 27:
    {
      log("testing to add an assertion, and a follow up question (delay between)");
      testFinished = false;
      passToComsys("here is the mug");
      sleepProcess(3000);
      passToComsys("is it green");
      testFinished = true;
    }
    break;  
  case 28:
    {
      log("testing to add an assertion monologue (delays between)");
      testFinished = false;
      passToComsys("here is the mug");
      sleepProcess(3000);
      passToComsys("it is green");
      sleepProcess(3000);
      passToComsys("and there is a table");
      sleepProcess(3000);
      passToComsys("it is big and white");
      testFinished = true;
    }
    break;
  case 29:
    {
      log("testing to add an assertion monologue (delays between)");
      testFinished = false;
      passToComsys("here is the mug");
      sleepProcess(3000);
      passToComsys("it is green");
      sleepProcess(3000);
      passToComsys("right it is also round");
      testFinished = true;
    }
    break;
  case 30:
    {
      log("testing to refer to a passed event");
      testFinished = false;
      passToComsys("you picked up the green mug");
      testFinished = true;
    }
    break;
  case 31:
    {
      log("assert a passed fact");
      testFinished = false;
      passToComsys("the mug was in the kitchen yesterday");
      testFinished = true;
    }
    break;
  case 32:
    {
      log("testing to refer to a passed fact");
      testFinished = false;
      passToComsys("get the mug that was in the kitchen yesterday");
      testFinished = true;
    }
    break;
  case 33:
    {
      log("refer to passed events in a query (directed to the episodic memory)");
      testFinished = false;
      passToComsys("what did you do this morning");
      testFinished = true;
    }
    break;
  case 34:
    {
      log("refer to passed events in a query (directed to the episodic memory)");
      testFinished = false;
      passToComsys("from where did you get the mug");
      testFinished = true;
    }
    break;
  case 35:
    {
      log("refer to a future event");
      testFinished = false;
      passToComsys("when you get to the kitchen you should give the cup to hendrik");
      testFinished = true;
    }
    break;
  case 36:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("put the red thing to the left of the blue thing");
      testFinished = true;
    }
    break;
  case 37:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("put the red ball to the left of the blue objects");
      testFinished = true;
    }
    break;  
  case 38:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("move the ball to the left of the box");
      testFinished = true;
    }
    break;
  case 39:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("put the square near the red car");
      testFinished = true;
    }
    break;
  case 40:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is near the red ball");
      testFinished = true;
    }
    break;
  case 41:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is to the left of the red ball");
      testFinished = true;
    }
    break;
  case 42:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is round");
      testFinished = true;
    }
    break;
  case 43:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is red");
      testFinished = true;
    }
    break;
  case 44:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is big");
      testFinished = true;
    }
    break;
  case 45:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is reachable");
      testFinished = true;
    }
    break;
  case 46:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what is empty");
      testFinished = true;
    }
    break;
  case 47:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what colour is the ball");
      testFinished = true;
    }
    break;
  case 48:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("what size is the box");
      testFinished = true;
    }
    break;
  case 49:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is the red square to the left of the triangle");
      testFinished = true;
    }
    break;
  case 50:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is the red square left of the triangle");
      testFinished = true;
    }
    break;
  case 51:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is the red square near the triangle");
      testFinished = true;
    }
    break;
  case 52:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is the red thing a square");
      testFinished = true;
    }
    break;
  case 53:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is this room the kitchen");
      testFinished = true;
    }
    break;
  case 54:
    {
      log("testing a critical playmate string");
      testFinished = false;
      passToComsys("is the thing a square");
      testFinished = true;
    }
    break;
  default:
    cerr << "incorrect test number: " << test << endl;
    failExit();
  }
  sleepProcess(1000);
  trigger_dot();
  sleepProcess(1000);
  trigger_dot();
  sleepProcess(1000);
  trigger_dot();
  sleepProcess(1000);
  trigger_dot();
}
  
void 
ComsysTester::testCompleteness() {
  handler.purgeAllLoadedData();
  try {
    switch(test) {
    case 0: 
      {
	if(testFinished && allBound()) {
	  ProxySet proxies = handler.allProxiesFromWM();
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  sleepProcess(100);
	  if(!(true_for_all(proxies,ConsistencyCheck<ProxyPtr>(*this)) &&
	       true_for_all(unions,ConsistencyCheck<UnionPtr>(*this)))) {
	    testCompleteness();
	    return;
	  }
	  
	  assert(proxies.size() == 1);
	  assert(unions.size() == 1);
	  assert(true_for_all(proxies, 
			      hasFeature<ProxyPtr,BindingFeatures::Concept>() &&
			      featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("triangle")) &&
			      hasFeature<ProxyPtr,BindingFeatures::Colour>() &&
			      featureCheck<ProxyPtr,BindingFeatures::Colour>(ColourRegexMatcher("red"))
			      )
		 );
	  successExit();
	  
	}
      }
      break;
    case 1: // same as 2, but with no delay
    case 2: 
      {
	if(addSignalledProxyIDs.size() >= 2 && testFinished && allBound()) {
	  ProxySet proxies = handler.allProxiesFromWM();
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == 2);
	  assert(unions.size() == 2);
	  assert(true_for_all(proxies, 
			      featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("triangle")) ->*
			      featureCheck<ProxyPtr,BindingFeatures::Colour>(ColourRegexMatcher("red"))
			      )
		 );
	  assert(true_for_all(proxies, 
			      featureCheck<ProxyPtr,BindingFeatures::Concept>(ConceptRegexMatcher("thing")) ->*
			      featureCheck<ProxyPtr,BindingFeatures::Colour>(ColourRegexMatcher("green"))
			      )
		 );
	  
	  successExit();
	}
      }
      break;
    case 3: 
    case 4: 
      {
	if(addSignalledProxyIDs.size() >= 7 && testFinished && allBound()) {
	  ProxySet proxies = handler.allProxiesFromWM();
	  UnionSet unions = handler.extractUnionsFromProxies(proxies);
	  assert(proxies.size() == 7);
	  successExit();
	}
      }
      break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
    case 42:
    case 43:
    case 44:
    case 45:
    case 46:
    case 47:
    case 48:
    case 49:
    case 50:
    case 51:
    case 52:
    case 53:
    case 54:
      if(!proxiesOnWM.empty() && 
	 !addSignalledProxyIDs.empty() 
	 && testFinished 
	 && allBound()) {
	log("everything is bound, right now that's enough");
	successExit();
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
  retest = 0;
}


void 
ComsysTester::passToComsys(const string& _sentence) 
{
  log(string("Sending this to comsys: \"") + _sentence +"\"");
  PhonString* sentence = new PhonString();
  const string id = newDataID();
  sentence->id = CORBA::string_dup(id.c_str());
  sentence->wordSequence = CORBA::string_dup(_sentence.c_str());
  tokenizer<> tok(_sentence);
  int words = 0;
  for(tokenizer<>::const_iterator beg= tok.begin(); 
      beg!=tok.end();
      ++beg) {
    ++words;
  }
  sentence->length = words; // seems redundant
  addToWorkingMemoryHelper(id, comsysID, sentence, cast::cdl::BLOCKING);
  static int utterance_no(1);
  if(!dotTitle.empty())
    dotTitle += "\\n";
  dotTitle += "utterance" + boost::lexical_cast<string>(utterance_no) + ": \\\"" + _sentence + "\\\"";
}


bool
ComsysTester::allBound() 
{  
  bool result = false;
  if (status.stable &&
      !proxiesOnWM.empty()) {
    bool consistency_checked = false;
    while(!consistency_checked) {
      result = false;
      ProxySet proxies = handler.loadProxies(proxiesOnWM);
      if(true_for_all(proxies,ProxyStateChecker(BindingData::BOUND) && !ProxyUnionIDChecker(""))) { // return true after some assertions
	// some consistency checks:
	assert(true_for_all(proxies,
			    hasFeature<ProxyPtr,BindingFeatures::SourceID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::ThisProxyID>() &&
			    hasFeature<ProxyPtr,BindingFeatures::CreationTime>() &&
			    (TypeCheck<ProxyPtr>(BindingData::GROUP) == hasFeature<ProxyPtr,BindingFeatures::Group>())
			    )
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

void 
ComsysTester::somethingInterestingUpdated(const cast::cdl::WorkingMemoryChange &) {
  testCompleteness();
}

  
} // namespace Binding

