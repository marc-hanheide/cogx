#include "AbstractVisionTester.hpp"

#include <errno.h>

using namespace std;
using namespace boost;
using namespace Vision;

namespace cast {

AbstractVisionTester::AbstractVisionTester(const string &_id)
    :  WorkingMemoryAttachedComponent(_id),  
       PrivilegedManagedProcess(_id),
       m_exitOnCompletion(true),
       m_successValue(cdl::testing::CAST_TEST_PASS),
       m_failureValue(cdl::testing::CAST_TEST_FAIL) 
{
    cout << "AbstractVisionTester::AbstractVisionTester\n";
}

AbstractVisionTester::~AbstractVisionTester() {
   
}

void AbstractVisionTester::configure(std::map<std::string,std::string> & _config) {
    PrivilegedManagedProcess::configure(_config);
    
    map<string,string>::const_iterator i = _config.find("--exit"); 
    if(i != _config.end()) {
      if(i->second == "false") {
	  m_exitOnCompletion = false;
      }
      else {
	  m_exitOnCompletion = true;
      }
      log("m_exitOnCompletion = %d", m_exitOnCompletion);
    }
    
    
    i = _config.find("--test"); 
    if(i != _config.end()) {
	string testString = i->second;
      vector<string> tests; 
      SubarchitectureWorkingMemoryProtocol::tokenizeString(testString, tests, ",");
      
      for(vector<string>::iterator i = tests.begin(); i < tests.end(); ++i) {
	  queueTest(*i);
      }
      
    }
    
}

void * runTest( void * _ptr ) 
{
    AbstractVisionTester::AbstractVisionTest * pTest = (AbstractVisionTester::AbstractVisionTest *) _ptr;
    pTest->run();
    return 0;
  }

bool AbstractVisionTester::performTest(const shared_ptr<AbstractVisionTest> & _test) {

    assert(_test.get() != NULL);
    
    if(pthread_create(&m_testThread, NULL, runTest, _test.get()) != 0) {
      throw CASTException(__HERE__, "failed to create test thread: %s",
			  strerror(errno));
    }
    
    //give the test time to init
    sleep(1);
    //cout<<"waiting for completion"<<endl;
    
    // wait for the test fo complete
    bool passed = _test->waitFor();
    
    
    //then make sure the thread is finished
    if(pthread_join(m_testThread, NULL) != 0) {
	throw CASTException(__HERE__, "%s failed to join test thread: %s",
			    getProcessIdentifier().c_str(),
			    strerror(errno));
    }
    

    
    // println("complete");
    return passed;
}

bool AbstractVisionTester::runTests() {
    
    bool passed = true;
    
    while (!m_perform.empty()) {
	
	// get next test to perform
	string nextTestID = m_perform.front();
	//and remove it
	m_perform.pop();
	
	// retrieve test and remove from map
	shared_ptr<AbstractVisionTest> nextTest = m_tests[nextTestID];
	
	string output;
	if (performTest(nextTest)) {
	    output = "passed";
	} else {
	    output = "failed";
	    passed = false;
	}
	log("%s: %s",nextTestID.c_str(), output.c_str());
    }
    return passed;
}

void AbstractVisionTester::runComponent() 
{   
    // allow time for other components to start, sleep one second
    //sleep(1);

    // a bunch of things that may go wrong
    assert(getProcessIdentifier() != "");
    assert(m_subarchitectureID != ""); 
    assert(!m_tests.empty());
    
    // if no tests are explicitly queued, perform all registered tests
    if (m_perform.empty()) {
	log("no tests queued, performing all registered tests instead");
	//m_perform = new LinkedList<String>(m_tests.keySet());
    }
    
    bool allPassed = runTests();
    
    
    //HACK let things settle down
    sleep(1);
    
    if (m_exitOnCompletion) {
	if (allPassed) {
	    ::exit(m_successValue);
	} else {
	    ::exit(m_failureValue);
	}
    }
    
}



void AbstractVisionTester::registerTest(string _id, shared_ptr<AbstractVisionTest> _test) 
    throw(CASTException) 
{
    assert(_id.length() > 0);
    assert(_test);
    
    //check whether that test actually exists before adding it
    TestMap::iterator testIt = m_tests.find(_id);
    if (testIt != m_tests.end()) {
	throw CASTException(__HERE__,"test already exists: %s", _id.c_str());
    } else {
      m_tests[_id] = _test;
    }
}

void AbstractVisionTester::queueTest(string _id) throw(CASTException) 
{
    assert(_id.length() > 0);
    
    //check whether that test actually exists before adding it
    TestMap::iterator testIt = m_tests.find(_id);
    if (testIt != m_tests.end()) {
	log("queued test: %s", _id.c_str());
	m_perform.push(_id);
    } 
    else {
	throw CASTException(__HERE__,"unknown test to queue: %s",_id.c_str());
    }
}


void AbstractVisionTester::setPullConnector(const string &_connectionID,
					    PullConnectorOut<ImageFrame> *pc)
{
    cout << "AbstractVisionTester::setPullConnector\n";
    img_pull = pc;
}


Vision::ImageFrame* AbstractVisionTester::GetImage(int camNum)
{
    cout << "AbstractVisionTester::GetImage\n";
    if(img_pull)
    {
	char reference[100];
	
	snprintf(reference, 100, "%d", camNum);
	FrameworkQuery query(getProcessIdentifier(), "");
	query.setQuery(string(reference));
	
	FrameworkLocalData<ImageFrame> *img_local_data = 0;
	img_pull->pull(query, img_local_data);
	if(img_local_data)
	{
	    ImageFrame *img = img_local_data->data();
	    // protect our data
	    img_local_data->data() = NULL;
	    // clean up rest
	    delete img_local_data;
	    return img;
	}
	else {
	    throw BALTException(__HERE__, "image pull failed");
	}
    }
    else {
	throw BALTException(__HERE__, "pull connector for image not set");
    }
}







//////////////////////////////////
//
//    make common cast calls available 
//
/////////////////////////////////
AbstractVisionTester::AbstractVisionTest::AbstractVisionTest(AbstractVisionTester & _tester) : 
    m_tester(_tester) {
    
    pthread_mutexattr_t attr;
    // note: errors here are very unlikely, so we just do a "weaker" overall
    // checking
    int err =  pthread_mutexattr_init(&attr);
    if(err != 0) {
	throw CASTException(__HERE__, "failed to create mutex: %s", strerror(err));
    }
    
    
#ifdef __LINUX__    
    //nah: this doesn't work, and isn't necessary, on macs for some reason
    err = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if(err != 0) {
      throw CASTException(__HERE__, "failed to create mutex: %s", strerror(err));
    }
#endif
    
    err = pthread_mutex_init(&m_completionLock, &attr);
    if(err != 0) {
      throw CASTException(__HERE__, "failed to create mutex: %s", strerror(err));
    }
    
    err = pthread_mutexattr_destroy(&attr);
    if(err != 0) {
	throw CASTException(__HERE__, "failed to create mutex: %s", strerror(err));
    }
}

AbstractVisionTester::AbstractVisionTest::~AbstractVisionTest() {
    
}

void AbstractVisionTester::AbstractVisionTest::testComplete(bool _passed) {
    m_testComplete = true;
    m_passed = _passed;
    int err = pthread_mutex_unlock(&m_completionLock); 
    if(err != 0) {
      throw CASTException(__HERE__, "failed mutex unlock: %s", strerror(err));
    }
    
}

void AbstractVisionTester::AbstractVisionTest::run() 
{
    m_testComplete = false;
    m_passed = false;
    //lock mutex for wait
    int err = pthread_mutex_lock(&m_completionLock);
    if(err != 0) {
	throw CASTException(__HERE__, "failed mutex lock: %s", strerror(err));
    }
    
    startTest();
  }

bool AbstractVisionTester::AbstractVisionTest::waitFor() {
    int err = pthread_mutex_lock(&m_completionLock); 
    if(err != 0) {
	throw CASTException(__HERE__, "failed mutex lock: %s", strerror(err));
    }
    
    err = pthread_mutex_unlock(&m_completionLock); 
    if(err != 0) {
	throw CASTException(__HERE__, "failed mutex unlock: %s", strerror(err));
    }

    
    m_tester.println("end of waitFor");
    return m_passed;
}





}
