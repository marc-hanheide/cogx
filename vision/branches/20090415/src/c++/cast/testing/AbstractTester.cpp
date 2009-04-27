/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Michael Zillich, Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "AbstractTester.hpp"


#include <errno.h>

using namespace std;
using namespace boost;



namespace cast {
  
  AbstractTester::AbstractTester()
    :  m_exitOnCompletion(true),
       m_successValue(cdl::testing::CASTTESTPASS),
       m_failureValue(cdl::testing::CASTTESTFAIL) {
    
  }

  AbstractTester::~AbstractTester() {

    // do it in body
//     //wait for test to finish
//     if(pthread_join(m_testThread, NULL) != 0) {
//       throw CASTException(__HERE__, "%s failed to join test thread: %s",
// 			  getComponentID().c_str(),
// 			  strerror(errno));
//     }
  }

  void AbstractTester::configure(const std::map<std::string,std::string> & _config) {
    ManagedComponent::configure(_config);

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
      tokenizeString(testString, tests, ",");

      for(vector<string>::iterator i = tests.begin(); 
	  i < tests.end();
	  ++i) {
	queueTest(*i);
      }

    }

  }

  void * runTest( void * _ptr ) {

    //cout<<"runTest"<<endl;
    AbstractTester::AbstractTest * pTest = (AbstractTester::AbstractTest *) _ptr;
    pTest->run();
    return 0;
  }

  bool AbstractTester::performTest(const shared_ptr<AbstractTest> & _test) {

    assert(_test.get() != NULL);
    
    if(pthread_create(&m_testThread, NULL, runTest, _test.get()) != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create test thread: %s",
					   strerror(errno)));
    }

    //give the test time to init
    sleep(1);
    //cout<<"waiting for completion"<<endl;

    // wait for the test fo complete
    bool passed = _test->waitFor();


    //then make sure the thread is finished
    if(pthread_join(m_testThread, NULL) != 0) {
      throw CASTException(exceptionMessage(__HERE__, "%s failed to join test thread: %s",
			  getComponentID().c_str(),
					   strerror(errno)));
    }



    // println("complete");
    return passed;
  }

  bool AbstractTester::runTests() {

    bool passed = true;

    while (!m_perform.empty()) {

      // get next test to perform
      string nextTestID = m_perform.front();
      //and remove it
      m_perform.pop();

      // retrieve test and remove from map
      shared_ptr<AbstractTest> nextTest = m_tests[nextTestID];

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

  void AbstractTester::runComponent() {

    // a bunch of things that may go wrong
    assert(getComponentID() != "");
    assert(m_subarchitectureID != ""); 
    assert(!m_tests.empty());

    // if no tests are explicitly queued, perform all registered tests
    if (m_perform.empty()) {
      log("no tests queued, performing all registered tests instead");
      //m_perform = new LinkedList<String>(m_tests.keySet());
    }

    bool allPassed = runTests();



    if (m_exitOnCompletion) {
      //HACK let things settle down
      sleepComponent(5000);    
      if (allPassed) {
	::exit(m_successValue);
      } else {
	::exit(m_failureValue);
      }
    }

  }



  void AbstractTester::registerTest(string _id, shared_ptr<AbstractTest> _test) throw(CASTException) {
    assert(_id.length() > 0);
    assert(_test);
    
    //check whether that test actually exists before adding it
    TestMap::iterator testIt = m_tests.find(_id);
    if (testIt != m_tests.end()) {
      throw CASTException(exceptionMessage(__HERE__,"test already exists: %s", _id.c_str()));
    } else {
      m_tests[_id] = _test;
    }
  }

  void AbstractTester::queueTest(string _id) throw(CASTException) {
    assert(_id.length() > 0);

    //check whether that test actually exists before adding it
    TestMap::iterator testIt = m_tests.find(_id);
    if (testIt != m_tests.end()) {
      log("queued test: %s", _id.c_str());
      m_perform.push(_id);
    } 
    else {
      throw CASTException(exceptionMessage(__HERE__,"unknown test to queue: %s",_id.c_str()));
    }
  }

  AbstractTester::AbstractTest::AbstractTest(AbstractTester & _tester) : 
    m_tester(_tester) {
    
    pthread_mutexattr_t attr;
    // note: errors here are very unlikely, so we just do a "weaker" overall
    // checking
    int err =  pthread_mutexattr_init(&attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }


#ifdef __LINUX__    
    //nah: this doesn't work, and isn't necessary, on macs for some reason
    err = pthread_mutexattr_setpshared(&attr, PTHREAD_COMPONENT_SHARED);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }
#endif

    err = pthread_mutex_init(&m_completionLock, &attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }

    err = pthread_mutexattr_destroy(&attr);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed to create mutex: %s", strerror(err)));
    }
  }

  AbstractTester::AbstractTest::~AbstractTest() {
    
  }

  void AbstractTester::AbstractTest::testComplete(bool _passed) {
    m_testComplete = true;
    m_passed = _passed;
    int err = pthread_mutex_unlock(&m_completionLock); 
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex unlock: %s", strerror(err)));
    }

  }

  void AbstractTester::AbstractTest::run() {
    m_testComplete = false;
    m_passed = false;
    //lock mutex for wait
    int err = pthread_mutex_lock(&m_completionLock);
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex lock: %s", strerror(err)));
    }

    m_tester.lockComponent();
    startTest();
    m_tester.unlockComponent();
  }

  bool AbstractTester::AbstractTest::waitFor() {
    int err = pthread_mutex_lock(&m_completionLock); 
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex lock: %s", strerror(err)));
    }

    err = pthread_mutex_unlock(&m_completionLock); 
    if(err != 0) {
      throw CASTException(exceptionMessage(__HERE__, "failed mutex unlock: %s", strerror(err)));
    }


    //m_tester.println("end of waitFor");
    return m_passed;
  }
   


} //namespace cast

