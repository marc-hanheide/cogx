
#include "DiabolicTester.hpp"
#include <boost/lexical_cast.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/lexical_cast.hpp>

namespace cast {
  namespace testing {

    using namespace boost;
    using namespace std;
    using namespace cast; 
    using namespace cast::cdl;
    using namespace cast::cdl::testing;

    DiabolicTester::DiabolicTester(const string &_id) :
      WorkingMemoryAttachedComponent(_id),
      PrivilegedManagedProcess(_id),
      m_locking(false)

    {
      m_queueBehaviour = cdl::QUEUE;
    }

    void
    DiabolicTester::start() {    
      PrivilegedManagedProcess::start();
  
      if(m_mode == READER) {
	addChangeFilter(createLocalTypeFilter<TestDummyStruct>(cdl::ADD),
			new MemberFunctionChangeReceiver<DiabolicTester>(this,
									 &DiabolicTester::dummyAdded));
	addChangeFilter(createLocalTypeFilter<TestDummyStruct>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<DiabolicTester>(this,
									 &DiabolicTester::dummyOverwritten));  
      }
    
    }

    void 
    DiabolicTester::configure(map<string,string> & _config) {
      PrivilegedManagedProcess::configure(_config);
      m_mode = READER;
      map<string,string>::const_iterator itr = _config.find("--writer");
      if(itr != _config.end()) {
	m_mode = WRITER;
	log("I'm a writer");
      } else {
	log("I'm a reader");
      }
  
      itr = _config.find("-n");
      if(itr != _config.end()) {
	m_n = lexical_cast<unsigned int>(itr->second);
      }
      itr = _config.find("--locking");
      if(itr != _config.end()) {
	m_locking = true;
      }
    }

    string
    random_string(unsigned int _max_length)
    {
      return string(rand() % _max_length, 'X');
    }

    void DiabolicTester::runComponent() {
      sleepProcess(1000); // sleep for a second to allow the rest to be properly started
      if(m_mode == WRITER) {
	m_dummyID = newDataID();
	TestDummyStruct* dummy = new TestDummyStruct;
	dummy->m_dummy = CORBA::string_dup((string("first dummy string ") + random_string(500)).c_str());
	addToWorkingMemory(m_dummyID,
			   dummy,
			   BLOCKING);
	if(m_locking)
	  tryLockEntry(m_dummyID,LOCKED_O);
	log("added");
	for(unsigned int i = 0 ; i < m_n ; ++i) {
	  // random sleep to add some suspense
	  sleepProcess(rand()%10);
	  dummy = new TestDummyStruct;
	  dummy->m_dummy = CORBA::string_dup(string("dummy string #" + lexical_cast<string>(i) + " " + random_string(500)).c_str());
	  log("overwriting");
	  if(m_locking)
	    overwriteWorkingMemory(m_dummyID,
				   dummy);
	  else 
	    overwriteWorkingMemory(m_dummyID,
				   dummy,
				   BLOCKING);

	}
	sleepProcess(1000);
	successExit();
      }
    }

    void
    DiabolicTester::dummyAdded(const cast::cdl::WorkingMemoryChange & _wmc)
    {
      assert(!m_dummyCache.get());
      m_dummyCache = auto_ptr<CachedCASTData<TestDummyStruct> >(new CachedCASTData<TestDummyStruct>(*this,string(_wmc.m_address.m_id)));
  
    }

    void
    DiabolicTester::dummyOverwritten(const cast::cdl::WorkingMemoryChange & _wmc) {
      // now, if the length of the string is odd, then keep reloading until the next update
      log("overwritten");
      while((string((*m_dummyCache)->m_dummy).size() + m_n) % 2) {
	sleepProcess(rand()%10);
	log("reloaded");
      }
    }
  

  } // namespace testing 
} // namespace cast



/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const std::string &_id) {
    return new cast::testing::DiabolicTester(_id);
  }
}

