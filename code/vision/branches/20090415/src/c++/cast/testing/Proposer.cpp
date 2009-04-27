#include "Proposer.hpp"
#include "TestTaskManager.hpp"


using namespace std;
using namespace boost;
using namespace cast::cdl;
using namespace cast::cdl::testing;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new cast::Proposer(_id);
  }
}

namespace cast {

  Proposer::Proposer(const string &_id) : 
    WorkingMemoryAttachedComponent(_id),
    ManagedProcess(_id),
    m_count(0),
    m_expected(0),
    m_response(ADOPTED) {

  }

  Proposer::~Proposer() {
  }

  void Proposer::configure(std::map<std::string,std::string> & _config) {

    ManagedProcess::configure(_config);

    map<string,string>::const_iterator i = _config.find("--count");
    if(i != _config.end()) {
      m_count = atoi(i->second.c_str());
    }

    i = _config.find("--response");
    if(i != _config.end()) {
      if(i->second == "ADOPTED") {
	m_response = ADOPTED;
      }
      else if(i->second == "REJECTED") {
	m_response = REJECTED;
      }
      else {
	m_response = MIXED;
      }      
    }

  }

  void Proposer::taskAdopted(const string &_taskID) {
    
    log("taskAdopted: %s", _taskID.c_str());

    if (m_response == ADOPTED
	|| m_response == MIXED) {

      int number = TestTaskManager::taskNumber(_taskID);

      if (number == m_expected) {
	m_expected++;

	randomSleep();

	// now say we're done
	  taskComplete(_taskID,
		       PROCESSING_COMPLETE_SUCCESS);

	if (m_expected == m_count) {
	  //sleep a little to let everyone else finis
	  sleepProcess(5000);
	  ::exit(CAST_TEST_PASS);
	}
      } else {
	::exit(CAST_TEST_FAIL);
      }

    } else {
      ::exit(CAST_TEST_FAIL);
    }
  }
 
  void Proposer::taskRejected(const string &_taskID) {
    if (m_response == REJECTED
	|| m_response == MIXED) {

      int number = TestTaskManager::taskNumber(_taskID);

      if (number == m_expected) {
	m_expected++;

	randomSleep();

	// now say we're done
	  taskComplete(_taskID,
		       PROCESSING_COMPLETE_SUCCESS);

	if (m_expected == m_count) {
	  //sleep a little to let everyone else finis
	  sleepProcess(5000);
	  ::exit(CAST_TEST_PASS);
	}
      } else {
	::exit(CAST_TEST_FAIL);
      }

    } else {
      ::exit(CAST_TEST_FAIL);
    }

  }

  void Proposer::runComponent() {
    for (int i = 0; i < m_count; i++) {
      lockProcess();
      proposeInformationProcessingTask(TestTaskManager::
				       numberedTask(getProcessIdentifier(), i), 
				       TestTaskManager::TEST_TASK);
      randomSleep();      
      unlockProcess();

      //      log("task proposed: %d", i);
    }   
  }

  void Proposer::randomSleep() {
    sleepProcess(rand() % 500);
  }
  

} //namespace cast
