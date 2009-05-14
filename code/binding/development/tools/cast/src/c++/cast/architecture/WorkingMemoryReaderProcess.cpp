/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
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

#include "WorkingMemoryReaderProcess.hpp"

#include <sstream>

using namespace std;
using namespace boost;

namespace cast {

  using namespace cdl;

  WorkingMemoryChangeThread::WorkingMemoryChangeThread(WorkingMemoryReaderProcess * _pWMRP) 
    : omni_thread(NULL, PRIORITY_HIGH) {
    
    m_bRun = false;
    m_pWMRP = _pWMRP;
    
    m_pChangeList = new list<cdl::WorkingMemoryChange>();
    
    //m_pChangeCondition = new omni_condition(&m_changeConditionMutex);
    
    m_pChangeSemaphore = new omni_semaphore(0);
    
  }


  WorkingMemoryChangeThread::~WorkingMemoryChangeThread() {
    m_pChangeList->clear();
  
    //   m_changeConditionMutex.lock();
    //   m_pChangeCondition->signal();
    //   m_changeConditionMutex.unlock();
  
    m_pChangeSemaphore->post();
  
  
  

    delete m_pChangeList;


    delete m_pChangeSemaphore;
  }


 
  void* WorkingMemoryChangeThread::run_undetached(void *arg) {

    //cout<<"home! "<<m_cid<<endl;
    if(cdl::DISCARD == m_pWMRP->m_queueBehaviour) {
      runDiscard();
    }
    else if(cdl::QUEUE == m_pWMRP->m_queueBehaviour) {
      runQueue();
    }

    //necessary return bits
    int* rv = new int(0);
    return (void*)rv;

  }
  

  void WorkingMemoryChangeThread::runQueue() {

    cdl::WorkingMemoryChangeList changeList;
    bool listEmpty = true;
    bool used = false;

    while(m_bRun) {
    
      //cout<<m_pWMRP->getProcessIdentifier()<<": "<<m_pChangeList->size()<<" change events "<<endl;

      m_queueAccess.lock();
      listEmpty = m_pChangeList->empty();
      m_queueAccess.unlock();

      while(!listEmpty) {

	//try to gain access
	////if(m_pWMRP->m_semaphore.trywait()) {
	m_pWMRP->lockProcess();
	//cout<<m_pWMRP->getProcessIdentifier()<<": "<<" RUN "<<endl;


	m_queueAccess.lock();
	
	changeList.length(m_pChangeList->size());
      
	for (unsigned int i = 0; i < changeList.length(); i++) {
	  //get last element
	  changeList[i] = m_pChangeList->back();
	  //and remove it
	  m_pChangeList->pop_back();
	}	
      
	if (m_pChangeList->size() != 0) {
	  m_pWMRP
	    ->println("error: emptied change list is not empty!");
	}
	
	m_queueAccess.unlock();

	
	try {
	  //remove any outstanding change filters before forwarding
	  //changes
	  removeChangeFilters();

	  //then write to derived class
	  used = forwardToSubclass(changeList);
	}
	catch(const BALTException & _e) {
	  ostringstream outStream;
	  outStream<<"Caught filter exception, have you removed a filter\n"
		   <<_e.what();
	  m_pWMRP->println(outStream.str());
	}


	//unlock this thread now something has been done
	m_pWMRP->unlockProcess();

	//now signal any threads that may have been waiting for new
	//changes
	if(used) {
	  m_pWMRP->m_wmcConditionMutex.lock();
	  m_pWMRP->m_pWMCCondition->broadcast();
	  m_pWMRP->m_wmcConditionMutex.unlock();
	}
	//cout<<m_pWMRP->getProcessIdentifier()<<": "<<" RUN DONE "<<endl;

	////}
	////else {
	//////cout<<m_pWMRP->getProcessIdentifier()<<": "<<" NOT "<<endl;
	////m_pWMRP->log("unable to access thread");

	////m_pWMRP->m_unlockNotificationMutex.lock();
	//////cout<<"inside mutex"<<endl;
	////while (m_bRun && !m_pWMRP->m_semaphore.trywait()) {
	//////cout<<"waiting on condition"<<endl;
	////m_pWMRP->m_pUnlockNotificationCondition->wait();
	//////cout<<"woken up"<<endl;
	////}	
	//////cout<<"unlocked"<<endl;
	////m_pWMRP->m_unlockNotificationMutex.unlock();

	//////must release thread after the trywait is successful above!
	////m_pWMRP->unlockProcess();


	////      }

	m_queueAccess.lock();
	listEmpty = m_pChangeList->empty();
	m_queueAccess.unlock();

      }
        
      //make sure someone else gets a turn...
      //yield(); -- doesn't work!


      m_pChangeSemaphore->wait();

      //     m_changeConditionMutex.lock();

      //     //check still running before waiting
      //     while (m_bRun && m_pChangeList->empty()) {
      //       //m_pWMRP->println(" wait.... ");
      //       //cout.flush();
      //       if(!m_pChangeList->empty()) {
      // 	cout<<"UHOUHOUHOUHO"<<endl;
      //       }
      //       m_pChangeCondition->wait();
      //       //m_pWMRP->println(" let's go! ");
      //     }

      //     m_changeConditionMutex.unlock();
    
    }

  }


  bool WorkingMemoryChangeThread::forwardToSubclass(const cdl::WorkingMemoryChangeList & _wmcl) {
  
    bool used  = false;
    
    if(m_pWMRP->m_pChangeObjects) {
 
      //remove any previously flagged filters
      removeChangeFilters();

      for(unsigned int i = 0; i < _wmcl.length(); i++) {
      

// 	WorkingMemoryChangeFilterMap::const_iterator receiverList
// 	  = m_pWMRP->m_pChangeObjects->get(_wmcl[i]);
	m_pWMRP->m_pChangeObjects->get(_wmcl[i], m_receivers);
	
	ostringstream outStream;
	outStream<<"change: "<<_wmcl[i]<<" , receivers: "<< m_receivers.size();
	m_pWMRP->debug(outStream.str());

	if(!m_receivers.empty()) {

	  //m_pWMRP->printfln("receiver list length = %i",receiverList->second.size());
	  
    
	  WorkingMemoryChangeReceiver * pReceiver = NULL;

	  for(vector<WorkingMemoryChangeReceiver *>::iterator j = m_receivers.begin();
	      j < m_receivers.end();
	      j++) {

	    pReceiver = *j;

	    //m_pWMRP->println("pReceiver got");

	    //if the filter matches
	    if(NULL != pReceiver) {
	    
	      try {
		//m_pWMRP->println("calling change");
	  
		pReceiver->workingMemoryChanged(_wmcl[i]);
		used = true;
		//m_pWMRP->println("done");
	      }
	      catch(const DoesNotExistOnWMException &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass DoesNotExistOnWMException caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(const CASTException &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass CASTException caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(const BALTException &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass BALTException caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(const CORBA::BAD_PARAM &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass CORBA::BAD_PARAM caught");		
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		m_pWMRP->println(e._name());
		std::abort();
	      }
	      catch(const CORBA::Exception &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass CORBA::Exception caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));
		m_pWMRP->println(e._name());
		std::abort();
	      }
	      catch(const std::exception &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass std::exception caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(...) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass unknown exception caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl[i].m_type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl[i].m_address.m_id) + " in " + 
				 string(_wmcl[i].m_address.m_subarchitecture));	  		
		std::abort();
	      }

	    }
	    else {
	      //this is ok now
	      //ostringstream ostr;	
	      //ostr<<"Missing change object for filtered change: "
	      //    <<m_tmpFilter.m_type<<m_tmpFilter.m_operation<<endl;
	      //throw BALTException(__HERE__, ostr.str().c_str());
	    }
	  }

	  m_receivers.clear(); 


	}


	  //remove any outstanding change filters before forwarding
	  //changes
	removeChangeFilters();
	
      }
    
    }

    return used;
    //this looks like a good place to give up control 
    //omni_thread::yield();
  }

  void WorkingMemoryChangeThread::removeChangeFilters() const {
    if(!m_pWMRP->m_receiversToRemove.empty()) {
      //remove any receivers as requested during event processing
      for(vector<const WorkingMemoryChangeReceiver *>::iterator i = m_pWMRP->m_receiversToRemove.begin();
	  i < m_pWMRP->m_receiversToRemove.end();
	  ++i) {
	m_pWMRP->removeChangeFilterHelper(*i);	
      }
      m_pWMRP->m_receiversToRemove.clear();
    }
  }


  void WorkingMemoryChangeThread::runDiscard() {

    cdl::WorkingMemoryChangeList changeList;
    changeList.length(1); // length is always 1 on discard
    bool listEmpty = true;
    bool used = false;
	
    while(m_bRun) {
    
      //cout<<m_pWMRP->getProcessIdentifier()<<": "<<m_pChangeList->size()<<" change events "<<endl;

      m_queueAccess.lock();
      listEmpty = m_pChangeList->empty();
      m_queueAccess.unlock();




      while(!listEmpty) {

	//produce the correct list contents..
      
	// for backward compat discard all but more recent change!
    
	//try to gain access
	////if(m_pWMRP->m_semaphore.trywait()) {
    
	//m_pWMRP->println("m_pWMRP locking");
	m_pWMRP->lockProcess();
	//m_pWMRP->println("m_pWMRP locked");
	//cout<<m_pWMRP->getProcessIdentifier()<<": "<<" RUN "<<endl;


	m_queueAccess.lock();
	
	if(m_pChangeList->size() > 1) {
	  ostringstream outStream;
	  outStream<<"discarding "<<m_pChangeList->size() - 1<<" change events ";
	  m_pWMRP->debug(outStream.str());
	}

	changeList[0] = m_pChangeList->front();
	m_pChangeList->clear();
	m_queueAccess.unlock();
	
	try {

	  //remove any outstanding change filters before forwarding
	  //changes
	  removeChangeFilters();

	  //then write to derived class
	  used = forwardToSubclass(changeList);
	}
	catch(const BALTException & _e) {
	  ostringstream outStream;
	  outStream<<"Caught filter exception, have you removed a filter\n"
		   <<_e.what();
	  m_pWMRP->println(outStream.str());
	}

	//unlock this thread now something has been done
	//m_pWMRP->println("m_pWMRP unlocking");
	m_pWMRP->unlockProcess();
	//m_pWMRP->println("m_pWMRP unlocked");

	//now signal any threads that may have been waiting for new
	//changes
	if(used) {
	  m_pWMRP->m_wmcConditionMutex.lock();
	  m_pWMRP->m_pWMCCondition->broadcast();
	  m_pWMRP->m_wmcConditionMutex.unlock();
	}
	//cout<<m_pWMRP->getProcessIdentifier()<<": "<<" RUN DONE "<<endl;

	////}
	////      else {
	////	//cout<<m_pWMRP->getProcessIdentifier()<<": "<<" NOT "<<endl;
	////m_pWMRP->log("unable to access thread");
	
	////m_pWMRP->m_unlockNotificationMutex.lock();
	//////cout<<"inside mutex"<<endl;
	////while (m_bRun && !m_pWMRP->m_semaphore.trywait()) {
	//////cout<<"waiting on condition"<<endl;
	////m_pWMRP->m_pUnlockNotificationCondition->wait();
	//////cout<<"woken up"<<endl;
	////}	
	//cout<<"unlocked"<<endl;
	////m_pWMRP->m_unlockNotificationMutex.unlock();

	//must release thread after the trywait is successful above!
	////m_pWMRP->unlockProcess();


	////}

      
	m_queueAccess.lock();
	listEmpty = m_pChangeList->empty();
	m_queueAccess.unlock();

      }

      m_pChangeSemaphore->wait();

      //     m_changeConditionMutex.lock();
      //     //check still running before waiting
      //     while (m_bRun && m_pChangeList->empty()) {
      //       m_pChangeCondition->wait();
      //       //cout<<"let's go! "<<endl;
      //     }    
      //     m_changeConditionMutex.unlock();
    
    }

  }
  
  void WorkingMemoryChangeThread::start() {
    m_bRun = true;
    start_undetached();
  }

  void WorkingMemoryChangeThread::stop() {
    m_bRun = false;
    // m_changeConditionMutex.lock();
    //   m_pChangeCondition->signal();
    //   m_changeConditionMutex.unlock();

    m_pChangeSemaphore->post();

  }



  void WorkingMemoryChangeThread::queueChange(const cdl::WorkingMemoryChange & _change) {

    //m_changeConditionMutex.lock();
  
    m_queueAccess.lock();
    m_pChangeList->push_front(_change);
    //cout<<m_pWMRP->getProcessIdentifier()<<" queue size: "<<m_pChangeList->size()<<endl;
    m_queueAccess.unlock();

    m_pChangeSemaphore->post();
  

    //m_pChangeCondition->signal();
    //m_changeConditionMutex.unlock();
  }


  WorkingMemoryReaderProcess::WorkingMemoryReaderProcess(const string &_id) 
    : //InspectableComponent(_id),
      WorkingMemoryAttachedComponent(_id),
      m_pWMChangeThread(new WorkingMemoryChangeThread(this)),
      m_queueBehaviour(cdl::QUEUE),
      m_pFilterOutput(NULL),
      m_pChangeObjects(NULL),
      m_pWMCCondition(new omni_condition(&m_wmcConditionMutex)) {

      setReceiveXarchChangeNotifications(false);
      receiveChanges();
    }


  WorkingMemoryReaderProcess::~WorkingMemoryReaderProcess() {


     if (m_pChangeObjects) {

//       //delete change objects
//       for(WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver>::iterator i = m_pChangeObjects->begin();
// 	  i != m_pChangeObjects->end();
// 	  i++) {


// 	for(unsigned int j = 0;
// 	    j < i->second.size();
// 	    j++) {

	  
// 	  //TODO .. if the objects are components, then this breaks the system!!!
// 	  //delete i->second[j];
// 	  i->second[j] = NULL;
	  
// 	}

//       }
        
//       m_pChangeObjects->clear();
      delete m_pChangeObjects;
      m_pChangeObjects = NULL;
    }

    if(m_pWMChangeThread) {
      
      m_pWMChangeThread->stop();

      int* rv;
      //cout<<"waiting for WorkingMemoryChangeThread... ";
      
      try {
	//join calls the destuctor too
	m_pWMChangeThread->join((void**)&rv);
	//cout<<"OK"<<endl;
      } 
      catch (const omni_thread_invalid& e) {
	cout<<" THREAD INVALID... was it started?"<<endl;
      }
    }

    //clean up condition variable
    m_wmcConditionMutex.lock();
    m_pWMCCondition->broadcast();
    m_wmcConditionMutex.unlock();
    delete m_pWMCCondition;


  }


  void WorkingMemoryReaderProcess::start() {
    WorkingMemoryAttachedComponent::start();

    if(m_pWMChangeThread) {
      m_pWMChangeThread->start();
    }
  }

  void WorkingMemoryReaderProcess::stop() {
    WorkingMemoryAttachedComponent::stop();
    
    if(m_pWMChangeThread) {
      m_pWMChangeThread->stop();
    }

    //release sleeping threads
    m_wmcConditionMutex.lock();
    m_pWMCCondition->broadcast();
    m_wmcConditionMutex.unlock();

  }

  bool WorkingMemoryReaderProcess::isReceivingXarchChangeNotifications() {
    return m_receiveXarchChangeNotifications;
  }

  void WorkingMemoryReaderProcess::setReceiveXarchChangeNotifications(
								      bool _receiveXarchChangeNotifications) {
    m_receiveXarchChangeNotifications = _receiveXarchChangeNotifications;

  }

  void 
  WorkingMemoryReaderProcess::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData) {
  
    if(m_status == STATUS_RUN && m_bReceivingChanges) {

      //prefer to use change objects
      if(m_pChangeObjects) {
	m_pWMChangeThread->queueChange(_pData->getData());      
      }
    
    }
  
    delete _pData;

  }

  



  void 
  WorkingMemoryReaderProcess::setPushConnector(const string & _connectionID, 
					       PushConnectorOut<cdl::WorkingMemoryChangeFilter> * _pOut) {
    m_pFilterOutput = _pOut;
  }

  void WorkingMemoryReaderProcess::receiveNoChanges() {
    m_bReceivingChanges = false;
  }

  void WorkingMemoryReaderProcess::receiveChanges() {
    m_bReceivingChanges = true;
  }




  void WorkingMemoryReaderProcess::addChangeFilter(const WorkingMemoryChangeFilter & _filter, //not const because we want to change it if it's incorrect!
						   WorkingMemoryChangeReceiver * _pReceiver) {


    assert(_pReceiver != NULL);
    
    //    logf("%s %s %s %s %d", _type.c_str(), _src.c_str(), _changeID.c_str(), _changeSA.c_str(), _restriction);
    
    if(!m_pFilterOutput){
      throw(BALTException(__FILE__, __FUNCTION__, __LINE__, 
			  "Filter ouput connection not yet set. Call addChangeFilter methods in an overridden start() method or later."));
    }

    //make our own copy for storing then sending
    WorkingMemoryChangeFilter *pFilter = new WorkingMemoryChangeFilter(_filter);

    //sneaky operation to keep filter consistent without always
    //needing a component for info
    if(pFilter->m_restriction == cdl::LOCAL_SA &&
       strlen(pFilter->m_address.m_subarchitecture) == 0) {
      pFilter->m_address.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
    }
    //also make sure subarch/restrictions are correct
    else if(strcmp(pFilter->m_address.m_subarchitecture,m_subarchitectureID.c_str()) == 0 && 
	    pFilter->m_restriction == cdl::ALL_SA) {
      pFilter->m_restriction = cdl::LOCAL_SA;
    } 
    

    //set the origin of the filter
    pFilter->m_origin = CORBA::string_dup(getProcessIdentifier().c_str());    
  
    // lazy creation
    if (!m_pChangeObjects) {
      m_pChangeObjects = new WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver *>();
    }

    (*m_pChangeObjects)[*pFilter].push_back(_pReceiver);
    //cout<<"new filter length: "<<m_pChangeObjects->size()<<endl;
  

    //this frees up the filter memory
    m_pFilterOutput->
      push(new FrameworkLocalData<WorkingMemoryChangeFilter>(getProcessIdentifier(),
							     pFilter));

    //and flush it
    m_pFilterOutput->flush();
  }






  void WorkingMemoryReaderProcess::removeChangeFilter(const WorkingMemoryChangeReceiver * _pReceiver, const  ReceiverDeleteCondition & _condition) {

    if(cdl::DELETE_RECEIVER == _condition) {
      _pReceiver->deleteOnRemoval();
    }

    m_receiversToRemove.push_back(_pReceiver);
  }

  void WorkingMemoryReaderProcess::removeChangeFilterHelper(const WorkingMemoryChangeReceiver * _pReceiver) {
    
    assert(_pReceiver != NULL);
    assert(m_pFilterOutput != NULL);

    if (m_pChangeObjects) {
      vector<WorkingMemoryChangeFilter> removed;


      //TODO: why does the following line not compile?
      //m_pChangeObjects->remove(_pReceiver, removed);
      //HACK: to overcome above oddness
      WorkingMemoryChangeReceiver * receiver = const_cast<WorkingMemoryChangeReceiver*>(_pReceiver);
      m_pChangeObjects->remove(receiver, removed);
      ///logf("filters to remove: %d", removed.size());


      for(vector<WorkingMemoryChangeFilter>::iterator i = removed.begin();
	  i < removed.end(); ++i) {
	//create a new filter to send
	WorkingMemoryChangeFilter * pFilter = new WorkingMemoryChangeFilter(*i);
	//and say we're deleting it
	pFilter->m_filterChange = cdl::DELETE;
	//then send
	m_pFilterOutput->
	  push(new FrameworkLocalData<WorkingMemoryChangeFilter>(getProcessIdentifier(),
								 pFilter));
      }

      //if we need to clean this up, then do so
      if(_pReceiver->isDeletedOnRemoval()) {
	debug("Deleting receiver from memory and setting pointer to NULL");
	delete _pReceiver;
	_pReceiver = NULL;
      }

    }




  }


//   void WorkingMemoryReaderProcess::removeChangeFilter(const string & _type,
// 						      const WorkingMemoryOperation & _op, 
// 						      const string & _src, 
// 						      const string & _changeID,
// 						      const string & _changeSA, 
// 						      const FilterRestriction & _restriction) {
  


//     if(!m_pChangeObjects) {
//       throw(BALTException(__FILE__, __FUNCTION__, __LINE__, 
// 			  "No filters set, so no removal possible"));
//     }

//     if(!m_pFilterOutput){
//       throw(BALTException(__FILE__, __FUNCTION__, __LINE__, 
// 			  "Filter ouput connection not yet set. Call removeChangeFilter methods in an overridden start() method or later."));
//     }

//     WorkingMemoryChangeFilter *pFilter = new WorkingMemoryChangeFilter();
//     pFilter->m_operation = _op;
//     pFilter->m_src = CORBA::string_dup(_src.c_str());
//     pFilter->m_address.m_id = CORBA::string_dup(_changeID.c_str());
//     pFilter->m_address.m_subarchitecture = CORBA::string_dup(_changeSA.c_str());
//     pFilter->m_type = CORBA::string_dup(_type.c_str());
//     pFilter->m_restriction = _restriction;
//     pFilter->m_filterChange = cdl::DELETE;
  
//     m_pChangeObjects->erase(*pFilter);
//     //cout<<"new filter length: "<<m_pChangeObjects->size()<<endl;

//     m_pFilterOutput->
//       push(new FrameworkLocalData<WorkingMemoryChangeFilter>(getProcessIdentifier(),
// 							     pFilter));
//   }



//   void WorkingMemoryReaderProcess::clearFilterObjects() {

//     if (m_pChangeObjects) {

//       //delete change objects
//       for(WorkingMemoryChangeFilterMap::iterator i = m_pChangeObjects->begin();
// 	  i != m_pChangeObjects->end();
// 	  i++) {

// 	for(unsigned int j = 0;
// 	    j < i->second.size();
// 	    j++) {	  
// 	  delete i->second[j];
// 	  i->second[j] = NULL;
	  
// 	}
//       }
    
//       m_pChangeObjects->clear();
//       delete m_pChangeObjects;
//       m_pChangeObjects = NULL;
//     }

//   }




  void WorkingMemoryReaderProcess::waitForChanges() {
    m_wmcConditionMutex.lock();
    m_pWMCCondition->wait();
    m_wmcConditionMutex.unlock();
  }


} //namespace cast
