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

#include "WorkingMemoryReaderComponent.hpp"

#include <sstream>

using namespace std;
using namespace boost;
using namespace Ice;
using namespace IceUtil;

namespace cast {

  using namespace cdl;

  WorkingMemoryChangeThread::WorkingMemoryChangeThread(WorkingMemoryReaderComponent * _pWMRP) :
    m_bRun(false),
    m_pWMRP(_pWMRP) {
    
    
  }


  WorkingMemoryChangeThread::~WorkingMemoryChangeThread() {
  }


 
  void WorkingMemoryChangeThread::run() {
    m_bRun = true;

    //m_pWMRP->println("running change thread");

    if(cdl::DISCARD == m_pWMRP->m_queueBehaviour) {
      runDiscard();
    }
    else if(cdl::QUEUE == m_pWMRP->m_queueBehaviour) {
      runQueue();
    }
  }
  

  //TODO locking needs a massive reworking and probably simplification
  void WorkingMemoryChangeThread::runQueue() {

    list<cdl::WorkingMemoryChange> changeList;
    bool listEmpty = true;
    bool used = false;

    while(m_bRun) {
    
      m_queueAccess.lock();
      listEmpty = m_changeList.empty();
      m_queueAccess.unlock();

      //m_pWMRP->println("list empty: %d",listEmpty);

      while(!listEmpty) {

	//      m_pWMRP->println("doing");


	m_pWMRP->lockComponent();

	//      m_pWMRP->println("component locked");


	m_queueAccess.lock();

	//      m_pWMRP->println("queue locked");

	
	while(!m_changeList.empty()) {
	  //get last element
	  changeList.push_front(m_changeList.back());
	  //and remove it
	  m_changeList.pop_back();
	}	
      
	assert(m_changeList.size() == 0); //"error: emptied change list is not empty!");
       
	
	m_queueAccess.unlock();

	
	//remove any outstanding change filters before forwarding
	//changes
	removeChangeFilters();
	
	//then write to derived class
	used = forwardToSubclass(changeList);
	assert(changeList.size() == 0);
	

	//unlock this thread now something has been done
	m_pWMRP->unlockComponent();

	//now signal any threads that may have been waiting for new
	//changes
	if(used) {
	  Monitor<IceUtil::Mutex>::Lock lock(m_pWMRP->m_wmcMonitor);      
	  m_pWMRP->m_wmcMonitor.notifyAll();
	}
	//cout<<m_pWMRP->getComponentIdentifier()<<": "<<" RUN DONE "<<endl;


	m_queueAccess.lock();
	listEmpty = m_changeList.empty();
	m_queueAccess.unlock();

      }
        
      //make sure someone else gets a turn...
      //yield(); -- doesn't work!


      {//in a block to ensure proper lock/unlock
	Monitor<IceUtil::Mutex>::Lock lock(m_changeMonitor);
	//	m_pWMRP->println("queue waiting");	
	m_changeMonitor.wait();	      
      }
      
      //      m_pWMRP->println("queue awake");
    
    }

  }


  bool 
  WorkingMemoryChangeThread::forwardToSubclass(std::list<cdl::WorkingMemoryChange> & _wmcl) {
  
    bool used  = false;
    
    if(m_pWMRP->m_pChangeObjects) {
 
      //remove any previously flagged filters
      removeChangeFilters();

      //for(unsigned int i = 0; i < _wmcl.length(); i++) {
      while(!_wmcl.empty()) {
	      

	// 	WorkingMemoryChangeFilterMap::const_iterator receiverList
	// 	  = m_pWMRP->m_pChangeObjects->get(_wmcl[i]);
	m_pWMRP->m_pChangeObjects->get(_wmcl.back(), m_receivers);
	
	if(m_pWMRP->m_bDebugOutput) {
	  ostringstream outStream;
	  outStream<<"change: "<<_wmcl.back()<<" , receivers: "<< m_receivers.size();
	  m_pWMRP->debug(outStream.str());
	}

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
	  
		pReceiver->workingMemoryChanged(_wmcl.back());
		used = true;
		//m_pWMRP->println("done");
	      }
	      catch(const WMException &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass WMException caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl.back().type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl.back().address.id) + " in " + 
				 string(_wmcl.back().address.subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(const CASTException &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass CASTException caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl.back().type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl.back().address.id) + " in " + 
				 string(_wmcl.back().address.subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(const std::exception &e) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass std::exception caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl.back().type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl.back().address.id) + " in " + 
				 string(_wmcl.back().address.subarchitecture));	  		
		m_pWMRP->println(e.what());
		std::abort();
	      }
	      catch(...) {
		m_pWMRP->println("WorkingMemoryChangeThread::forwardToSubclass unknown exception caught");
		m_pWMRP->println("The type of change event involved in this error was: " + string(_wmcl.back().type) );	  		
		m_pWMRP->println("The changed address involved in this error was: " + 
				 string(_wmcl.back().address.id) + " in " + 
				 string(_wmcl.back().address.subarchitecture));	  		
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

	
	//processing  done for that change
	_wmcl.pop_back();


	//remove any outstanding change filters before forwarding
	//changes
	removeChangeFilters();
	
      }
    
    }

    return used;
  }

  void WorkingMemoryChangeThread::removeChangeFilters() const {
    if(!m_pWMRP->m_receiversToRemove.empty()) {
      //remove any receivers as requested during event componenting
      for(vector<const WorkingMemoryChangeReceiver *>::iterator i = m_pWMRP->m_receiversToRemove.begin();
	  i < m_pWMRP->m_receiversToRemove.end();
	  ++i) {
	m_pWMRP->removeChangeFilterHelper(*i);	
      }
      m_pWMRP->m_receiversToRemove.clear();
    }
  }


  void WorkingMemoryChangeThread::runDiscard() {

    list<cdl::WorkingMemoryChange> changeList;
    bool listEmpty = true;
    bool used = false;
	
    while(m_bRun) {
    
      m_queueAccess.lock();
      listEmpty = m_changeList.empty();
      m_queueAccess.unlock();

      while(!listEmpty) {

	m_pWMRP->lockComponent();


	m_queueAccess.lock();
	
	if(m_changeList.size() > 1 && m_pWMRP->m_bDebugOutput) {
	  ostringstream outStream;
	  outStream<<"discarding "<<m_changeList.size() - 1<<" change events ";
	  m_pWMRP->debug(outStream.str());
	}

	changeList.push_front(m_changeList.front());
	
	m_changeList.clear();
	m_queueAccess.unlock();
	

	//remove any outstanding change filters before forwarding
	//changes
	removeChangeFilters();
	
	//then write to derived class
	used = forwardToSubclass(changeList);
	assert(changeList.size() == 0);
	
	m_pWMRP->unlockComponent();

	//now signal any threads that may have been waiting for new
	//changes
	if(used) {
	  Monitor<IceUtil::Mutex>::Lock lock(m_pWMRP->m_wmcMonitor);      
	  m_pWMRP->m_wmcMonitor.notifyAll();
	}
	
	m_queueAccess.lock();
	listEmpty = m_changeList.empty();
	m_queueAccess.unlock();

      }
    
      {//in a block to ensure proper lock/unlock
	Monitor<IceUtil::Mutex>::Lock lock(m_changeMonitor);
	m_changeMonitor.wait();
      }
    }

  }
  
  

  void WorkingMemoryChangeThread::stop() {
    m_bRun = false;
 
    m_queueAccess.lock();
    m_changeList.clear();
    m_queueAccess.unlock();

   
    {//in a block to ensure proper lock/unlock
      Monitor<IceUtil::Mutex>::Lock lock(m_changeMonitor);
      m_changeMonitor.notifyAll();
    }

  }



  void WorkingMemoryChangeThread::queueChange(const cdl::WorkingMemoryChange & _change) {
    if(m_bRun) {
      
      m_queueAccess.lock();
      m_changeList.push_front(_change);
      m_queueAccess.unlock();
      
      {//in a block to ensure proper lock/unlock
	Monitor<IceUtil::Mutex>::Lock lock(m_changeMonitor);
	m_changeMonitor.notify();
      }
      
      //m_pWMRP->println("queued change and notified");
    }
  }


  WorkingMemoryReaderComponent::WorkingMemoryReaderComponent() 
    : m_pWMChangeThread(new WorkingMemoryChangeThread(this)),
      m_queueBehaviour(cdl::QUEUE) {

    setReceiveXarchChangeNotifications(false);
    receiveChanges();
  }


  WorkingMemoryReaderComponent::~WorkingMemoryReaderComponent() {

  }


  void WorkingMemoryReaderComponent::startInternal() {
    WorkingMemoryAttachedComponent::startInternal();

    if(m_pWMChangeThread) {
      m_pWMChangeThreadControl = m_pWMChangeThread->start();
    }
  }

  void WorkingMemoryReaderComponent::stopInternal() {
    
    if(m_pWMChangeThread) {
      m_pWMChangeThread->stop();
      debug("joining change thread");
      m_pWMChangeThreadControl.join();
    }

    {//done in a block so lock is released asap for subclass
      //release sleeping threads
      Monitor<IceUtil::Mutex>::Lock lock(m_wmcMonitor);            
      m_wmcMonitor.notifyAll();
    }

    //println("stopping internal");

    WorkingMemoryAttachedComponent::stopInternal();

    //println("done");
  }

  bool WorkingMemoryReaderComponent::isReceivingXarchChangeNotifications() {
    return m_receiveXarchChangeNotifications;
  }

  void WorkingMemoryReaderComponent::setReceiveXarchChangeNotifications(
									bool _receiveXarchChangeNotifications) {
    m_receiveXarchChangeNotifications = _receiveXarchChangeNotifications;

  }

  
  void 
  WorkingMemoryReaderComponent::receiveChangeEvent(const cdl::WorkingMemoryChange& _wmc, 
						   const Ice::Current & _ctx) {
    if(isRunning() && m_bReceivingChanges) {
      //prefer to use change objects
      if(m_pChangeObjects) {
	m_pWMChangeThread->queueChange(_wmc);      
      }
    }
    
  }





  void WorkingMemoryReaderComponent::receiveNoChanges() {
    m_bReceivingChanges = false;
  }

  void WorkingMemoryReaderComponent::receiveChanges() {
    m_bReceivingChanges = true;
  }




  void 
  WorkingMemoryReaderComponent::addChangeFilter(const WorkingMemoryChangeFilter & _filter, 
						WorkingMemoryChangeReceiver * _pReceiver) {


     assert(_pReceiver != NULL);
     assert(m_workingMemory);

     //    logf("%s %s %s %s %d", _type.c_str(), _src.c_str(), _changeID.c_str(), _changeSA.c_str(), _restriction);
    
     //make our own copy for editig, storing, then sending
     WorkingMemoryChangeFilter filter(_filter);

    //sneaky operation to keep filter consistent without always
    //needing a component for info
    if(filter.restriction == cdl::LOCALSA &&
       filter.address.subarchitecture.length() == 0) {
      filter.address.subarchitecture = getSubarchitectureID();
    }
    //also make sure subarch/restrictions are correct
    else if(getSubarchitectureID() == filter.address.subarchitecture && 
	    filter.restriction == cdl::ALLSA) {
      filter.restriction = cdl::LOCALSA;
    } 
    

    //set the origin of the filter
    filter.origin = getComponentID();
    
    // lazy creation
    if (!m_pChangeObjects) {
      m_pChangeObjects = boost::shared_ptr<WorkingMemoryChangeFilterMap< WorkingMemoryChangeReceiver *> >(new WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver *>());
    }


    (*m_pChangeObjects)[filter].push_back(_pReceiver);
    //cout<<"new filter length: "<<m_pChangeObjects->size()<<endl;  

    m_workingMemory->registerComponentFilter(filter);
  }







  void WorkingMemoryReaderComponent::removeChangeFilter(const WorkingMemoryChangeReceiver * _pReceiver, const  ReceiverDeleteCondition & _condition) {

    if(cdl::DELETERECEIVER == _condition) {
      _pReceiver->deleteOnRemoval();
    }

    m_receiversToRemove.push_back(_pReceiver);
  }

  void WorkingMemoryReaderComponent::removeChangeFilterHelper(const WorkingMemoryChangeReceiver * _pReceiver) {
    
    assert(_pReceiver != NULL);
    assert(m_workingMemory);

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
	//remove filter from wm
	m_workingMemory->removeComponentFilter(*i);
      }

      //if we need to clean this up, then do so
      if(_pReceiver->isDeletedOnRemoval()) {
	debug("Deleting receiver from memory and setting pointer to NULL");
	delete _pReceiver;
	_pReceiver = NULL;
      }

    }
  
  }
  





  void WorkingMemoryReaderComponent::waitForChanges() {
    Monitor<IceUtil::Mutex>::Lock lock(m_wmcMonitor);      
    m_wmcMonitor.wait();
  }
  

} //namespace cast
