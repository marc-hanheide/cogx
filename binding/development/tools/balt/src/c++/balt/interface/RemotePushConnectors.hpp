/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef REMOTE_PUSH_CONNECTORS_H_
#define REMOTE_PUSH_CONNECTORS_H_

#include <balt/core/BALTCore.hpp>
#include <balt/core/BALTType.hpp>
#include "RemoteDataTranslator.hpp"
#include "includes.hpp"



template <class T>
class RemotePushSendingThread : public omni_thread {

protected:

  //was before...
  //  m_out = RemoteConnectors::RemotePushConnector::_nil();
  std::vector<RemoteConnectors::RemotePushConnector_ptr> * m_pPushReceivers;
  std::queue<FrameworkLocalData<T> *> * m_pDataList;
  omni_mutex m_listAccess;
  omni_mutex m_connAccess;

  omni_mutex m_flushLock;



  //omni_mutex m_listConditionMutex;
  //omni_condition * m_pListCondition;

  omni_semaphore * m_pListContentsSemaphore;
  omni_semaphore * m_pListIsEmptySemaphore;

  //default state is that we are in the middle of sending
  bool m_isSending;
  bool bRun;

  std::string m_cid;

 
  RemoteDataTranslator<T> * m_pTranslator;


public:


  RemotePushSendingThread(RemoteDataTranslator<T> * _pTranslator) :
    omni_thread(),
    m_pTranslator(_pTranslator) {
    m_pPushReceivers = new std::vector<RemoteConnectors::RemotePushConnector_ptr>();
    m_pDataList = new std::queue<FrameworkLocalData<T> *>();
    //m_pListCondition = new omni_condition(&m_listConditionMutex);

    m_pListContentsSemaphore = new omni_semaphore(0);
    m_pListIsEmptySemaphore = new omni_semaphore(0);

    m_isSending = true; //safer to assume that something is happening
    bRun = false;    
  };


  void registerPushReceiver(RemoteConnectors::RemotePushConnector_ptr _pPr) {
    m_pPushReceivers->push_back(_pPr);
  }

  void queue(FrameworkLocalData<T> * _pData) {
    if(bRun) {
      
      //wait if we're flushing
      m_flushLock.lock();
      
      //queue data for sending
      //std::cout<<"call data queue "<<m_cid<<std::endl;
      m_listAccess.lock();
      m_pDataList->push(_pData);
      m_listAccess.unlock();
      
      m_pListContentsSemaphore->post();
      
      //wait if we're flushing
      m_flushLock.unlock();
      
      
    }
  }
  
  void flush() {
    
    //std::cout<<"PushConnectorImpl::flush 1"<<std::endl;
    
    //lock list from further additions during the flush
    m_flushLock.lock();
    
    m_listAccess.lock();
    bool listEmpty = m_pDataList->empty();
    m_listAccess.unlock();
    
    //std::cout<<"PushConnectorImpl::flush 2"<<std::endl;
    
    if(!listEmpty || m_isSending) {
      
      //if the list isn't empty, or we're in the middle of sending, then
      //it's safe to wait... the list will not get any fuller as we're
      //locking queue
      
      while(!listEmpty || m_isSending) {
	//std::cout<<m_cid<<": "<<"just waiting for list to become empty"<<std::endl;
	m_pListIsEmptySemaphore->wait();
	
	//update list state
	m_listAccess.lock();
	listEmpty = m_pDataList->empty();
	m_listAccess.unlock();
	
	//std::cout<<m_cid<<": "<<"OK then!"<<std::endl;
      }
      
      assert(m_pDataList->empty());
      
    }

    //after all data has been flushed to connectors, flush all
    //connectors too
    for(unsigned int i = 0; i < m_pPushReceivers->size(); i++) {
      m_pPushReceivers->at(i)->flush();
    }
    
    //now let things start happening again
    m_flushLock.unlock();
    
    //std::cout<<"PushConnectorImpl::flush 3"<<std::endl;
    
  }
  
  void start() {
    bRun = true;
    start_undetached();
  }

  void stop() {
    bRun = false;
    
    m_listAccess.lock();
    while(!m_pDataList->empty()) {
      m_pDataList->pop(); //Delete contents also???
    }
    m_listAccess.unlock();
    
    m_pListContentsSemaphore->post();
  }
  

protected:

  virtual ~RemotePushSendingThread() {
    //    m_listAccess.lock();
    //m_connAccess.lock();
    //delete m_pPushReceivers;
    //TODO should empty the list too!
    //delete m_pDataList;
    //delete m_pListCondition;
    m_pListContentsSemaphore->post();
    delete m_pListContentsSemaphore;
  }

  void sendData(FrameworkLocalData<T> * _pData) {
  
    try {

      if(1 == m_pPushReceivers->size()) {
	if(CORBA::is_nil(m_pPushReceivers->at(0))) {
	  std::cerr<<"setPushConnector not called, or pointer connector is null"<<std::endl;
	} 
	else {
	      
	  CORBA::Any a;
	    
	  //consuming translation 
	  m_pTranslator->translate(_pData->data(),a);
	    
	  //cout<<"translation done"<<std::endl;
	  //cout.flush();
	    
	  m_pPushReceivers->at(0)->push(_pData->getSource().c_str(),a);

	  //no need to delete as it's been consumed
	  _pData = NULL;
	
	}
      }            
      else {

	std::cerr<<"WARNING: remote sending to multiple receivers not tested"<<std::endl;

	for(unsigned int i = 0; i < m_pPushReceivers->size(); i++) {
	  
	  
	  if(CORBA::is_nil(m_pPushReceivers->at(i))) {
	    std::cerr<<"setPushConnector not called, or pointer connector is null"<<std::endl;
	  } 
	  else {
	      
	    CORBA::Any a;
	    //consuming translation, so copy for multiple receivers
	    // will never happen tho... currently
	    T * t = new T(*_pData->data());
	    m_pTranslator->translate(t,a);	    
	    //cout<<"translation done"<<std::endl;
	    //cout.flush();	    
	    m_pPushReceivers->at(i)->push(_pData->getSource().c_str(),a);
	    t = NULL;

	  }
	  
	}


	//free data memory
	delete _pData;      
      }
    } 
    catch(const CORBA::Exception & e) {
      std::cerr<<"CORBA::Exception caught in RemotePushSendingThread::sendData()"<<std::endl;
      std::cerr<<e._name()<<std::endl;
      std::cerr<<"typename: "<<BALTType::typeName<T>()<<std::endl;
      std::abort();//what error
    }
    catch(const std::exception & e) {
      std::cerr<<"std::exception caught in RemotePushSendingThread::sendData()"<<std::endl;
      std::cerr<<"typename: "<<BALTType::typeName<T>()<<std::endl;
      std::cerr<<e.what()<<std::endl;
      std::abort();//what error
    }

  }
  
  virtual void* run_undetached(void *arg) {

    //std::cout<<"running "<<m_cid<<std::endl;
    
    FrameworkLocalData<T> * pData = NULL;
    bool listEmpty = true;

    while (bRun) {
      
      m_listAccess.lock();
      listEmpty = m_pDataList->empty();
      m_listAccess.unlock();

      while(!listEmpty) {

	//get next from list
	m_listAccess.lock();
	pData = m_pDataList->front();
	m_pDataList->pop();
	m_listAccess.unlock();

	//send it
	m_connAccess.lock();
	sendData(pData);
	m_connAccess.unlock();

	//check list contents
	m_listAccess.lock();
	listEmpty = m_pDataList->empty();
	m_listAccess.unlock();
	
	//std::cout<<"END OF SEND "<<m_cid<<std::endl;
	//std::cout<<"END OF SEND "<<m_pDataList->size()<<std::endl;
      }
	
	
      //set all data sent to true
      m_isSending = false;      

      //post on list empty semaphore
      m_pListIsEmptySemaphore->post();
      



      m_pListContentsSemaphore->wait();
	
      //set sending to true
      m_isSending = true;
    }

    //std::cout<<"home! "<<m_cid<<std::endl;

    //necessary return bits
    int* rv = new int(0);
    return (void*)rv;
  }
  

};


template <class T>
class RemotePushSenderImpl : 
  public POA_RemoteConnectors::RemotePushSender,
  public FrameworkConnector,
  public PushConnectorOut<T> {

public:

  RemotePushSenderImpl(const std::string & _id) : FrameworkConnector(_id) {
    m_pTranslator = NULL;
    //cout<<"remote push type: "<<typeid(T).name()<<std::endl;
    m_pSendThread = NULL;    

  }

  virtual ~RemotePushSenderImpl() {
    if(m_pSendThread) {
      
      m_pSendThread->stop();
      
      int* rv;
      std::cout<<"waiting for "<<getConnectorIdentifier()<<" PushSender thread... ";
      
      try {
	//join calls the destuctor too
	m_pSendThread->join((void**)&rv);
	std::cout<<"OK"<<std::endl;
      } 
      catch (const omni_thread_invalid& e) {
	std::cout<<" THREAD INVALID... was it started?"<<std::endl;
      }
    }
    
  }

  //PushConnectorOut methods...
  virtual void push(FrameworkLocalData<T> *_pData)  {
    //cout<<"remote push type: "<<typeid(_pData.getData()).name()<<std::endl;
    if(m_pSendThread) {
      m_pSendThread->queue(_pData);
    }
  }

  virtual void flush() {
    if(m_pSendThread) {
      m_pSendThread->flush();
    }    
  }
    
  //RemotePushSender methods...
  virtual void setPushConnector(RemoteConnectors::RemotePushConnector_ptr _rpc) {
    
    //create send thread 
    if(!m_pSendThread) {
      m_pSendThread = 
	new RemotePushSendingThread<T>(m_pTranslator);
      m_pSendThread->start();
    }
    
    m_pSendThread->registerPushReceiver(_rpc);
  }


  /**
   * Sets the translator object used by this connector. This memory
   * is then owned by this objects and could be deleted at any time!
   * This also destroys any previously stored translator.
   */
  void setTranslator(RemoteDataTranslator<T> * _pTranslator) {
    if(m_pTranslator) {
      delete m_pTranslator;
    }

    m_pTranslator = _pTranslator;
  }




protected:
  RemotePushSendingThread<T> * m_pSendThread;
  RemoteDataTranslator<T> * m_pTranslator;
};


template <class T>
class RemotePushReceiverImpl : 
  public POA_RemoteConnectors::RemotePushReceiver,
  public FrameworkConnector,
  public PushConnectorRegister<T> {

public:
  RemotePushReceiverImpl(const std::string & _id) : FrameworkConnector(_id)  {
    m_pTranslator = NULL;
    m_pPushReceiver = NULL;
  }
  

  virtual ~RemotePushReceiverImpl() {
    //delete m_pPushReceiver;
  }
  
  //PushConnectorRegister methods...
  virtual void registerPushReceiver(PushReceiver<T> * _pPr)  {
    m_pPushReceiver = _pPr;
  }

  //RemotePushReceiver methods...
  virtual void receivePushData(const char* _src, const CORBA::Any &_pData) {
    

    try {
      //TODO some checking here
      
      //cout<<"void RemotePushReceiverImpl::receivePushData(const char* _src, const CORBA::Any &_pData)"<<std::endl;
      //cout<<_src<<std::endl;
      
      //allocates new memory
      T * p_data = m_pTranslator->translate(_pData);
      //which is then stored into the local data wrapper
      m_pPushReceiver->receivePushData(new FrameworkLocalData<T>(_src,p_data));
      
    } 
    catch(std::exception & e) {
      std::cerr<<"Std::Exception caught in RemotePullSenderImpl::receivePullQuery()"<<std::endl;
      std::cerr<<e.what()<<std::endl;
      std::abort();//what error
    }

  }
  
  /**
   * Sets the translator object used by this connector. This memory
   * is then owned by this objects and could be deleted at any time!
   * This also destroys any previously stored translator.
   */
  void setTranslator(RemoteDataTranslator<T> * _pTranslator) {
    if(m_pTranslator) {
      delete m_pTranslator;
    }
    m_pTranslator = _pTranslator;
  }

private:
  PushReceiver <T>  * m_pPushReceiver;
  RemoteDataTranslator<T> * m_pTranslator;
  
};

#endif
