/*
 * BALT - The Boxes and Lines Toolkit for component communication.
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


/**
 * This is now a huge place to keep things that end up as circular
 * dependencies.
 **/

 
#ifndef PUSH_PULL_INTERFACES_H_
#define PUSH_PULL_INTERFACES_H_

#include "includes.hpp"
#include "FrameworkLocalData.hpp"
#include "FrameworkConnector.hpp"
#include "FrameworkQuery.hpp"
#include "BALTException.hpp"
#include "BALTType.hpp"

#include "LocalDatatypeManager.hpp"



template <class T>
void addLocalDatatype();

template <class T>
void addRemoteDatatype();

template <class T>
void addLocalDatatype(const std::string & _datatype);

// template <class T>
// void addRemoteAndLocalDatatypes();



/**
 * Interface to define a process that can receiver information in a
 * push-based manner.
*/
template <class T>
class PushReceiver {

public:

  PushReceiver() {
    addLocalDatatype<T>();
  }

    /**
     * Empty destructor.
     */
    virtual ~PushReceiver() {};

    /**
     * Method to receive push data. When any push connector attached
     * to this process receives data it uses this method to push the
     * data into the process. The process must then use
     * FrameworkLocalData::getSource() to determine where the
     * information came from. The PushReceiver must take
     * responsibility for manging the memory pointed to by _pData.
     *
     * @param _pData The incoming push data.
     *
     */


    virtual
    void receivePushData(FrameworkLocalData<T> *_pData) = 0;

};


/**
 * Interface to define a connector that can push data into a
 * PushReceiver process. This is separated from the PushConnectorOut
 * interface because the remote connectors do not need both aspects of
 * the functionality.
 */
template <class T>
class PushConnectorRegister {
public:

    /** Empty destructor. **/
    virtual ~PushConnectorRegister() {};

    /** Register a push receiver process with this connector.
     *
     * @param _pPr A pointer to the receiver process that will receive
     * pushes from this connector.
     */
    virtual
    void registerPushReceiver(PushReceiver<T> * _pPr) = 0;

};


/**
 * Interface to define a connector that can be used for push output
 * from a process. This is separated from the PushConnectorRegister
 * interface because the remote connectors do not need both aspects of
 * the functionality.
 */
template <class T>
class PushConnectorOut {
public:

    /** Empty destructor. **/
    virtual ~PushConnectorOut() {};


    /**
     * Push a data object on this connector. _pData should point to
     * memory that can become the responsibility of the receiving
     * process (i.e. on heap).
     *
     * @param _pData The data to be pushed.
     */

    virtual
    void push(FrameworkLocalData<T> *_pData) = 0;


    /**
     * Flush all data on this connector.
     **/
    virtual
    void flush() = 0;


};


/**
 * Interface to define a process that can send push data. The interface
 * is used to connect PushConnectorOut objects to this process.
 */
template <class T>
class PushSender {

public:
  PushSender() {
    addLocalDatatype<T>();
  }


    /** Empty destructor. **/
    virtual ~PushSender() {};

    /** Set the push connector that this process will use for
     *      output. NOTE: This is badly named because it implies only one
     *  connector will be used.
     *
     *  @param _pOut The output connector for the process.
     */

    virtual void setPushConnector(const std::string & _connectionID, 
				  PushConnectorOut<T> * _pOut) = 0;

};


/**
 * Interface to define a full push connector. 
 */
template <class T>
class PushConnector : 
  public PushConnectorOut<T>,
  public PushConnectorRegister<T>, 
  public FrameworkConnector {

public :
  PushConnector(const std::string &_connectionID) : 
    FrameworkConnector(_connectionID) {};
};



/**
 * Interface to define a process that can receive queries in a
 * pull-based manner.
*/
template <class T>
class PullReceiver {

public:

  PullReceiver() {
    addLocalDatatype<T>();
  }

    /**
     * Empty destructor.
     */
    virtual ~PullReceiver() {};

    /**
     * Receive a pull query from a PullSender. The input object may
     * define some parameters. The returned pointer should point to
     * heap memory that is left as the responsibility of the calling
     * processes.
     *
     * @param _query The query being made.  
     * @param _pData A pointer that should be pointed at the result of the query.
     * 
     */
    virtual void
      receivePullQuery(const FrameworkQuery &_query, FrameworkLocalData<T> *& _pData) = 0;
};


/**
 * Interface to define a connector that can pull data from a
 * PullReceiver process. This is separated from the PullConnectorOut
 * interface because the remote connectors do not need both aspects of
 * the functionality.
 */

template <class T>
class PullConnectorRegister {
public:

    /** Empty destructor. **/
    virtual ~PullConnectorRegister() {};

    /** Register a pull receiver process with this connector.
     *
     * @param _pPr A pointer to the receiver process that will receive
     * pulles from this connector.
     */
    virtual void registerPullReceiver(PullReceiver<T> * _pPr) = 0;

};


/**
 * Interface to define a connector that can be used for pull output
 * from a process. This is separated from the PullConnectorRegister
 * interface because the remote connectors do not need both aspects of
 * the functionality.
 */
template <class T>
class PullConnectorOut {
public:

    /** Empty destructor. **/
    virtual ~PullConnectorOut() {};


    /**
     * Pull a data object on this connector.
     *
     * @param _query The data to be pulled.
     *
     * @param _pData A pointer to a data object in memory that becomes
     * the responsibility of the calling process.
     */
    virtual void pull(const FrameworkQuery &_query, FrameworkLocalData<T> *& _pData) = 0;
};


/**
 * Interface to define a process that can send pull data. The interface
 * is used to connect PullConnectorOut objects to this process.
 */
template <class T>
class PullSender {

public:

  PullSender() {
    addLocalDatatype<T>();
  }


    /** Empty destructor. **/
    virtual ~PullSender() {};

    /** Set the pull connector that this process will use for
     *  output. NOTE: This is badly named because it implies only one
     *  connector will be used.
     *
     *  @param _pOut The output connector for the process.
     */

    virtual void setPullConnector(const std::string & _connectionID, 
				  PullConnectorOut<T> * _pOut) = 0;

};


/**
 * Interface to define a full pull connector.
 */
template <class T>
class PullConnector : public PullConnectorOut<T>,
  public PullConnectorRegister<T>, public FrameworkConnector {

 public :
  PullConnector(const std::string &_connectionID) :FrameworkConnector(_connectionID) {};

};




template <class T>
class PushSendingThread : public omni_thread {

 protected:

  std::vector<PushReceiver<T>*> * m_pPushReceivers;
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

 public:


  PushSendingThread(const std::string & _cid) :
    omni_thread() {
    m_pPushReceivers = new std::vector<PushReceiver<T>*>();
    m_pDataList = new std::queue<FrameworkLocalData<T> *>();
    //m_pListCondition = new omni_condition(&m_listConditionMutex);

    m_pListContentsSemaphore = new omni_semaphore(0);
    m_pListIsEmptySemaphore = new omni_semaphore(0);

    m_isSending = true; //safer to assume that something is happening
    bRun = false;
    m_cid = std::string(_cid);
  };


    void registerPushReceiver(PushReceiver<T> * _pPr) {
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

    virtual ~PushSendingThread() {
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
  
      //static int count = 0;

      //double check
      if(bRun) {
  
	//count++;

	//if there is only one connected receiver, then don't copy
	if(1 == m_pPushReceivers->size()) {
	  //std::cout<<m_cid<<": "<<"before receive: "<<count<<" "<<_pData->getSource()<<std::endl;
	  m_pPushReceivers->at(0)->receivePushData(_pData);
	  //std::cout<<m_cid<<": "<<"after receive: "<<count<<std::endl;
	}      
	//else they all get copies
	else {
	  for(unsigned int i = 0; i < m_pPushReceivers->size(); i++) {
	    //std::cout<<m_cid<<": "<<"before receive: "<<count<<" "<<_pData->getSource()<<std::endl;

	    m_pPushReceivers->at(i)->receivePushData(new FrameworkLocalData<T>(*_pData));
	    //std::cout<<m_cid<<": "<<"after receive: "<<count<<std::endl;
	  }
	  //delete the original
	  delete _pData;
	  _pData = NULL;
	}
      }
      else {
	//delete the original
	delete _pData;
	_pData = NULL;
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
class PushConnectorImpl : public PushConnector<T> {
  
  

 public:


  

  PushConnectorImpl(const std::string & _connectorID) 
    : PushConnector<T>(_connectorID) {
    m_pSendThread = NULL;    
  }
    
    virtual ~PushConnectorImpl() {
      //TODO wait for thread
      //std::cout<<"~PushConnectorImpl "<<m_cid<<std::endl;

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
  
    virtual void stopConnector(){
      if(m_pSendThread) {
	m_pSendThread->stop();
      }
    };

    virtual void push(FrameworkLocalData<T> * _pData) {

      if(m_pSendThread) {
	m_pSendThread->queue(_pData);
      }
    }

    virtual void flush() {
      if(m_pSendThread) {
	m_pSendThread->flush();
      }

    }
  
    //need to redeclare to allow usage in templated method
    const std::string & getConnectorIdentifier() const {
      return FrameworkConnector::getConnectorIdentifier();
    }


    virtual void registerPushReceiver(PushReceiver<T> * _pPr) {

      //create send thread 
      if(!m_pSendThread) {
	m_pSendThread = 
	  new PushSendingThread<T>(getConnectorIdentifier());
	m_pSendThread->start();
      }

      m_pSendThread->registerPushReceiver(_pPr);
    }
    
 protected:
    PushSendingThread<T> * m_pSendThread;

};




//pull is always single threaded as it blocks

template <class T>
class PullConnectorImpl : public PullConnector<T> {

 public:

  PullConnectorImpl(const std::string &_connectorID) : PullConnector<T>(_connectorID) {
    m_pPullReceiver = NULL;
  }


    virtual ~PullConnectorImpl() {}

    //PullConnector methods...

    virtual void pull(const FrameworkQuery &_query, FrameworkLocalData<T> *&_pData) {
      //std::cout<<"PullConnectorImpl::pull()"<<std::endl;
      //could maybe check that the pointer is null before using it...
      m_pPullReceiver->receivePullQuery(_query,_pData);
    }

    virtual void registerPullReceiver(PullReceiver<T> * _pPr) {
      m_pPullReceiver = _pPr;
    }

 private:


    PullReceiver<T> * m_pPullReceiver;

};



template <class T>
class GenericConnectionCreator : public LocalConnectionCreator {
  
public:

  GenericConnectionCreator(){
    //cout<<"GenericConnectionCreator: "<<" typeid:"<<typeid(T).name()<<endl;
  };

  virtual ~GenericConnectionCreator(){};
  
  virtual PushConnector<T> * createPushConnection(const std::vector<FrameworkProcess *>  &_senders, 
						  const std::vector<FrameworkProcess *> &_receivers, 
						  const std::string &  _connectorID) const { 


    //cerr<<"createPushConnection  IN"<<endl;
    //cerr<<_connectorID<<endl;
    //cerr.flush();


    //create connector
    PushConnectorImpl<T> *p_conn = new PushConnectorImpl<T>(_connectorID);
    
    //for each of the senders
    
    for(std::vector<FrameworkProcess *>::const_iterator i = _senders.begin();
	i < _senders.end(); i++) {
       

      FrameworkProcess *pfp = *i;


/*       if(pfp->getProcessIdentifier() == "funny.man") { */
/* 	//cout<<"special case"<<endl; */

/* 	PushSender<std::std::string>  * pss = dynamic_cast<PushSender<std::std::string> * >(pfp); */

/* 	if(!pss) { */
/* 	  delete p_conn; */
	  
/* 	  ostd::stringstream ostr;	 */
/* 	  ostr<<"ERROR: Cannot cast to PushSender<std::std::string>: "<<(*i)->getProcessIdentifier()<<" id:"<<_connectorID<<" typeid:"<<typeid(T).name()<<endl; */
	  
/* 	  throw BALTException(__HERE__, ostr.str().c_str()); */
/* 	} */
/*       } */

      //cast to the correct types..
      PushSender<T> *p_psp = dynamic_cast<PushSender<T>*>(pfp);
      
      if(!p_psp) {
	delete p_conn;

	std::ostringstream ostr;	
	ostr<<"ERROR: Cannot cast to PushSender: "<<(*i)->getProcessIdentifier()<<" id:"<<_connectorID<<" typeid:"<<typeid(T).name()<<std::endl;
 
	throw BALTException(__HERE__, ostr.str().c_str());

      }
      else {
	p_psp->setPushConnector(_connectorID,p_conn);
      }
    }

    
    
    //for each of the receivers
    for(std::vector<FrameworkProcess *>::const_iterator i = _receivers.begin();
	i < _receivers.end(); i++) {
    
      //cast to correct type
      PushReceiver<T> *p_prp = dynamic_cast<PushReceiver<T>*>(*i);
      //PushReceiver<T> *p_prp = (PushReceiver<T>*)*i;
      
      if(!p_prp) {
	std::ostringstream ostr;
	ostr<<"ERROR: Cannot cast to PushReceiver: "<<(*i)->getProcessIdentifier()<<" "<<_connectorID<<" "<<typeid(T).name()<<std::endl;
	delete p_conn;
	throw BALTException(__HERE__, ostr.str().c_str());
      }
      else {
	p_conn->registerPushReceiver(p_prp);
      }
    }

    //cerr<<"createPushConnection OUT"<<endl;
    //cerr.flush();

    
    return p_conn;

  }

  virtual PullConnector<T> * createPullConnection(const std::vector<FrameworkProcess *>  &_senders, 
						  FrameworkProcess * _receiver, 
						  const std::string &  _connectorID) const {

    PullConnectorImpl<T> *p_conn = new PullConnectorImpl<T>(_connectorID);
    
    //for each sender
    for(std::vector<FrameworkProcess *>::const_iterator i = _senders.begin();
	i < _senders.end(); i++) {
       
      //cast them to the correct types..
      PullSender<T> *p_psp = dynamic_cast<PullSender<T>*>(*i);
    
      if(!p_psp) {
	std::ostringstream ostr;
	ostr<<"ERROR: Cannot cast to PullSender: "<<(*i)->getProcessIdentifier()<<" "<<typeid(T).name()<<std::endl;
	delete p_conn;
	throw BALTException(__HERE__, ostr.str().c_str());
		
      }
      else {
	p_psp->setPullConnector(_connectorID,p_conn);      
      }
      
    }

    PullReceiver<T> *p_prp = dynamic_cast<PullReceiver<T>*>(_receiver);
    if(!p_prp) {
      std::ostringstream ostr;
      ostr<<"ERROR: Cannot cast to PullReceiver: "<<_receiver->getProcessIdentifier()<<" "<<typeid(T).name()<<std::endl;
      delete p_conn;
      throw BALTException(__HERE__, ostr.str().c_str());
	
    }
    
    p_conn->registerPullReceiver(p_prp);

    return p_conn;
  }

};


template <class T>
void addLocalDatatype() {
  const std::string & datatype(BALTType::typeName<T>());
  LocalDatatypeManager::addDatatype(datatype,new GenericConnectionCreator<T>());
}


template <class T>
void addLocalDatatype(const std::string & _datatype) {
  LocalDatatypeManager::addDatatype(_datatype,new GenericConnectionCreator<T>());
}



//nasty loop, but hey
//#include <balt/interface/RemoteDatatypeManager.hpp>

// template <class T>
// void addRemoteDatatype() {
//   RemoteDatatypeManager::addObjectDatatype<T>();
// }

// template <class T>
// void addRemoteAndLocalDatatypes() {
//   addLocalDatatype();
//   addRemoteDatatype();
// }



#endif
