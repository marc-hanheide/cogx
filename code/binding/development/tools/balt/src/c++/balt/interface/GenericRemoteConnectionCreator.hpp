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

#ifndef GENERIC_REMOTE_CONNECTION_CREATOR
#define GENERIC_REMOTE_CONNECTION_CREATOR

#include <balt/core/BALTCore.hpp>

#include "RemoteDataTranslator.hpp"

#include "RemotePullConnectors.hpp"
#include "RemotePushConnectors.hpp"

#include "LocalConnectionManager.hpp"

#include "includes.hpp"



template <class T>
class GenericRemoteConnectionCreator : public RemoteConnectionCreator {

public:

  GenericRemoteConnectionCreator(RemoteDataTranslator<T> * _pTranslator) {
    m_pTranslator = _pTranslator;
  }

  virtual ~GenericRemoteConnectionCreator() {
    delete m_pTranslator;
  };

  virtual POA_RemoteConnectors::RemotePushSender * connectAsSender(FrameworkProcess * _pSender, 
								   RemoteConnectors::RemotePushConnector_ptr _rpc, 
								   const std::string & _connectionID)  const {
    
    
    //create a Anew local/remote adaptor
    RemotePushSenderImpl<T> * p_rpsi = new RemotePushSenderImpl<T>(_connectionID);
    p_rpsi->setTranslator(m_pTranslator);
    p_rpsi->setPushConnector(_rpc);
    

    PushSender<T> *p_psp = dynamic_cast<PushSender<T>*>(_pSender);
    //PushSender<T> *p_psp = (PushSender<T>*)(_pSender);
    
    if(!p_psp) {
      std::ostringstream outStream;
      outStream<<"ERROR: Cannot cast to PushSender for remote connection: "<<_pSender->getProcessIdentifier()<<" id:"<<_connectionID<<" typeid:"<<typeid(T).name()<<std::endl;
      outStream<<typeid(*_pSender).name()<<std::endl;
      throw BALTException(__HERE__,
			  outStream.str().c_str());
      return NULL;
    }
    
    p_psp->setPushConnector(_connectionID,p_rpsi);



    
    return p_rpsi;
    
  }
  
  virtual POA_RemoteConnectors::RemotePushReceiver * connectAsReceiver(FrameworkProcess * _pReceiver, 
									 RemoteConnectors::RemotePushConnector_ptr _rpc, 
									 const std::string & _connectionID)  const {
    
    //START HERE
    RemotePushReceiverImpl<T> *p_rpri = new RemotePushReceiverImpl<T>(_connectionID);
    p_rpri->setTranslator(m_pTranslator);
    
    CORBA::Object_var ob = LocalConnectionManager::getORB()->resolve_initial_references("RootPOA");
    PortableServer::POA_var poa = PortableServer::POA::_narrow(ob);
    
    
    PortableServer::ObjectId_var rprid = poa->activate_object(p_rpri);
    ob = p_rpri->_this();
    
    PortableServer::POAManager_var pman = poa->the_POAManager();
    pman->activate();
    
    PushReceiver<T> *p_prp = dynamic_cast<PushReceiver<T>*>(_pReceiver);
    if(!p_prp) {
      std::ostringstream outStream;
      outStream<<"ERROR: Cannot cast to PushReceiver  for remote connection: "<<_pReceiver->getProcessIdentifier()<<std::endl;
      outStream<<typeid(*_pReceiver).name()<<std::endl;
      throw BALTException(__HERE__,
			  outStream.str().c_str());

      return NULL;
    }
    
    _rpc->registerPushReceiver(p_rpri->_this());
    p_rpri->registerPushReceiver(p_prp);
    
    return p_rpri;
  }

    virtual POA_RemoteConnectors::RemotePullSender * connectAsSender(FrameworkProcess * _pSender, 
								     RemoteConnectors::RemotePullConnector_ptr _rpc,
								     const std::string & _connectionID)  const {


      //create a Anew local/remote adaptor
      RemotePullSenderImpl<T> * p_rpsi = new RemotePullSenderImpl<T>(_connectionID);
      p_rpsi->setTranslator(m_pTranslator);
      p_rpsi->setPullConnector(_rpc);
      
      
      PullSender<T> *p_psp = dynamic_cast<PullSender<T>*>(_pSender);
      if(!p_psp) {
	std::ostringstream outStream;
	
        outStream<<"ERROR: Cannot cast to PullSender for remote connection: "<<_pSender->getProcessIdentifier()<<std::endl;
	outStream<<typeid(*_pSender).name()<<std::endl;
	throw BALTException(__HERE__,
			    outStream.str().c_str());

        return NULL;
      }
      
      p_psp->setPullConnector(_connectionID,p_rpsi);
      return p_rpsi;
    }

    
    virtual POA_RemoteConnectors::RemotePullReceiver * connectAsReceiver(FrameworkProcess * _pReceiver, 
									 RemoteConnectors::RemotePullConnector_ptr _rpc, 
									 const std::string & _connectionID)  const {


      RemotePullReceiverImpl<T> *p_rpri = new RemotePullReceiverImpl<T>(_connectionID);
      p_rpri->setTranslator(m_pTranslator);
      
      CORBA::Object_var ob = LocalConnectionManager::getORB()->resolve_initial_references("RootPOA");
      PortableServer::POA_var poa = PortableServer::POA::_narrow(ob);
      
      
      PortableServer::ObjectId_var rprid = poa->activate_object(p_rpri);
      ob = p_rpri->_this();
      
      PortableServer::POAManager_var pman = poa->the_POAManager();
      pman->activate();
      
      PullReceiver<T> *p_prp = dynamic_cast<PullReceiver<T>*>(_pReceiver);
      
      if(!p_prp) {
	std::ostringstream outStream;

	outStream<<"ERROR: Cannot cast to PullReceiver for remote connection: "<<_pReceiver->getProcessIdentifier()<<std::endl;
	outStream<<typeid(*_pReceiver).name()<<std::endl;
	throw BALTException(__HERE__,
			    outStream.str().c_str());
	return NULL;
      }
            
      _rpc->registerPullReceiver(p_rpri->_this());
      p_rpri->registerPullReceiver(p_prp);
      
      return p_rpri;
    }

 private:
    RemoteDataTranslator<T> * m_pTranslator;


};

#endif

