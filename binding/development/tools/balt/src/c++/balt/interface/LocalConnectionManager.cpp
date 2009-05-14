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

#include "LocalConnectionManager.hpp"

#include "RemoteConnectionCreator.hpp"
#include "RemoteDatatypeManager.hpp"

#include "RemotePushConnectors.hpp"
#include "RemotePullConnectors.hpp"
#include <balt/core/BALTException.hpp>

using namespace std;

CORBA::ORB_var LocalConnectionManager::m_orb;
CosNaming::NamingContext_var LocalConnectionManager::m_namingContext;

int getEntry( CosNaming::BindingIterator_var& bi,
              CosNaming::Binding_var& entry ) {
    try {
        if (bi->next_one( entry ) == true)
            return 0;
        else
            return -1;
    } catch (...) {
        return -1;
    }

}

LocalConnectionManager::LocalConnectionManager() {
  LocalConnectionManager("localhost","1050");
}

LocalConnectionManager::LocalConnectionManager(const string &_namingHost,
        const string &_namingPort) {

  try {
	

    //m_pConnectors = new vector<PushConnectorImpl*>();

    m_pProcesses = new ProcessMap();
    m_pConnectors = new ConnectorVector();

    string nameService = "NameService=corbaname::"
                         + _namingHost
                         + ":"
                         + _namingPort;

    //cout<<"InitRef = "<<nameService<<endl;

    const char* options[][2] = {
                                   { "InitRef",
                                     nameService.c_str()
                                   }
                                   , { 0, 0 }
                               };

    int argc=0;

    //initialise the orb
    m_orb = CORBA::ORB_init(argc, NULL, "omniORB4", options);

    //get naming service
    CORBA::Object_var initServ = m_orb->resolve_initial_references("NameService");
    //std::cerr << "OK" << std::endl;

    if (CORBA::is_nil(initServ)) {
        std::cerr << "Naming service ... NOT FOUND" << std::endl;
    }

    // obtain the naming context
    m_namingContext = CosNaming::NamingContext::_narrow(initServ);

    if (CORBA::is_nil(m_namingContext)) {
        std::cerr << "Naming service ... INVALID OBJECT" << std::endl;
    }

    //std::cout << "Naming service ... OK" << std::endl;

    //printServers();
  } catch (const CORBA::TRANSIENT& e) {
    cerr<<"LocalConnectionManager ERROR: is nameserver running on: "<<_namingHost<<
      " "<<_namingPort<<endl;
    std::abort();
  }


}

LocalConnectionManager::~LocalConnectionManager() {

  //cout<<"LocalConnectionManager::~LocalConnectionManager()"<<endl;
  //cout<<"# processes: "<<m_pProcesses->size()<<endl;

    //TODO Stop and clean up processes ... could be nasty

    //Stop ALL first
    for(ProcessMap::iterator i = m_pProcesses->begin();
            i != m_pProcesses->end(); i++) {
      //cout<<"stopping "<<i->first<<" ... "<<endl;
      cout.flush(); 
      
      stopProcess(i->first);
      //cout<<"OK"<<endl;
    }

    //sleep(5);


    //now clean up connectors 
    for(ConnectorVector::iterator i = m_pConnectors->begin();
	i < m_pConnectors->end(); i++) {
      FrameworkConnector *pFC = *i;
      if(pFC) {
	pFC->stopConnector();
      }
     
    }


    //now clean up
    for(ProcessMap::iterator i = m_pProcesses->begin();
	i != m_pProcesses->end(); i++) {

      //  cout<<i->first<<endl;
      //cout<<i->second->state()<<endl;
      
      //remove from map... actually don't as they're deleted later
      //m_pProcesses->erase(i);
      
      int* rv;
      cout<<"waiting for "<<i->first<<" thread... ";
      
      try {
	//join calls the destuctor too
	i->second->join((void**)&rv);
	cout<<"OK"<<endl;
      } catch (const omni_thread_invalid& e) {
	cout<<" THREAD INVALID... was it started?"<<endl;
      }
      
      //object has been deleted in join call
      i->second = NULL;

    }
    
    
    //delete m_pProcesses;


//   for(vector<PushConnectorImpl*>::iterator i = m_pConnectors->begin();
//       i < m_pConnectors->end(); i++) {

//     //remove from map
//     m_pConnectors->erase(i);

//     //free memory
//     delete (*i);
//   }

   delete m_pProcesses;


   //now clean up connectors 
   for(ConnectorVector::iterator i = m_pConnectors->begin();
       i < m_pConnectors->end(); i++) {
     FrameworkConnector *pFC = *i;
     if(pFC) {
       //cout<<"deleting connector"<<endl;
       delete pFC;
       *i = NULL;
     }
     
   }

   delete m_pConnectors;

   //cout<<"End of constructor"<<endl;
   //xcout.flush();

}




void LocalConnectionManager::createProcess(const string &_className,
					   const string &_procName,
					   map<string,string> &_config) {

    FrameworkProcess *pCreated = NULL;

    const FrameworkProcessCreator * pFPC = FrameworkProcessCreatorManager::processCreator(_className);
    
    if(pFPC) {
      pCreated = pFPC->createNewProcess(_procName);
    } else {
      cerr<<"LocalConnectionManager ERROR: Unknown class name: "<<_className<<endl;
      //std::abort();
      return;
    }


    pCreated->configure(_config);

    (*m_pProcesses)[_procName] = pCreated;


//     if(_className == PUSH_SENDER_NAME) {
//       MaxiPushSenderProcess *pPSP = new MaxiPushSenderProcess(_procName);
//       CORBA::LongSeq ls;
//       ls.length(4);
//       ls[0] = 1;
//       ls[1] = 2;
//       ls[2] = 3;
//       ls[3] = 4;
      
//       pPSP->setData(ls);

//       pPSP->setData(string("Data created by " + _procName));
//       pPSP->setData((float)1.234);
//       pPSP->setData((long)99);
      

//       pCreated = pPSP;
//     } else if(_className == PUSH_RECEIVER_NAME) {
//       pCreated = new MaxiPushReceiverProcess(_procName);
//     } else if(_className == PULL_SENDER_NAME) {
//       pCreated = new MaxiPullSenderProcess(_procName);
//     } else if(_className == PULL_RECEIVER_NAME) {
      
//       MaxiPullReceiverProcess *pPRP = new MaxiPullReceiverProcess(_procName);
//       CORBA::LongSeq ls;
//       ls.length(4);
//       ls[0] = 1;
//       ls[1] = 2;
//       ls[2] = 3;
//       ls[3] = 4;
      
//       pPRP->setData(ls);

//       pPRP->setData(string("Data created by " + _procName));
//       pPRP->setData((float)1.234);
//       pPRP->setData((long)99);


//       pCreated = pPRP;


    
}




void LocalConnectionManager::startProcess(const string &_procName) {

  ProcessMap::iterator i = m_pProcesses->find(_procName);

    if(i != m_pProcesses->end()) {
        //cout<<"before start"<<endl;
      i->second->start();
      //cout<<"after start"<<endl;
    } 
    else {
      throw(BALTException(__HERE__,
			  "LocalConnectionManager ERROR: Unknown process to start: %s",
			  _procName.c_str()));
    }

}

void LocalConnectionManager::runProcess(const string &_procName) {

  ProcessMap::iterator i = m_pProcesses->find(_procName);

    if(i != m_pProcesses->end()) {
         //cout<<"before run"<<endl;
      i->second->run();
      //cout<<"after run"<<endl;
    }
    else {
      throw(BALTException(__HERE__,
			  "LocalConnectionManager ERROR: Unknown process to run: %s",
			  _procName.c_str()));
    }

}

void LocalConnectionManager::stopProcess(const string &_procName) {

    ProcessMap::iterator i = m_pProcesses->find(_procName);

    //stop processing
    if(i != m_pProcesses->end()) {
        i->second->stop();
    } 
    else {
      throw(BALTException(__HERE__,
			  "LocalConnectionManager ERROR: Unknown process to stop: %s",
			  _procName.c_str()));
    }


}




void LocalConnectionManager::createPushConnection(
						  const vector<string> &_senderNames,
						  const vector<string> &_receiverNames,
						  const string &_dataType,
						  const string &_connectionID) {


  //cout<<"createPushConnection: "<<_dataType<<" "<<_connectionID<<endl;

  //obtain pointers to processes
  vector<FrameworkProcess *> pSenders;
  vector<FrameworkProcess *> pReceivers;
  
  ProcessMap::iterator j;
  
  for(vector<string>::const_iterator i = _senderNames.begin();
      i < _senderNames.end(); i++) {

    j = m_pProcesses->find(*i);
    if(j != m_pProcesses->end()) {
      pSenders.push_back(j->second);
    } else {
      cerr<<"LocalConnectionManager ERROR: Unknown push send process: "<<*i<<endl;
      return;
    }
    
  }

  for(vector<string>::const_iterator i = _receiverNames.begin();
      i < _receiverNames.end(); i++) {

    j = m_pProcesses->find(*i);
    if(j != m_pProcesses->end()) {
      pReceivers.push_back(j->second);
    } else {
      cerr<<"LocalConnectionManager ERROR: Unknown push receive process: "<<*i<<endl;
      return;
    }
  } 
 
  const LocalConnectionCreator * lcc = LocalDatatypeManager::connectionCreator(_dataType);
  
  if(lcc) {
    FrameworkConnector *pFC = 
      lcc->createPushConnection(pSenders,pReceivers,_connectionID);
    m_pConnectors->push_back(pFC);
  }

  //cout<<"createPushConnection END "<<endl;
}

void LocalConnectionManager::createPullConnection(
        const vector<string> &_senderNames,
        const string &_receiverName,
        const string &_dataType,
        const string &_connectionID) {

    //obtain pointers to processes
  vector<FrameworkProcess *> pSenders;
  FrameworkProcess * pReceiver = NULL;
  
  ProcessMap::iterator j;

  for(vector<string>::const_iterator i = _senderNames.begin();
      i < _senderNames.end(); i++) {

    j = m_pProcesses->find(*i);
    if(j != m_pProcesses->end()) {
      pSenders.push_back(j->second);
    } else {
      cerr<<"LocalConnectionManager ERROR: Unknown pull send process: "<<*i<<endl;
      return;
    }
    
  }
  
  j = m_pProcesses->find(_receiverName);
  if(j != m_pProcesses->end()) {
    pReceiver = j->second;
  } else {
    cerr<<"LocalConnectionManager ERROR: Unknown pull receive process: "<<_receiverName<<endl;
    return;
  }
  
  const LocalConnectionCreator * lcc = LocalDatatypeManager::connectionCreator(_dataType);

  if(lcc) {
    FrameworkConnector *pFC = 
      lcc->createPullConnection(pSenders,pReceiver,_connectionID);
    m_pConnectors->push_back(pFC);
  }


}



void LocalConnectionManager::connectRemotePushConnectionSender(const string &_senderName,
							       const string &_remoteID, 
							       const string &_dataType,
							       const string &_connectionID) {

  //cout<<"Remote connect as sender: "<<_senderName<<" "<<_remoteID<<endl;


    ProcessMap::iterator i = m_pProcesses->find(_senderName);
    //stop processing
    if(i == m_pProcesses->end()) {
        cerr<<"LocalConnectionManager ERROR: Unknown process to connect remotely as sender: "<<_senderName<<endl;
        return;
    }


    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(_remoteID.c_str());
    name[0].kind = CORBA::string_dup("");

    //must free this up later
    CORBA::Object_ptr obj = m_namingContext->resolve( name );
    if(CORBA::is_nil(obj)) {
        cerr << "Unable to resolve remote server: " <<_remoteID<< endl;
    } else {
        //cout << "Name resolved" << endl;
    }

    RemoteConnectors::RemotePushConnector_ptr rpc = RemoteConnectors::RemotePushConnector::_narrow(obj);

    const RemoteConnectionCreator *rcc = RemoteDatatypeManager::connectionCreator(_dataType);
    rcc->connectAsSender(i->second, rpc,_connectionID);
    
}


void LocalConnectionManager::connectRemotePushConnectionReceiver(const string &_receiverName,
								 const string &_remoteID, 
								 const string &_dataType,
								 const string &_connectionID) {
  //    cout<<"Remote connect as receiver: "<<_receiverName<<" "<<_remoteID<<endl;


    ProcessMap::iterator i = m_pProcesses->find(_receiverName);
    //stop processing
    if(i == m_pProcesses->end()) {
        cerr<<"LocalConnectionManager ERROR: Unknown process to connect remotely as receiver: "<<_receiverName<<endl;
        return;
    }


    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(_remoteID.c_str());
    name[0].kind = CORBA::string_dup("");


    //must free this up later
    CORBA::Object_ptr obj = m_namingContext->resolve( name );
    if(CORBA::is_nil(obj)) {
        cerr << "Unable to resolve remote server: " <<_remoteID<< endl;
    } else {
        //cout << "Name resolved" << endl;
    }

    RemoteConnectors::RemotePushConnector_ptr rpc = RemoteConnectors::RemotePushConnector::_narrow(obj);
    
    const RemoteConnectionCreator *rcc = RemoteDatatypeManager::connectionCreator(_dataType);
    rcc->connectAsReceiver(i->second, rpc, _connectionID);

}


void LocalConnectionManager::printServers() {

    CosNaming::BindingList_var bl;
    CosNaming::BindingIterator_var bi;

    m_namingContext->list( 0, bl, bi);

    CosNaming::Binding_var entry;

    while ( getEntry( bi, entry ) == 0 ) {
        for (unsigned int i=0; i<entry->binding_name.length(); i++) {
            cout << "\t"
            << entry->binding_name[i].id
            << endl;
        }
    }


}


void LocalConnectionManager::connectRemotePullConnectionSender(const string &_senderName,
							       const string &_remoteID, 
							       const string &_dataType,
							       const string &_connectionID) {

    //cout<<"Remote connect as pull sender: "<<_senderName<<" "<<_remoteID<<endl;


    ProcessMap::iterator i = m_pProcesses->find(_senderName);
    //stop processing
    if(i == m_pProcesses->end()) {
        cerr<<"LocalConnectionManager ERROR: Unknown process to connect remotely as sender: "<<_senderName<<endl;
        return;
    }


    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(_remoteID.c_str());
    name[0].kind = CORBA::string_dup("");


    //must free this up later
    CORBA::Object_ptr obj = m_namingContext->resolve( name );
    if(CORBA::is_nil(obj)) {
        cerr << "Unable to resolve remote server: " <<_remoteID<< endl;
    } else {
        //cout << "Name resolved" << endl;
    }


    RemoteConnectors::RemotePullConnector_ptr rpc = RemoteConnectors::RemotePullConnector::_narrow(obj);


    const RemoteConnectionCreator *rcc = RemoteDatatypeManager::connectionCreator(_dataType);
    rcc->connectAsSender(i->second, rpc, _connectionID);


}


void LocalConnectionManager::connectRemotePullConnectionReceiver(const string &_receiverName,
								 const string &_remoteID, 
								 const string &_dataType,
								 const string &_connectionID) {
    //cout<<"Remote connect as pull receiver: "<<_receiverName<<" "<<_remoteID<<endl;


    ProcessMap::iterator i = m_pProcesses->find(_receiverName);
    //stop processing
    if(i == m_pProcesses->end()) {
        cerr<<"LocalConnectionManager ERROR: Unknown process to connect remotely as receiver: "<<_receiverName<<endl;
        return;
    }


    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(_remoteID.c_str());
    name[0].kind = CORBA::string_dup("");


    //must free this up later
    CORBA::Object_ptr obj = m_namingContext->resolve( name );
    if(CORBA::is_nil(obj)) {
        cerr << "Unable to resolve remote server: " <<_remoteID<< endl;
    } else {
        //cout << "Name resolved" << endl;
    }

    RemoteConnectors::RemotePullConnector_ptr rpc = RemoteConnectors::RemotePullConnector::_narrow(obj);


    const RemoteConnectionCreator *rcc = RemoteDatatypeManager::connectionCreator(_dataType);
    rcc->connectAsReceiver(i->second, rpc, _connectionID);


}
