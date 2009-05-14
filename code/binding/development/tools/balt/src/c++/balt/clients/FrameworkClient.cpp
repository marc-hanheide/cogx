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

#include "FrameworkBasics.hpph"
#include "interface/BALTInterface.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;


#define SERVER_PREFIX "FrameworkProcessManager:"

void getFrameworkHostnames(const FrameworkBasics::ConnectionGraph &_connections,
			   vector<string> &_hostnames) {

  cout<<"framework hosts:";

  string hostname;
  
  vector<string>::iterator p;

  FrameworkBasics::DescriptionList dlist;

  for (unsigned int i = 0; i < _connections.length(); i++) {

    dlist = _connections[i].m_senders;

    for (unsigned int j = 0; j < dlist.length(); j++) {

      hostname = dlist[j].m_hostName;
      
      p = find(_hostnames.begin(),_hostnames.end(),hostname);
      
      if (p == _hostnames.end()) {
	_hostnames.push_back(hostname);
	cout<<" "<<hostname;
      }
      
    }


    dlist = _connections[i].m_receivers;
    
    for (unsigned int j = 0; j < dlist.length(); j++) {

      hostname = dlist[j].m_hostName;
      
      p = find(_hostnames.begin(),_hostnames.end(),hostname);
      
      if (p == _hostnames.end()) {
	_hostnames.push_back(hostname);
	cout<<" "<<hostname;
      }
      
    }

  }

  cout<<endl;

}


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

/// \todo better checking!
bool areSameHost(const string & _hostA, const string & _hostB) {
  return _hostA == _hostB;
}

void printServers() {

  CosNaming::BindingList_var bl;
  CosNaming::BindingIterator_var bi;

  LocalConnectionManager::getNamingContext()->list( 0, bl, bi);

  CosNaming::Binding_var entry;


  while ( getEntry( bi, entry ) == 0 ) {
    for (unsigned int i=0; i<entry->binding_name.length(); i++) {
      
      cout << "\t"
	   << entry->binding_name[i].id
	   << endl;
	
    }
  }
  
}




FrameworkBasics::FrameworkProcessManager_var 
getProcessManager(const CosNaming::NamingContext_var _namingContext,
		  const string &_hostname) {


  CosNaming::BindingList_var bl;
  CosNaming::BindingIterator_var bi;

  _namingContext->list( 0, bl, bi);

  CosNaming::Binding_var entry;

  string objName;
  string serverHost;
  string::size_type prefixPosition;

  string serverPrefix = SERVER_PREFIX;

  while ( getEntry( bi, entry ) == 0 ) {
    for (unsigned int i=0; i<entry->binding_name.length(); i++) {
      
      objName = entry->binding_name[i].id;
      prefixPosition = objName.find(serverPrefix,0);
      if(prefixPosition != string::npos) {
	if(prefixPosition == 0) {
	  //cout<<"found a server: "<<objName<<endl;
	  serverHost = objName.substr(serverPrefix.length(),objName.length() - serverPrefix.length());
	  //cout<<"host: \""<<serverHost<<"\""<<endl;
	  if(areSameHost(_hostname,serverHost)) {
	    cout<<"found server: "<<objName<<endl;
	
	    CosNaming::Name name;
	    name.length(1);
	    name[0].id = CORBA::string_dup(objName.c_str());
	    name[0].kind = CORBA::string_dup("");

	    //must free this up later
	    CORBA::Object_ptr obj = _namingContext->resolve(name);
	    
	    if(CORBA::is_nil(obj)) {
	      cerr << "ERROR: Unable to resolve remote server: " <<objName<< endl;
	      std::abort();
	    } else {
	      //cout << "Name resolved" << endl;
	      return FrameworkBasics::FrameworkProcessManager::_narrow(obj);
	    }
	  }
	}
      }
    }
  }
  

  return FrameworkBasics::FrameworkProcessManager::_nil();

}



void getProcessManagers(const CosNaming::NamingContext_var _namingContext,
			const vector<string> &_hostnames,
			vector<FrameworkBasics::FrameworkProcessManager_var> &_processManagers) {

  for(vector<string>::const_iterator i = _hostnames.begin();
      i != _hostnames.end();
      ++i) {
    
    FrameworkBasics::FrameworkProcessManager_var fpm
      = getProcessManager(_namingContext,*i);
    
    if(fpm == FrameworkBasics::FrameworkProcessManager::_nil()) {
      cerr<<"Unable to find a process manager running on host: "<<*i<<endl;
      std::abort();
    }
    else {
      _processManagers.push_back(fpm);
    }
  }
  

}

int main( int argc, const char* argv[] ) {

    if(argc == 2) {
     cout<<"naming host: "<<argv[1]<<endl;

    FrameworkBasics::ProcessDescription proc1Desc;
    proc1Desc.m_processName = CORBA::string_dup("Push Sender 1");
    //proc1Desc.m_className = CORBA::string_dup("MaxiPushSenderProcess");
    //proc1Desc.m_language = FrameworkBasics::CPP_PROCESS;
    proc1Desc.m_className = CORBA::string_dup("examples.PushSenderProcess");
    proc1Desc.m_language = FrameworkBasics::JAVA_PROCESS;
    proc1Desc.m_hostName = CORBA::string_dup("waterhouse");
    proc1Desc.m_configurationInfo = CORBA::string_dup("dummy");
    
    FrameworkBasics::ProcessDescription proc2Desc;
    proc2Desc.m_processName = CORBA::string_dup("Push Receiver 1");
    //proc2Desc.m_className = CORBA::string_dup("MaxiPushReceiverProcess");
    //proc2Desc.m_language = FrameworkBasics::CPP_PROCESS;
    proc2Desc.m_className = CORBA::string_dup("examples.PushReceiverProcess");
    proc2Desc.m_language = FrameworkBasics::JAVA_PROCESS;
    proc2Desc.m_hostName = CORBA::string_dup("waterhouse");
    proc2Desc.m_configurationInfo = CORBA::string_dup("dummy");
    
    FrameworkBasics::ConnectionGraph connections;
    connections.length(4);


    FrameworkBasics::DescriptionList senders;
    FrameworkBasics::DescriptionList receivers;
    
    senders.length(1);
    senders[0] = proc1Desc;

    receivers.length(1);
    receivers[0] = proc2Desc;

    connections[0].m_senders = senders;
    connections[0].m_receivers = receivers;
    connections[0].m_connectionType = FrameworkBasics::PUSH_CONNECTION;
    connections[0].m_dataType = CORBA::string_dup("String");
    connections[0].m_connectionID = CORBA::string_dup("String Push Connection");

    connections[1].m_senders = senders;
    connections[1].m_receivers = receivers;
    connections[1].m_connectionType = FrameworkBasics::PUSH_CONNECTION;
    connections[1].m_dataType = CORBA::string_dup("Int");
    connections[1].m_connectionID = CORBA::string_dup("Int Push Connection");

    connections[2].m_senders = senders;
    connections[2].m_receivers = receivers;
    connections[2].m_connectionType = FrameworkBasics::PUSH_CONNECTION;
    connections[2].m_dataType = CORBA::string_dup("Float");
    connections[2].m_connectionID = CORBA::string_dup("Float Push Connection");

    connections[3].m_senders = senders;
    connections[3].m_receivers = receivers;
    connections[3].m_connectionType = FrameworkBasics::PUSH_CONNECTION;
    connections[3].m_dataType = CORBA::string_dup("CharSeq");
    connections[3].m_connectionID = CORBA::string_dup("CharSeq Push Connection");


    //use the local connection manager's orb.
    LocalConnectionManager lcm(argv[1],"1050");
    
    vector<string> hostnames;
    getFrameworkHostnames(connections,hostnames);

    vector<FrameworkBasics::FrameworkProcessManager_var> processManagers;

    getProcessManagers(LocalConnectionManager::getNamingContext(),
		       hostnames,
		       processManagers);
    

    
    // create the graph processes
    for (unsigned int i = 0; i < processManagers.size(); ++i) {
      processManagers[i]->createGraph(connections);
    }

    // create any remote connection objects
    for (unsigned int i = 0; i < processManagers.size(); ++i) {
      processManagers[i]->createRemoteConnections();
    }

    // sleep to give all the connections a chance to start up
    // (unnecessary?)
    sleep(2);
    
    // connect all the objects locally and remotely
    for (unsigned int i = 0; i < processManagers.size(); ++i) {
      processManagers[i]->connectGraph();
    }
    
    // start the processes running
    for (unsigned int i = 0; i < processManagers.size(); ++i) {
      processManagers[i]->startGraph();
    }
    
    // let the processes do something for a while
    sleep(5);
 
    // kill off the processes
    for (unsigned int i = 0; i < processManagers.size(); ++i) {
      processManagers[i]->stopGraph();
    }
    

  }
  else {
    cerr<<"naming host not specified"<<endl;
    cerr<<"usage: "<<argv[0]<<" <naming host>"<<endl;
  }

  return 0;
}
