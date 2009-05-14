/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes, Henrik Jacobsson
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

#include "SubarchitectureWorkingMemory.hpp"
#include <cast/core/ArchitectureConfiguration.hpp>
#include <cast/core/CASTWorkingMemory.hpp>

#include <cast/core/CASTUtils.hpp>

using namespace std;

using namespace boost;

namespace cast {

  using namespace cdl;

  SubarchitectureWorkingMemory::SubarchitectureWorkingMemory(const string &_id) 
    : CASTProcessingComponent(_id),
      m_wmChangeFiltering(true),
      m_wmDistributedFiltering(true),
      m_xarchWMCFPushConnector(NULL) {

    //println("created wm");

    // determines whether this wm should broadcast to oher
    // sub-architectures
    setSendXarchChangeNotifications(false);


  }

  SubarchitectureWorkingMemory::~SubarchitectureWorkingMemory() {

  }


  void SubarchitectureWorkingMemory::setPushConnector(const string & _connectionID, 
						      PushConnectorOut<cdl::WorkingMemoryChange> * _pOut) {

    //println("cid: " + _connectionID);

    // if it is a connection elsewhere in the architecture
    if (_connectionID.substr(0,ArchitectureConfiguration::XARCH_PREFIX.length()) == ArchitectureConfiguration::XARCH_PREFIX) {

      //HACK: this is really rather ugly
      string wmid = _connectionID.substr(
					 ArchitectureConfiguration::XARCH_PREFIX.length() + 1,
					 (_connectionID.length() - string(":wmchanges").length() - ArchitectureConfiguration::XARCH_PREFIX.length() - 1));
      //println("xa");
      //println(wmid);
      m_xarchChangeConnections[wmid] = _pOut;
    }
    else {
      //println("lo");
      m_wmChangeBroadcast.push_back(_pOut);
    }

  }
		
  string SubarchitectureWorkingMemory::subarchitectureFromID(const string & _id) {
    return _id.substr(0, _id.find(':'));
  }	

  void SubarchitectureWorkingMemory::setPullConnector(const string & _connectionID, 
						      PullConnectorOut<cdl::WorkingMemoryEntryList> * _pOut) {

    if (_connectionID.substr(0,ArchitectureConfiguration::XARCH_PREFIX.length()) == ArchitectureConfiguration::XARCH_PREFIX) {
      //println(_connectionID);
      string subarch = subarchitectureFromID(_connectionID.substr(ArchitectureConfiguration::XARCH_PREFIX.length()+1));
      m_xarchWMPullConnections[subarch] = _pOut;
    }
 
  }

  void SubarchitectureWorkingMemory::setPushConnector(const string & _connectionID, 
						      PushConnectorOut<cdl::WorkingMemoryEntry> * _pOut) {

    if (_connectionID.substr(0,ArchitectureConfiguration::XARCH_PREFIX.length()) == ArchitectureConfiguration::XARCH_PREFIX) {
      //println(_connectionID);
      string subarch = subarchitectureFromID(_connectionID.substr(ArchitectureConfiguration::XARCH_PREFIX.length()+1));
      m_xarchWMPushConnections[subarch] = _pOut;
    }
 
  }




 

  void 
  SubarchitectureWorkingMemory::addComponentChangeFilter(const string &_src,
							 const cdl::WorkingMemoryChangeFilter &_filter) {

    debug("SubarchitectureWorkingMemory::addComponentChangeFilter()");
    debug(_src);
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());
    m_componentFilters[_filter].push_back(_src);
    //cout<<"new filters length: "<<m_componentFilters.size()<<endl;
    //cout<<"only local: "<<m_componentFilters.localFiltersOnly()<<endl;

  }


  void 
  SubarchitectureWorkingMemory::deleteComponentChangeFilter(const string &_src,
							    const cdl::WorkingMemoryChangeFilter &_filter) {
    debug("SubarchitectureWorkingMemory::deleteComponentChangeFilter()");
    debug(_src);
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());  
    vector<string> removed;
    m_componentFilters.remove(_filter, removed);
    //cout<<"new filters length: "<<m_componentFilters.size()<<endl;
    //cout<<"only local: "<<m_componentFilters.localFiltersOnly()<<endl;

  }

  void 
  SubarchitectureWorkingMemory::addWMChangeFilter(const string &_src,
						  const cdl::WorkingMemoryChangeFilter &_filter) {

    debug("SubarchitectureWorkingMemory::addWMChangeFilter()");
    debug(_src);
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());
    m_wmFilters[_filter].push_back(_src);
    //cout<<"new filters length: "<<m_wmFilters.size()<<endl;
    //cout<<"only local: "<<m_wmFilters.localFiltersOnly()<<endl;

  }


  void 
  SubarchitectureWorkingMemory::deleteWMChangeFilter(const string &_src,
						     const cdl::WorkingMemoryChangeFilter &_filter) {
    debug("SubarchitectureWorkingMemory::deleteWMChangeFilter()");
    debug(_src);
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());  
    vector<string> removed;
    m_wmFilters.remove(_filter, removed);
    //cout<<"new filters length: "<<m_wmFilters.size()<<endl;
    //cout<<"only local: "<<m_wmFilters.localFiltersOnly()<<endl;

  }


					       
  void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChangeFilter> *_pData) {
    //println("SubarchitectureWorkingMemory::receivePushData()");

    const cdl::WorkingMemoryChangeFilter & filter = _pData->getData();

    assert(filter.m_filterChange != cdl::OVERWRITE);
    string src(_pData->getSource());

   
    // ignore our own filters
    if (src != getProcessIdentifier()) {

      //TODO debug
//       debug("SubarchitectureWorkingMemory.receivePushData(): " + src
// 	    + CASTUtils.toString(_data));

      lockProcess();

      if (isWorkingMemoryID(src)) {
	processWorkingMemoryFilter(src, filter);
      } else {
	processComponentFilter(src, filter);
      }

      unlockProcess();
    }

    delete _pData;

  }

  void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<CASTWorkingMemoryEntry> *_pData) {

  
    //println("local wme received");

    //in this case _pData points to the memory created by the original
    //component, although this may not be a good long-term
    //assumption. Either way this will be memory that we have control of
  
    CASTWorkingMemoryEntry *pCWME = _pData->data();
    _pData->data() = NULL;
    delete _pData;

    //cout<<endl;
    //cout<<"SubarchitectureWorkingMemory::receivePushData: data ptr: count: "<<pCWME->item().use_count()<<endl;
    //cout<<endl;


 
    if(m_bDebugOutput) {      
      string op;
      if(pCWME->getOperation() == cdl::ADD) {
	op = "ADD";
      }
      else if(pCWME->getOperation() == cdl::OVERWRITE) {
	op = "OVR";
      }
      else {
	op = "DEL";
      }
      debug("");
      if(pCWME->item()) {
	debug(pCWME->item()->getType());
      }
      debug(pCWME->getAddress().m_id);
      debug(op);    
    }


    performOperation(pCWME);
    delete pCWME;

    //conservative access strategy
    //m_memAccessMutex.unlock();
    //unlockProcess();
    //cout<<"out of operation"<<endl;
    //m_workingMemory.debug();

    //println("local wme dealt with");

  }

  void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryEntry> *_pData) {

    //conservative access strategy
    //m_memAccessMutex.lock();


    //println("wme received");

    //static int cycle = 0;

    //int thisCycle = cycle++;
  

    try {

      //cout<<"++into receivePushData "<<thisCycle<<endl;
      cdl::WorkingMemoryEntry *pWME = _pData->data();
      string type(pWME->m_type);

      if(m_bDebugOutput) {
      
	string op;
	if(pWME->m_operation == cdl::ADD) {
	  op = "ADD";
	}
	else if(pWME->m_operation == cdl::OVERWRITE) {
	  op = "OVR";
	}
	else {
	  op = "DEL";
	}

	debug("");
	debug(type);
	debug(pWME->m_address.m_id);
	debug(op);
      
      }
    
      CASTWorkingMemoryItem * pWMI 
	= new WorkingMemoryItem<CORBA::Any>(string(pWME->m_address.m_id),
					    type, 					 
					    new CORBA::Any(pWME->m_data));
    
      CASTWorkingMemoryEntry *pCWME 
	= new CASTWorkingMemoryEntry(_pData->getSource(),
				     pWME->m_operation,
				     pWME->m_address,pWMI);

      performOperation(pCWME);

      delete _pData;
      delete pCWME;


      //cout<<"--out of receivePushData RUN "<<thisCycle++<<endl;
  
      //conservative access strategy
      //m_memAccessMutex.unlock();

  
    }
//    catch (exception& e)
//      {
//	println("exception details follow...");
//	cerr << e.what() << endl;
//	std::abort();
 //     }
    catch(CORBA::SystemException& ex) {
      cerr << "Caught a CORBA::" << ex._name() << endl;
    }
    catch(CORBA::Exception& ex) {
      cerr << "Caught CORBA::Exception: " << ex._name() << endl;
    }
    catch(omniORB::fatalException& fe) {
      cerr << "Caught omniORB::fatalException:" << endl;
      cerr << "  file: " << fe.file() << endl;
      cerr << "  line: " << fe.line() << endl;
      cerr << "  mesg: " << fe.errmsg() << endl;
    }
    catch (omni_thread_fatal& e)
      {
	println("omni_thread_fatal exception details follow...");
	cerr << e.error << endl;

	//       if(e.error == EINVAL) {
	// 	cerr<<"EINVAL"<<endl;
	//       }
	//       else if(e.error == EBUSY) {
	// 	cerr<<"EBUSY"<<endl;
	//       }
	//       else if(e.error == EAGAIN) {
	// 	cerr<<"EAGAIN"<<endl;
	//       }
	//       else if(e.error == EDEADLK) {
	// 	cerr<<"EDEADLK"<<endl;
	//       }
	//       else if(e.error == EPERM) {
	// 	cerr<<"EPERM"<<endl;
	//       }
	//       else if(e.error == ENOMEM) {
	// 	cerr<<"ENOMEM"<<endl;
	//       }

	std::abort();
      }
    catch (omni_thread_invalid& e)
      {
	println("omni_thread_invalid exception details follow...");
	//cerr << e.error << endl;
	std::abort();
      }
  
    catch (...) {
      println("exception details follow...");
      cerr << "default exception" << endl;
      std::abort();
    }


    //m_workingMemory.debug();

  
  }

  bool
  SubarchitectureWorkingMemory::isAllowedChange(const cdl::WorkingMemoryChange & _change) const {

    string subarch(_change.m_address.m_subarchitecture);
    StringSet::iterator i = m_ignoreList.find(subarch);
    return i == m_ignoreList.end() && m_componentFilters.allowsChange(_change);
  }
  
  bool
  SubarchitectureWorkingMemory::isAllowedChange(const std::string & _wmid,
						const cdl::WorkingMemoryChange & _change) const {

    vector<string> receivers;
    // get all receivers for this change
    m_wmFilters.get(_change, receivers);
    // if the given id is a recevier, then go ahead

    vector<string>::iterator found = find(receivers.begin(),
					  receivers.end(),
					  _wmid);
    return found != receivers.end();

  }
  


  void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData) {

    //println("received change list from other processes");

    lockProcess();

    // if the filters require external changes, allow them to be
    // forwarded
    if (!m_componentFilters.localFiltersOnly()) {

    
      // if we're allowed to filter here
      if (m_wmChangeFiltering) {
      
	ostringstream outStream;
	outStream<<"forwarding change: "<<*_pData->data();
	debug(outStream.str());


	for(WMChangePushConnectorVector::iterator i = m_wmChangeBroadcast.begin(); 
	    i < m_wmChangeBroadcast.end(); 
	    i++) {
	  (*i)->
	    push(new FrameworkLocalData<cdl::WorkingMemoryChange>(getProcessIdentifier(),
								  *_pData->data()));
	}
	//free the memory created during input
	delete _pData;
      
      }
      else {
      
	for(WMChangePushConnectorVector::iterator i = m_wmChangeBroadcast.begin(); 
	    i < m_wmChangeBroadcast.end(); 
	    i++) {
	  (*i)
	    ->push(new FrameworkLocalData<cdl::WorkingMemoryChange>(getProcessIdentifier(),
								    *_pData->data()));	  
	}	
      
	//free the memory created during input
	delete _pData;
      
      }
    
    }
    else {
      //println("cannot forward");
      delete _pData;
    }

    unlockProcess();



  }


  void SubarchitectureWorkingMemory::performOperation(CASTWorkingMemoryEntry *_pWME) {

    // if it's for this subarchitecture
    if (string(_pWME->getAddress().m_subarchitecture) == m_subarchitectureID) {
      performLocalOperation(_pWME);
    }
    else {
      ostringstream outStream;
      outStream<<"performing xarch push "<<_pWME->getAddress().m_subarchitecture<<endl;
      debug(outStream.str());
      performXarchOperation(_pWME);
    }

  }

  void SubarchitectureWorkingMemory::performXarchOperation(CASTWorkingMemoryEntry *_pWME) {
    //TODO... reduce redundancy that this introduces


    string subarchitectureID = string(_pWME->getAddress().m_subarchitecture);
    WMEntryPushConnectorMap::iterator i 
      = m_xarchWMPushConnections.find(subarchitectureID);
  
    if(i != m_xarchWMPushConnections.end()) {  
      PushConnectorOut<cdl::WorkingMemoryEntry> * pOut = i->second;
    
      //copy data into new form
      cdl::WorkingMemoryEntry *pWME = new cdl::WorkingMemoryEntry();
      pWME->m_operation = _pWME->getOperation();
      pWME->m_address = _pWME->getAddress();


      if(pWME->m_operation != cdl::DELETE) {
	pWME->m_type = CORBA::string_dup(_pWME->item()->getType().c_str());
	_pWME->item()->toAny(pWME->m_data);
	pWME->m_version = _pWME->item()->getVersion();
      }
      else {
	//cout<<"in the delete fix"<<endl;
	pWME->m_type = CORBA::string_dup("");
	//pWME->m_data = CORBA::Any(); //actually it's probably already set
	pWME->m_version = 0;
      }


      pOut->push(new FrameworkLocalData<cdl::WorkingMemoryEntry>
		 (
		  _pWME->getSource(),
		  pWME));

      //TODO won't always be necessary, how can this behaviour be
      //parameterised?
      pOut->flush();
    
    }
    else {
      println("Unknown subarchitecture: "
	      + subarchitectureID);
    }



  }

  void SubarchitectureWorkingMemory::performLocalOperation(CASTWorkingMemoryEntry *_pWME) {

    //only lock on local input
    lockProcess();

    string src = _pWME->getSource();
    string id(_pWME->getAddress().m_id);
    bool result = false;
    string type;
    //debug("from: "+src);

    if(_pWME->getOperation() == cdl::ADD) {
      //debug("adding");
      //result = addToWorkingMemory(id,type, &(_pWME->m_data));

      type = _pWME->item()->getType();
	    
      logMemoryAdd(src, m_subarchitectureID, id, type);

      result = addToWorkingMemory(id, _pWME->item());

      //     cout<<endl;
      //     cout<<"After add data ptr: count: "<<_pWME->item().use_count()<<endl;
      //     cout<<endl;


    }
    else if(_pWME->getOperation() == cdl::OVERWRITE) {
      //debug("overwriting");
      //result = overwriteWorkingMemory(id,type, &(_pWME->m_data));

      type = _pWME->item()->getType();
    
      logMemoryOverwrite(src, m_subarchitectureID, id, type);

      result = overwriteWorkingMemory(id, _pWME->item(), src);
    }
    else if(_pWME->getOperation() == cdl::DELETE) {
      debug("deleting");
    
      //cout<<"before removal"<<endl;
      //m_workingMemory.debug();
    
      shared_ptr<CASTWorkingMemoryItem> pResult = deleteFromWorkingMemory(id, src);
    
      if(pResult) {

	debug("got result");
      
	//cout<<"after wm removal: "<<pResult.use_count()<<endl;

	//if we have a pointer to what was removed
	result = true;
	type = pResult->getType();

	logMemoryDelete(src, m_subarchitectureID, id, type);
      
	debug("deleted"); 
      }
      else {
	debug("not got result");
	result = false;
      }

    }
  
    logMemorySize(m_workingMemory.size());


    if(result) {
      signalChange(_pWME->getOperation(),src,id,type);
    }
    else {
      println("Working memory operation failed: %i %s %s %s",
	       _pWME->getOperation(), src.c_str(), 
	       id.c_str(), type.c_str());
    }

    unlockProcess();


  }

  bool SubarchitectureWorkingMemory::addToWorkingMemory(const string & _id, 
							shared_ptr<CASTWorkingMemoryItem> _pData) {
    bool result = m_workingMemory.add(_id,_pData);
    if (result) {
      m_permissions.add(_id);
    }
    return result;
  } 

  bool  
  SubarchitectureWorkingMemory::overwriteWorkingMemory(const string & _id, 
						       shared_ptr<CASTWorkingMemoryItem> _pData,
						       const string & _component) {  


    //first sanity check
    if(!m_workingMemory.contains(_id)) {
      //return null
      debug("%s has attempted to overwrite a non-existant entry: %s", 
	    _component.c_str(),_id.c_str());
      return  shared_ptr<CASTWorkingMemoryItem>();
    }


    //sanity check locking elsewhere
    if (m_permissions.isLocked(_id)) {
      const WorkingMemoryPermissions & permissions = 
	m_permissions.getPermissions(_id);

      // if the item is delete locked
      if (!overwriteAllowed(permissions)) {
	// we're going to assume that client checking is up to scratch
	assert (m_permissions.isLockHolder(_id, _component));
      } 
      else {
	//this is never going to happen as all locks are overwrite
	//locks
      }
    }

    bool result = m_workingMemory.overwrite(_id,_pData);

    return result;
  }

  shared_ptr<CASTWorkingMemoryItem>  
  SubarchitectureWorkingMemory::deleteFromWorkingMemory(const string & _id,
							const string & _component) {


    //first sanity check
    if(!m_workingMemory.contains(_id)) {
      //return null
      debug("%s has attempted to deleted a non-existant entry: %s", 
	    _component.c_str(),_id.c_str());
      return  shared_ptr<CASTWorkingMemoryItem>();
    }
    
    bool isLocked = false;

    if (m_permissions.isLocked(_id)) {

      const WorkingMemoryPermissions & permissions = 
	m_permissions.getPermissions(_id);

      // if the item is delete locked
      if (!deleteAllowed(permissions)) {
	// we're going to assume that client checking is up to scratch
	assert (m_permissions.isLockHolder(_id, _component));
      } 
      else {
	// could just be an overwrite lock, in which case it doesn't
	// matter who is the lock holder
      }

      isLocked = true;
    }


    shared_ptr<CASTWorkingMemoryItem> pResult = m_workingMemory.remove(_id);

    if (isLocked) {
      // unlockl on deletion
      debug("unlocking on deletion: %s %s",_id.c_str(),_component.c_str());
      m_permissions.unlock(_id, _component);
    }
   
    debug("unlocking on deletion before permission removal: %s %s",_id.c_str(),_component.c_str());

    //must unlock around remove as it may block
    unlockProcess();
    m_permissions.remove(_id);
    lockProcess();

    debug("unlocking on deletion after permission removal: %s %s",_id.c_str(),_component.c_str());    

    return pResult;
  }


  void SubarchitectureWorkingMemory::signalChange(cdl::WorkingMemoryOperation _op, 
						  const string & _src,
						  const string &  _id,  
						  const string &  _type) {
  
    //   if(m_bLogOutput) {
    
    //     string op;
    //     if(_op == cdl::ADD) {
    //       op = "ADD";
    //     }
    //     else if(_op == cdl::OVERWRITE) {
    //       op = "OVR";
    //     }
    //     else {
    //       op = "DEL";
    //     }
    //     if(_type != "Vision:ImageBufferState") {	  
    //       ostringstream outStream;
    //       outStream<<op<<" "<<BALTTimer::getBALTTime()<<" "<<_src<<" "<<_type<<" "<<_id;	  
    //       debug(outStream.str());      
    //     }  
    //   }


    cdl::WorkingMemoryChange wmc;
    wmc.m_operation = _op;
    wmc.m_src = CORBA::string_dup(_src.c_str());
    wmc.m_address.m_id = CORBA::string_dup(_id.c_str());
    wmc.m_address.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
    wmc.m_type = CORBA::string_dup(_type.c_str());
    
    ostringstream outStream;
    outStream<<"local change: "<<wmc;
    debug(outStream.str());


    FrameworkLocalData<cdl::WorkingMemoryChange> fld(getProcessIdentifier(),wmc);

    //signal change locally
    //println("local change broadcast!!!");
    //if (isAllowedChange(wmc)) {
      WMChangePushConnectorVector::iterator i; 
      for(i = m_wmChangeBroadcast.begin(); 
	  i < m_wmChangeBroadcast.end(); 
	  i++) {
	(*i)
	  ->push(new FrameworkLocalData<cdl::WorkingMemoryChange>(fld));
      }
      //}

    // signal change across sub-architectures
    if (isSendingXarchChangeNotifications()) {
      WMChangePushConnectorMap::iterator i; 
      for(i = m_xarchChangeConnections.begin(); 
	  i != m_xarchChangeConnections.end(); 
	  i++) {
	//println("xarch change broadcast!!!");

	if(isAllowedChange(i->first,wmc)) {
	  i->second->push(new FrameworkLocalData<cdl::WorkingMemoryChange>(fld));	
	}
      }
    }



  }

  AbstractWorkingMemoryPullQuery * SubarchitectureWorkingMemory::parseWorkingMemoryQuery(const FrameworkQuery & _query) {
    return SubarchitectureWorkingMemoryProtocol::parseQuery(_query.getQuery());
  }


  void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query, 
						      FrameworkLocalData<vector<CASTWorkingMemoryEntry *> > *& _pData) {



    //println("received local pull");

    //create the query object
    AbstractWorkingMemoryPullQuery * pQuery 
      = parseWorkingMemoryQuery(_query);
  
    //create result list has to be a pointer/new because the memory is
    //later returned to another component
    vector<CASTWorkingMemoryEntry *> *pWMEL = 
      new vector<CASTWorkingMemoryEntry *>();
  
    handleWorkingMemoryQuery(pQuery,*pWMEL);
  
    //debug("query handled");

    //free up query memory
    delete pQuery;
  
    //vector<CASTWorkingMemoryEntry *> *pList = new vector<CASTWorkingMemoryEntry *>();
 
    //collect the ids first
    //for(unsigned int i = 0; i < pWMEL->size(); i++) {

    //doing this would send the stored copy... dangerous, although it could be consted
    //pList->push_back((*pWMEL)[i]);
    //pList->push_back(new CASTWorkingMemoryEntry(*(*pWMEL)[i]));
    //}
  
    //create data object
    _pData 
      = new FrameworkLocalData<vector<CASTWorkingMemoryEntry *> >(getProcessIdentifier(),
								  pWMEL);  

    //if(_pData) {
    //println("returning non null pData");
    //}


    //logging
    //   if(m_bLogOutput) {
    //     for(unsigned int i = 0; i < pWMEL->size(); i++) {
    //       CASTWorkingMemoryEntry * pCWME = (*pWMEL)[i];
    //       if(pCWME->item()->getType() != "Vision:ImageBufferState") {
    // 	ostringstream outStream;
    // 	outStream<<"GET"<<" "<<BALTTimer::getBALTTime()<<" "<<_query.getSource()<<" "<<pCWME->item()->getType()<<" "<<pCWME->getAddress().m_id;
    // 	debug(outStream.str());
    //       }
    //     }
    //   }

    //m_workingMemory.debug();
  }

  void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query, 
						      FrameworkLocalData<cdl::WorkingMemoryEntryList> *& _pData) {
  

    //println("pull query received");

    //create the query object
    AbstractWorkingMemoryPullQuery * pQuery 
      = parseWorkingMemoryQuery(_query);
  
    //create result list
    //cdl::WorkingMemoryEntryList *pWMEL = new cdl::WorkingMemoryEntryList();
    vector<CASTWorkingMemoryEntry *> wmel;
  
    handleWorkingMemoryQuery(pQuery,wmel);
  
    //debug("query handled");

    //free up query memory
    delete pQuery;
  
    cdl::WorkingMemoryEntryList *pList = new cdl::WorkingMemoryEntryList();
    pList->length(wmel.size());




    for(unsigned int i = 0; i < wmel.size(); i++) {
      CASTWorkingMemoryEntry * pCWME = wmel[i];
      (*pList)[i].m_operation = pCWME->getOperation();
      (*pList)[i].m_address = pCWME->getAddress();
      (*pList)[i].m_type = CORBA::string_dup(pCWME->item()->getType().c_str());
      (*pList)[i].m_version = pCWME->item()->getVersion();    
      //if the data was stored as an any, the any specialisation will
      //ensure the following line does nothing
      pCWME->item()->toAny((*pList)[i].m_data);

      //     if(m_bLogOutput && pCWME->item()->getType() != "Vision:ImageBufferState") {
      //       ostringstream outStream;
      //       outStream<<"GET"<<" "<<BALTTimer::getBALTTime()<<" "<<_query.getSource()<<" "<<pCWME->item()->getType()<<" "<<pCWME->getAddress().m_id;
      //       debug(outStream.str());
      //     }

    }

    //create data object
    _pData 
      = new FrameworkLocalData<cdl::WorkingMemoryEntryList>(getProcessIdentifier(),
							    pList);  

    //if(_pData) {
    //println("returning non null pData");
    //}

    //need to free entry memory
    for(vector<CASTWorkingMemoryEntry *>::iterator i = wmel.begin();
	i < wmel.end(); ++i) {
      delete (*i);
      (*i) = NULL;
    }
  

    //m_workingMemory.debug();

  }

  void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query, 
						      FrameworkLocalData< int > *& _pData) {
  
    //println("count query received");

    string id = _query.getQuery();
    int overwriteCount = -1;
    
    if(m_workingMemory.hasContained(id)) {      
      overwriteCount = m_workingMemory.getOverwriteCount(id);
    }
    else {
      debug("called local: wm has not contained: %s", id.c_str());
    }

    //create data object
    _pData 
      = new FrameworkLocalData< int >(getProcessIdentifier(),
				      new int(overwriteCount));  

    //println("and finished");
  }

  void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query, 
						      FrameworkLocalData< bool > *& _pData) {
  
    //println("exists query received");
  
    string id = _query.getQuery();
    bool exists = m_workingMemory.contains(id);
		 
    //create data object
    _pData 
      = new FrameworkLocalData< bool >(getProcessIdentifier(),
				       new bool(exists));  
  
    //println("and finished");
  }


  void SubarchitectureWorkingMemory::handleWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery, 
							      vector<CASTWorkingMemoryEntry *> & _wmel) {

    if(_pQuery->getSubarchitectureID() != m_subarchitectureID) {
      handleXarchWorkingMemoryQuery(_pQuery,_wmel);
    }
    else {
      handleLocalWorkingMemoryQuery(_pQuery,_wmel);
    }

  }

  void SubarchitectureWorkingMemory::handleLocalWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery, 								 vector<CASTWorkingMemoryEntry *> & _wmel) {


    //only lock local access, as xarch can look after itself
    lockProcess();

    if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID) {
      //println("ID query");
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

      logReadEvent(pQuery->getSourceID(), 
		   pQuery->getSourceSA(), 
		   *(pQuery->getQueryObject()));

      if(pQuery) {
	getEntryByID(pQuery,_wmel);
      }
      else {
	println("ERROR: casting query to ID");
      }

    } 
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::EXISTS) {

      //println("ID query");
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

      //  logReadEvent(pQuery->getSourceID(), 
      // 		 pQuery->getSourceSA(), 
      // 		 *(pQuery->getQueryObject()));


      string queryID = *(pQuery->getQueryObject());
      bool exists = false;
    
      if(m_workingMemory.contains(queryID)) {
	//println("exists TRUE");
	exists = true;
      }
      else {
	// println("exists FALSE");
      }

      shared_ptr< WorkingMemoryItem<bool> > pItem(new WorkingMemoryItem<bool>(queryID,"bool", new bool(exists)));

      // if true

      //make sure list only contains the new item
      _wmel.clear();

      cdl::WorkingMemoryAddress wma;
      wma.m_id = CORBA::string_dup(queryID.c_str());
      wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
    
      _wmel.push_back(new CASTWorkingMemoryEntry(getProcessIdentifier(),
						 cdl::GET,
						 wma,
						 pItem));


      //println("exists query, doing nothing");
    
      //     if(pQuery) {
      //       getEntryByID(pQuery,_wmel);
      //     }
      //     else {
      //       println("ERROR: casting query to ID");
      //     }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::OVERWRITE_COUNT) {

      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);
   
      string queryID = *(pQuery->getQueryObject());
      int overwriteCount = -1;
    
      if(m_workingMemory.hasContained(queryID)) {      
	overwriteCount = m_workingMemory.getOverwriteCount(queryID);
      }
      else {
	debug("called xarch: wm has not contained: %s", queryID.c_str()); 
      }

      shared_ptr< WorkingMemoryItem<int> > pItem(new WorkingMemoryItem<int>(queryID,"int", new int(overwriteCount)));

      // if true
    
      //make sure list only contains the new item
      _wmel.clear();

      cdl::WorkingMemoryAddress wma;
      wma.m_id = CORBA::string_dup(queryID.c_str());
      wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
    
      _wmel.push_back(new CASTWorkingMemoryEntry(getProcessIdentifier(),
						 cdl::GET,
						 wma,
						 pItem));

      //println("exists query, doing nothing");
    
      //     if(pQuery) {
      //       getEntryByID(pQuery,_wmel);
      //     }
      //     else {
      //       println("ERROR: casting query to ID");
      //     }

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID_ARRAY) {
      //println("ID Array query");
//       WorkingMemoryPullQuery< vector<string> > * pQuery = 
// 	dynamic_cast<WorkingMemoryPullQuery< vector<string> > *>(_pQuery);

//       logReadEvent(pQuery->getSourceID(), 
// 		   pQuery->getSourceSA(), 
// 		   "multi string query");

//       if(pQuery) {
// 	getEntriesByID(pQuery,_wmel);
//       }
//       else {
// 	println("ERROR: casting query to ID array");
//       }

      throw(SubarchitectureProcessException(__HERE__, 
					    "ID array query unsupported"));

    }
    else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::TYPE) {
      //println("Type query");
      WorkingMemoryPullQuery<string> * pQuery = 
	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

      logReadEvent(pQuery->getSourceID(), 
		   pQuery->getSourceSA(), 
		   *(pQuery->getQueryObject()), 
		   pQuery->getCount());

      if(pQuery) {
	getEntriesByType(pQuery,_wmel);
      }
      else {
	println("ERROR: casting query to type");
      }

    }
    else {
      println("Unknown query type");
    }

    unlockProcess();


  }

  void SubarchitectureWorkingMemory::handleXarchWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery, 
								   vector<CASTWorkingMemoryEntry *> & _wmel) {
   
    //TODO... reduce redundancy that this introduces

    WMEntryListPullConnectorMap::iterator i 
      = m_xarchWMPullConnections.find(_pQuery->getSubarchitectureID());
  

    //log("using id for remote pull: %s",_pQuery->getSubarchitectureID().c_str());

    if(i != m_xarchWMPullConnections.end()) {
  
      PullConnectorOut<cdl::WorkingMemoryEntryList> * pOut = i->second;

      //wrap the input pointer in a local data structure for pull

      string query = SubarchitectureWorkingMemoryProtocol::createQuery(_pQuery);
      FrameworkQuery fq(getProcessIdentifier(),query);
      FrameworkLocalData<cdl::WorkingMemoryEntryList> *pFLD = NULL;		    
    
      //println("handleXarchWorkingMemoryQuery");
    
      //println("handleXarchWorkingMemoryQuery");
      //println(query);

      pOut->pull(fq,pFLD);

      //println("handleXarchWorkingMemoryQuery after pull");

      if(!pFLD) {
	println("XArch pull returned null");
	//_pCWMEL = NULL;
      }
      else {
      
	//TODO some deleting!
	//       _pWMEL = pFLD->data();
	//       pFLD->data() = NULL;
	//       delete pFLD;

	cdl::WorkingMemoryEntryList * pWMEL = pFLD->data();
	for(unsigned int j = 0; j < pWMEL->length(); j++) {
	
	  //cdl::WorkingMemoryEntry *pWME = _pData->data();
	  string type((*pWMEL)[j].m_type);
	  string id((*pWMEL)[j].m_address.m_id);
	  int version((*pWMEL)[j].m_version);


	  //println("handleXarchWorkingMemoryQuery %s %s", type.c_str(), id.c_str());

	  //FIXME why is this copying here???  what happens to
	  //(*pWMEL)[j].m_data when delete pWMEL is called via delete
	  //pFLD? It's on the heap so it probably goes... hence the
	  //copying.... booo!

	  CASTWorkingMemoryItem * pWMI 
	    = new WorkingMemoryItem<CORBA::Any>(id,
						type, 
						version,
						//this induces a copy
						//and is equivalent to
						//the line below
						(*pWMEL)[j].m_data); 
	  //new CORBA::Any((*pWMEL)[j].m_data));
    
	  CASTWorkingMemoryEntry *pCWME 
	    = new CASTWorkingMemoryEntry(getProcessIdentifier(),
					 (*pWMEL)[j].m_operation,
					 (*pWMEL)[j].m_address,
					 pWMI);

	  _wmel.push_back(pCWME);

	}

	delete pFLD;
	pFLD = NULL;

      }

    }
    else {
      println("Unknown subarchitecture: "
	      + _pQuery->getSubarchitectureID());
    }
   
  }

  void 
  SubarchitectureWorkingMemory::readBlock(const std::string & _id, 
					  const std::string & _component) {

    
    debug("start readBlock: %s %s", _id.c_str(), _component.c_str());

    WorkingMemoryPermissions perms = m_permissions.getPermissions(_id);
    while(m_permissions.isLocked(_id) && 
	  !m_permissions.isLockHolder(_id, _component) &&
	  !readAllowed(perms)) {
	//debug("blocking read by " + _component + " because " + _id
	//    + " is locked by " + m_permissions.getLockHolder(_id));
	// block
	
	// unlock before we block to allow other things to happen
	debug("locking readBlock: %s %s", _id.c_str(), _component.c_str());
	unlockProcess();
	

	//lock entry, this will block 
	m_permissions.lock(_id,_component, cdl::LOCKED_ODR); 

	debug("locked readBlock: %s %s", _id.c_str(), _component.c_str());

	// relock to finish the operation
	lockProcess();

	debug("and inside locked readBlock: %s %s", _id.c_str(), _component.c_str());


	m_permissions.unlock(_id,_component); 
	
	debug("unlocked readBlock: %s %s", _id.c_str(), _component.c_str());	
	
	//now check if entry still exists as it could've been deleted
	//while we block... so we don't even hold a lock
	if(!m_workingMemory.contains(_id)) {
	  debug("deletion during readBlock, returning");
	  return;
	}
	
	//update permissions
	perms = m_permissions.getPermissions(_id);	
    }  
    
    debug("end readBlock: %s %s", _id.c_str(), _component.c_str());
  } 
  
  void SubarchitectureWorkingMemory::getEntryByID(WorkingMemoryPullQuery<string> * _pQuery,
						  vector<CASTWorkingMemoryEntry *> & _wmel) {

    

   
    string id(*(_pQuery->getQueryObject()));



    if(m_workingMemory.contains(id)){

      string component(_pQuery->getSourceID());
      readBlock(id,component);

      //debug("ID query: " + id);
      //debug(id);
      shared_ptr<CASTWorkingMemoryItem> pItem = m_workingMemory.get(id);

      //the item might not be there if something changed during the readBlock
      if(pItem) {

	cdl::WorkingMemoryAddress wma;
	wma.m_id = CORBA::string_dup(id.c_str());
	wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
	
	_wmel.push_back(new CASTWorkingMemoryEntry(getProcessIdentifier(),
						   cdl::GET,
						   wma,
						   pItem));	
      }
    }
    else {
      //debug("ERROR getEntryByID: Unknown request WM ID: " + id);
    }
  
   

  }

//   void SubarchitectureWorkingMemory::getEntriesByID(WorkingMemoryPullQuery< vector<string> > * _pQuery,
// 						    vector<CASTWorkingMemoryEntry *> & _wmel) {


//     //   debug("ID query");

//     //TODO, do this differently

//     vector<string> * pIDs = _pQuery->getQueryObject();

//     int count = 0;

//     //collect the ids first
//     for(vector<string>::iterator i = pIDs->begin();
// 	i < pIDs->end();
// 	i++) {
     

//       shared_ptr<CASTWorkingMemoryItem> pItem = m_workingMemory.get(*i);

//       if(pItem.get()) {
      
// 	cdl::WorkingMemoryAddress wma;
// 	wma.m_id = CORBA::string_dup((*i).c_str());
// 	wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
  
// 	_wmel.push_back(new CASTWorkingMemoryEntry(getProcessIdentifier(),
// 						   cdl::GET,
// 						   wma,
// 						   pItem));      
// 	count++;

// 	if(count == _pQuery->getCount()) {
// 	  break;
// 	}

//       }
//       else {
// 	debug("ERROR getEntriesByID: Unknown request WM ID: " + (*i));
//       }
//     }
   
//   }

  void SubarchitectureWorkingMemory::getEntriesByType(WorkingMemoryPullQuery<string> * _pQuery,
						      vector<CASTWorkingMemoryEntry*> & _wmel) {


    string type = *(_pQuery->getQueryObject());
  
    vector<string> ids;

    m_workingMemory.getIDsByType(type,_pQuery->getCount(), ids);

    string component(_pQuery->getSourceID());

    //collect the ids first
    for(unsigned int i = 0; i < ids.size();  i++) {

      //check whether we need to block
      readBlock(ids[i], component);
      
      shared_ptr<CASTWorkingMemoryItem> pItem
	= m_workingMemory.get(ids[i]);

      //if deletion during read block then things might not be there
      //any more
      if(pItem) {

	cdl::WorkingMemoryAddress wma;
	wma.m_id = CORBA::string_dup(ids[i].c_str());
	wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
	
	_wmel.push_back(new CASTWorkingMemoryEntry(getProcessIdentifier(),
						   cdl::GET,
						   wma,
						   pItem));      
      }
    }
  
  }

  void SubarchitectureWorkingMemory::configure(map<string,string>& _config) {
    CASTProcessingComponent::configure(_config);
    
    if(_config[cdl::IGNORE_SA_KEY] != "") {
      string ignore = _config[cdl::IGNORE_SA_KEY];
      vector<string> ignoreList;
      SubarchitectureWorkingMemoryProtocol::tokenizeString(ignore,
							   ignoreList,
							   ",");
      for(vector<string>::iterator i = ignoreList.begin();
	  i < ignoreList.end();
	  ++i) {
	ignoreChangesFromSubarchitecture(*i);
      }

    }

    buildIDLists(_config);


  }

  void SubarchitectureWorkingMemory::ignoreChangesFromSubarchitecture(const string & _subarch) {
    log("ignoring changes from: %s",_subarch.c_str());
    m_ignoreList.insert(_subarch);
  }

  void SubarchitectureWorkingMemory::setPushConnector(const string & _connectionID, 
						      PushConnectorOut<cdl::WorkingMemoryChangeFilter> * _pOut) {
    assert(m_xarchWMCFPushConnector == NULL);
    m_xarchWMCFPushConnector = _pOut;
  }

  void SubarchitectureWorkingMemory::sendFilter(const cdl::WorkingMemoryChangeFilter & _data)  {
    if(m_xarchWMCFPushConnector != NULL) {
      m_xarchWMCFPushConnector->push(new FrameworkLocalData<cdl::WorkingMemoryChangeFilter>(getProcessIdentifier(), _data));
      
      unlockProcess();
      //it's deadlock-dangerous here
      m_xarchWMCFPushConnector->flush();
      lockProcess();

    }
  }

  void SubarchitectureWorkingMemory::buildIDLists(std::map<std::string,std::string>& _config) {
    string wmIDs = _config[cdl::WM_IDS_KEY];
    assert (wmIDs != "");
    vector<string> ids;
    SubarchitectureWorkingMemoryProtocol::tokenizeString(wmIDs,
							 ids,
							 ",");
    for(vector<string>::iterator id = ids.begin();
	id < ids.end();
	++id) {
      //debug(*id);
      m_wmIDs.insert(*id);
    }      

  }  

  bool SubarchitectureWorkingMemory::isWorkingMemoryID(const std::string & _id) const {    
    StringSet::iterator i = m_wmIDs.find(_id);
    return i != m_wmIDs.end();
  }


  void SubarchitectureWorkingMemory::processWorkingMemoryFilter(const std::string & _src, 
								const cdl::WorkingMemoryChangeFilter & _filter)  {
    if (_filter.m_filterChange == cdl::ADD) {
      addWMChangeFilter(_src, _filter);
    }
    else {
      deleteWMChangeFilter(_src, _filter);
    }
    
  }
  
  void SubarchitectureWorkingMemory::processComponentFilter(const std::string & _src, 
							    const cdl::WorkingMemoryChangeFilter & _filter) {
    if (_filter.m_filterChange == cdl::ADD) {
      addComponentChangeFilter(_src, _filter);
    }
    else {
      deleteComponentChangeFilter(_src, _filter);
    }

    // if we're sharing our filters
    if (m_wmDistributedFiltering) {
      debug("fowarding filter");
      sendFilter(_filter);
    }
    

  }


  void SubarchitectureWorkingMemory::setPullConnector(const string & _connectionID, 
						      PullConnectorOut<cdl::WorkingMemoryPermissions> * _pOut) {

    if (_connectionID.substr(0,ArchitectureConfiguration::XARCH_PREFIX.length()) == ArchitectureConfiguration::XARCH_PREFIX) {
      //debug(_connectionID);
      string subarch = subarchitectureFromID(_connectionID.substr(ArchitectureConfiguration::XARCH_PREFIX.length()+1));
      m_xarchLockConnections[subarch] = _pOut;
    }
 
  }
  
  void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query, 
						      FrameworkLocalData<cdl::WorkingMemoryPermissions> *& _pData) {


    vector<string> tokens;
    SubarchitectureWorkingMemoryProtocol::tokenizeString(_query.getQuery(),
							 tokens,
							 "\\");
    assert (tokens.size() == 3);
    string subarch = tokens[1];

    if (subarch == m_subarchitectureID) {

      lockProcess();

      string id = tokens[0];
      WorkingMemoryLockRequest request = (WorkingMemoryLockRequest) atoi(tokens[2].c_str());
      WorkingMemoryPermissions permissions = handleLocalPermissionsQuery(id, request, _query.getSource());
      
      ostringstream outStream;
      outStream<<permissions;
      debug("SubarchitectureWorkingMemory::receivePullQuery: %s", outStream.str().c_str());

      _pData 
	= new FrameworkLocalData<WorkingMemoryPermissions>(getProcessIdentifier(),
							   permissions);  

      unlockProcess();
    } 
    else {
      
      PermissionsPullConnectorMap::iterator i 
	= m_xarchLockConnections.find(subarch);
      
      debug("forwarding permissions pull across subarch: %s", _query.getQuery().c_str());
      
      assert(i != m_xarchLockConnections.end());
  
      PullConnectorOut<cdl::WorkingMemoryPermissions> * pOut = i->second;

      FrameworkQuery fq(_query);
      pOut->pull(fq,_pData);


    }
    
  }
    
  cdl::WorkingMemoryPermissions
  SubarchitectureWorkingMemory::handleLocalPermissionsQuery(const std::string & _id,
							    const cdl::WorkingMemoryLockRequest & _request, 
							    const std::string & _component) 
    throw(SubarchitectureProcessException) {
    

    // sanity check up front
    if (!m_workingMemory.contains(_id)) {      
      return DOES_NOT_EXIST;
    }

    // if it's a lock
    if (_request < 3) {
      WorkingMemoryPermissions permissions = toPermissions(_request);
      //unlock before trying to lock
      unlockProcess();
      debug("%s locking: %s",_component.c_str(),_id.c_str());
      m_permissions.lock(_id, _component, permissions);

      lockProcess();

      //now check that it still exists, because it could've been
      //deleted before the lock was released
      if (!m_workingMemory.contains(_id)) {
	debug("entry %s deleted while %s waiting for lock", _id.c_str(), _component.c_str());
	m_permissions.unlock(_id, _component);
	permissions = DOES_NOT_EXIST;
      }
      else {	
	assert(m_permissions.isLockHolder(_id,_component));
	assert(m_permissions.getPermissions(_id) == permissions);
	debug("%s locked: ok",_component.c_str());
      }

      return permissions;
    }
    // it's a try-lock
    else if (_request < 6) {
      WorkingMemoryPermissions permissions = toPermissions(_request);

      if (m_permissions.tryLock(_id, _component, permissions)) {	

	debug("%s try locked: %s",_component.c_str(),_id.c_str());	
	assert(m_permissions.isLockHolder(_id,_component));
	assert(m_permissions.getPermissions(_id) == permissions);
	debug("%s try locked: ok",_component.c_str());
	return permissions;
      } 
      else {
	return ALREADY_LOCKED;
      }
      // it's a status request
    } 
    else if (_request == REQUEST_STATUS) {
      WorkingMemoryPermissions permissions = m_permissions.getPermissions(_id);
      ostringstream outStream;
      outStream<<permissions;
      debug("SubarchitectureWorkingMemory::handleLocalPermissionsQuery:REQUEST_STATUS: %s", outStream.str().c_str());
      return permissions;
    }
    // it's an unlock
    else if (_request == REQUEST_UNLOCK) {
      m_permissions.unlock(_id, _component);
      debug("%s unlocked %s",_component.c_str(),_id.c_str());
      return UNLOCKED;
    }

    //default if nothing happens

    throw(SubarchitectureProcessException(__HERE__,
					  "Invalid permission request"));


  }
  

} //namespace cast
