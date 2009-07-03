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

#include <CASTWorkingMemory.hpp>
#include <CASTUtils.hpp>

using namespace std;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::SubarchitectureWorkingMemory();
  }
}

namespace cast {

  using namespace cdl;
  using namespace interfaces;

  SubarchitectureWorkingMemory::SubarchitectureWorkingMemory() :
    m_wmDistributedFiltering(true) {

    setSendXarchChangeNotifications(true);
  }

  SubarchitectureWorkingMemory::~SubarchitectureWorkingMemory() {

  }

  void SubarchitectureWorkingMemory::addReader(
					       const interfaces::WorkingMemoryReaderComponentPrx & _reader,
					       const Ice::Current& _ctx) {
    //only have oneway connections, so now return signal or value
    //TODO see if errors from network are at all likely, and or replace with datagram proxies
    //
    m_readers.push_back(
			interfaces::WorkingMemoryReaderComponentPrx::uncheckedCast(
										   _reader->ice_oneway()));
  }

  //   void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChangeFilter> *_pData) {
  //     //println("SubarchitectureWorkingMemory::receivePushData()");

  //     const cdl::WorkingMemoryChangeFilter & filter = _pData->getData();

  //     assert(filter.m_filterChange != cdl::OVERWRITE);
  //     string src(_pData->getSource());


  //     // ignore our own filters
  //     if (src != getComponentIdentifier()) {

  //       //TODO debug
  // //       debug("SubarchitectureWorkingMemory.receivePushData(): " + src
  // // 	    + CASTUtils.toString(_data));

  //       lockComponent();

  //       if (isWorkingMemoryID(src)) {
  // 	processWorkingMemoryFilter(src, filter);
  //       } else {
  // 	processComponentFilter(src, filter);
  //       }

  //       unlockComponent();
  //     }

  //     delete _pData;

  //   }

  //   void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<CASTWorkingMemoryEntry> *_pData) {


  //     //println("local wme received");

  //     //in this case _pData points to the memory created by the original
  //     //component, although this may not be a good long-term
  //     //assumption. Either way this will be memory that we have control of

  //     CASTWorkingMemoryEntry *pCWME = _pData->data();
  //     _pData->data() = NULL;
  //     delete _pData;

  //     //cout<<endl;
  //     //cout<<"SubarchitectureWorkingMemory::receivePushData: data ptr: count: "<<pCWME->item().use_count()<<endl;
  //     //cout<<endl;


  //     if(m_bDebugOutput) {
  //       string op;
  //       if(pCWME->getOperation() == cdl::ADD) {
  // 	op = "ADD";
  //       }
  //       else if(pCWME->getOperation() == cdl::OVERWRITE) {
  // 	op = "OVR";
  //       }
  //       else {
  // 	op = "DEL";
  //       }
  //       debug("");
  //       if(pCWME->item()) {
  // 	debug(pCWME->item()->getType());
  //       }
  //       debug(pCWME->getAddress().m_id);
  //       debug(op);
  //     }


  //     performOperation(pCWME);
  //     delete pCWME;

  //     //conservative access strategy
  //     //m_memAccessMutex.unlock();
  //     //unlockComponent();
  //     //cout<<"out of operation"<<endl;
  //     //m_workingMemoory.debug();

  //     //println("local wme dealt with");

  //   }

  //   void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryEntry> *_pData) {

  //     //conservative access strategy
  //     //m_memAccessMutex.lock();


  //     //println("wme received");

  //     //static int cycle = 0;

  //     //int thisCycle = cycle++;


  //     try {

  //       //cout<<"++into receivePushData "<<thisCycle<<endl;
  //       cdl::WorkingMemoryEntry *pWME = _pData->data();
  //       string type(pWME->m_type);

  //       if(m_bDebugOutput) {

  // 	string op;
  // 	if(pWME->m_operation == cdl::ADD) {
  // 	  op = "ADD";
  // 	}
  // 	else if(pWME->m_operation == cdl::OVERWRITE) {
  // 	  op = "OVR";
  // 	}
  // 	else {
  // 	  op = "DEL";
  // 	}

  // 	debug("");
  // 	debug(type);
  // 	debug(pWME->m_address.m_id);
  // 	debug(op);

  //       }

  //       CASTWorkingMemoryItem * pWMI
  // 	= new WorkingMemoryItem<CORBA::Any>(string(pWME->m_address.m_id),
  // 					    type,
  // 					    new CORBA::Any(pWME->m_data));

  //       CASTWorkingMemoryEntry *pCWME
  // 	= new CASTWorkingMemoryEntry(_pData->getSource(),
  // 				     pWME->m_operation,
  // 				     pWME->m_address,pWMI);

  //       performOperation(pCWME);

  //       delete _pData;
  //       delete pCWME;


  //       //cout<<"--out of receivePushData RUN "<<thisCycle++<<endl;

  //       //conservative access strategy
  //       //m_memAccessMutex.unlock();


  //     }
  //     catch (std::exception& e)
  //       {
  // 	println("exception details follow...");
  // 	cerr << e.what() << endl;
  // 	std::abort();
  //       }
  //     catch(CORBA::SystemException& ex) {
  //       cerr << "Caught a CORBA::" << ex._name() << endl;
  //     }
  //     catch(CORBA::Exception& ex) {
  //       cerr << "Caught CORBA::Exception: " << ex._name() << endl;
  //     }
  //     catch(omniORB::fatalException& fe) {
  //       cerr << "Caught omniORB::fatalException:" << endl;
  //       cerr << "  file: " << fe.file() << endl;
  //       cerr << "  line: " << fe.line() << endl;
  //       cerr << "  mesg: " << fe.errmsg() << endl;
  //     }
  //     catch (omni_thread_fatal& e)
  //       {
  // 	println("omni_thread_fatal exception details follow...");
  // 	cerr << e.error << endl;

  // 	//       if(e.error == EINVAL) {
  // 	// 	cerr<<"EINVAL"<<endl;
  // 	//       }
  // 	//       else if(e.error == EBUSY) {
  // 	// 	cerr<<"EBUSY"<<endl;
  // 	//       }
  // 	//       else if(e.error == EAGAIN) {
  // 	// 	cerr<<"EAGAIN"<<endl;
  // 	//       }
  // 	//       else if(e.error == EDEADLK) {
  // 	// 	cerr<<"EDEADLK"<<endl;
  // 	//       }
  // 	//       else if(e.error == EPERM) {
  // 	// 	cerr<<"EPERM"<<endl;
  // 	//       }
  // 	//       else if(e.error == ENOMEM) {
  // 	// 	cerr<<"ENOMEM"<<endl;
  // 	//       }

  // 	std::abort();
  //       }
  //     catch (omni_thread_invalid& e)
  //       {
  // 	println("omni_thread_invalid exception details follow...");
  // 	//cerr << e.error << endl;
  // 	std::abort();
  //       }

  //     catch (...) {
  //       println("exception details follow...");
  //       cerr << "default exception" << endl;
  //       std::abort();
  //     }


  //     //m_workingMemory.debug();


  //   }

  bool SubarchitectureWorkingMemory::isAllowedChange(
						     const cdl::WorkingMemoryChange & _change) const {

    string subarch(_change.address.subarchitecture);
    StringSet::iterator i = m_ignoreList.find(subarch);
    return i == m_ignoreList.end() && m_componentFilters.allowsChange(_change);
  }

  bool SubarchitectureWorkingMemory::isAllowedChange(
						     const std::string & _subarch, const cdl::WorkingMemoryChange & _change) const {

    vector<string> receivers;
    // get all receivers for this change

    //     cout<<"isAllowedChange: "<<m_wmFilters.size()<<endl;
    m_wmFilters.get(_change, receivers);
    // if the given id is a recevier, then go ahead

    //     cout<<"isAllowedChange: "<<receivers.size()<<endl;


    vector<string>::iterator found = find(receivers.begin(), receivers.end(),
					  _subarch);

    //     for(vector<string>::const_iterator i = receivers.begin();
    // 	i < receivers.end(); ++i) {

    //       cout<<"isAllowedChange: "<<*i<<endl;
    //       cout<<"isAllowedChange: "<<_subarch<<endl;
    //     }

    //     cout<<"isAllowedChange: "<<(found != receivers.end())<<endl;

    return found != receivers.end();
  }

  //   void SubarchitectureWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData) {

  //     //println("received change list from other processes");

  //     lockComponent();

  //     // if the filters require external changes, allow them to be
  //     // forwarded
  //     if (!componentFilters.localFiltersOnly()) {


  //       // if we're allowed to filter here
  //       if (m_wmChangeFiltering) {

  // 	ostringstream outStream;
  // 	outStream<<"forwarding change: "<<*_pData->data();
  // 	debug(outStream.str());


  // 	for(WMChangePushConnectorVector::iterator i = m_wmChangeBroadcast.begin();
  // 	    i < m_wmChangeBroadcast.end();
  // 	    i++) {
  // 	  (*i)->
  // 	    push(new FrameworkLocalData<cdl::WorkingMemoryChange>(getComponentIdentifier(),
  // 								  *_pData->data()));
  // 	}
  // 	//free the memory created during input
  // 	delete _pData;

  //       }
  //       else {

  // 	for(WMChangePushConnectorVector::iterator i = m_wmChangeBroadcast.begin();
  // 	    i < m_wmChangeBroadcast.end();
  // 	    i++) {
  // 	  (*i)
  // 	    ->push(new FrameworkLocalData<cdl::WorkingMemoryChange>(getComponentIdentifier(),
  // 								    *_pData->data()));
  // 	}

  // 	//free the memory created during input
  // 	delete _pData;

  //       }

  //     }
  //     else {
  //       //println("cannot forward");
  //       delete _pData;
  //     }

  //     unlockComponent();


  //   }


  //   void SubarchitectureWorkingMemory::performOperation(CASTWorkingMemoryEntry *_pWME) {

  //     // if it's for this subarchitecture
  //     if (string(_pWME->getAddress().m_subarchitecture) == m_subarchitectureID) {
  //       performLocalOperation(_pWME);
  //     }
  //     else {
  //       ostringstream outStream;
  //       outStream<<"performing xarch push "<<_pWME->getAddress().m_subarchitecture<<endl;
  //       debug(outStream.str());
  //       performXarchOperation(_pWME);
  //     }

  //   }

  //   void SubarchitectureWorkingMemory::performXarchOperation(CASTWorkingMemoryEntry *_pWME) {
  //     //TODO... reduce redundancy that this introduces


  //     string subarchitectureID = string(_pWME->getAddress().m_subarchitecture);
  //     WMEntryPushConnectorMap::iterator i
  //       = m_xarchWMPushConnections.find(subarchitectureID);

  //     if(i != m_xarchWMPushConnections.end()) {
  //       PushConnectorOut<cdl::WorkingMemoryEntry> * pOut = i->second;

  //       //copy data into new form
  //       cdl::WorkingMemoryEntry *pWME = new cdl::WorkingMemoryEntry();
  //       pWME->m_operation = _pWME->getOperation();
  //       pWME->m_address = _pWME->getAddress();


  //       if(pWME->m_operation != cdl::DELETE) {
  // 	pWME->m_type = CORBA::string_dup(_pWME->item()->getType().c_str());
  // 	_pWME->item()->toAny(pWME->m_data);
  // 	pWME->m_version = _pWME->item()->getVersion();
  //       }
  //       else {
  // 	//cout<<"in the delete fix"<<endl;
  // 	pWME->m_type = CORBA::string_dup("");
  // 	//pWME->m_data = CORBA::Any(); //actually it's probably already set
  // 	pWME->m_version = 0;
  //       }


  //       pOut->push(new FrameworkLocalData<cdl::WorkingMemoryEntry>
  // 		 (
  // 		  _pWME->getSource(),
  // 		  pWME));

  //       //TODO won't always be necessary, how can this behaviour be
  //       //parameterised?
  //       pOut->flush();

  //     }
  //     else {
  //       println("Unknown subarchitecture: "
  // 	      + subarchitectureID);
  //     }


  //   }

  //   void SubarchitectureWorkingMemory::performLocalOperation(CASTWorkingMemoryEntry *_pWME) {

  //     //only lock on local input
  //     lockComponent();

  //     string src = _pWME->getSource();
  //     string id(_pWME->getAddress().m_id);
  //     bool result = false;
  //     string type;
  //     //debug("from: "+src);

  //     if(_pWME->getOperation() == cdl::ADD) {
  //       //debug("adding");
  //       //result = addToWorkingMemory(id,type, &(_pWME->m_data));

  //       type = _pWME->item()->getType();

  //       logMemoryAdd(src, m_subarchitectureID, id, type);

  //       result = addToWorkingMemory(id, _pWME->item());

  //       //     cout<<endl;
  //       //     cout<<"After add data ptr: count: "<<_pWME->item().use_count()<<endl;
  //       //     cout<<endl;


  //     }
  //     else if(_pWME->getOperation() == cdl::OVERWRITE) {
  //       //debug("overwriting");
  //       //result = overwriteWorkingMemory(id,type, &(_pWME->m_data));

  //       type = _pWME->item()->getType();

  //       logMemoryOverwrite(src, m_subarchitectureID, id, type);

  //       result = overwriteWorkingMemory(id, _pWME->item(), src);
  //     }
  //     else if(_pWME->getOperation() == cdl::DELETE) {
  //       debug("deleting");

  //       //cout<<"before removal"<<endl;
  //       //m_workingMemory.debug();

  //       WorkingMemoryEntryPtr pResult = deleteFromWorkingMemory(id, src);

  //       if(pResult) {

  // 	debug("got result");

  // 	//cout<<"after wm removal: "<<pResult.use_count()<<endl;

  // 	//if we have a pointer to what was removed
  // 	result = true;
  // 	type = pResult->getType();

  // 	logMemoryDelete(src, m_subarchitectureID, id, type);

  // 	debug("deleted");
  //       }
  //       else {
  // 	debug("not got result");
  // 	result = false;
  //       }

  //     }

  //     logMemorySize(m_workingMemory.size());


  //     if(result) {
  //       signalChange(_pWME->getOperation(),src,id,type);
  //     }
  //     else {
  //       println("Working memory operation failed: %i %s %s %s",
  // 	       _pWME->getOperation(), src.c_str(),
  // 	       id.c_str(), type.c_str());
  //     }

  //     unlockComponent();


  //   }


  void SubarchitectureWorkingMemory::overwriteWorkingMemory(
							    const std::string & _id, const std::string & _subarch,
							    const std::string & _type, const std::string & _component,
							    const Ice::ObjectPtr & _entry, const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if (getSubarchitectureID() == _subarch) {
      lockComponent();
      //DANGER this might produce an exception... change to critical
      //section
      bool result = overwriteWorkingMemory(_id, createEntry(_id, _type,
							    _entry), _component);
      //sanity check
      assert(result);
      signalChange(cdl::OVERWRITE, _component, _id, _type, _entry->ice_ids());
      unlockComponent();
    } else {
      //send on to the one that really cares
      getWorkingMemory(_subarch)->overwriteWorkingMemory(_id, _subarch,
							 _type, _component, _entry);
    }
  }

  bool SubarchitectureWorkingMemory::overwriteWorkingMemory(const string & _id,
							    WorkingMemoryEntryPtr _pData, const string & _component)
    throw (DoesNotExistOnWMException) {

    //first sanity check
    if (!m_workingMemory.contains(_id)) {
      throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,"Entry does not exist to overwrite. Was trying to overwrite id %s in subarchitecture %s" ,
						       _id.c_str(),getSubarchitectureID().c_str()),
				      makeWorkingMemoryAddress(_id,getSubarchitectureID())));
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

  void
  SubarchitectureWorkingMemory::deleteFromWorkingMemory(const std::string& _id,
							const std::string & _subarch,
							const std::string & _component,
							const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      lockComponent();
      //DANGER this might produce an exception... change to critical
      //section
      WorkingMemoryEntryPtr entry(deleteFromWorkingMemory(_id, _component));
      //sanity check
      assert(entry);
      signalChange(cdl::DELETE,_component,_id,entry->type, entry->ice_ids());
      unlockComponent();
    }
    else {
      //send on to the one that really cares
      getWorkingMemory(_subarch)->deleteFromWorkingMemory(_id,_subarch,_component);
    }
  }

  WorkingMemoryEntryPtr
  SubarchitectureWorkingMemory::deleteFromWorkingMemory(const string & _id,
							const string & _component)
    throw (DoesNotExistOnWMException) {

    //first sanity check
    if(!m_workingMemory.contains(_id)) {
      throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
						       "Entry does not exist to delete. Was trying to delete id %s in subarchitecture %s",
						       _id.c_str(),getSubarchitectureID().c_str()),
				      makeWorkingMemoryAddress(_id,getSubarchitectureID())));
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

    WorkingMemoryEntryPtr pResult(m_workingMemory.remove(_id));

    if (isLocked) {
      // unlock on deletion
      debug("unlocking on deletion: %s %s",_id.c_str(),_component.c_str());
      m_permissions.unlock(_id, _component);
    }

    debug("unlocking on deletion before permission removal: %s %s",_id.c_str(),_component.c_str());

    //must unlock around remove as it may block
    unlockComponent();
    m_permissions.remove(_id);
    lockComponent();

    debug("unlocking on deletion after permission removal: %s %s",_id.c_str(),_component.c_str());

    return pResult;
  }

  cdl::WorkingMemoryEntryPtr
  SubarchitectureWorkingMemory::getWorkingMemoryEntry(const std::string & _id,
						      const std::string & _subarch,
						      const std::string & _component,
						      const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //      println("local query");
      return getWorkingMemoryEntry(_id,_component);
    }
    else {
      //      println("remote query");

      //send on to the one that really cares
      return getWorkingMemory(_subarch)->getWorkingMemoryEntry(_id,_subarch,_component);
    }

  }

  cdl::WorkingMemoryEntryPtr
  SubarchitectureWorkingMemory::getWorkingMemoryEntry(const std::string & _id,
						      const std::string & _component)
    throw (DoesNotExistOnWMException) {

    lockComponent();

    if (m_workingMemory.contains(_id)) {


      //block access if necessary
      readBlock(_id,_component);

      WorkingMemoryEntryPtr entry = m_workingMemory.get(_id);

      unlockComponent();

      //the entry might not be there if something changed during the
      //readBlock
      if(entry) {
	return entry;
      }

    }

    unlockComponent();


    //if we get this far nothing exists to return
    throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
						     "Entry does not exist for reading. Was looking in subarch %s for id %s",
						     getSubarchitectureID().c_str(),_id.c_str()),
				    makeWorkingMemoryAddress(_id,getSubarchitectureID())));

  }


  void
  SubarchitectureWorkingMemory::getWorkingMemoryEntries(const std::string & _type,
							const std::string & _subarch,
							Ice::Int _count,
							const std::string & _component,
							cast::cdl::WorkingMemoryEntrySeq & _entries,
							const Ice::Current & _ctx)

    throw (UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //
      getWorkingMemoryEntries(_type,_count, _component, _entries);
    }
    else {
      //send on to the one that really cares
      getWorkingMemory(_subarch)->getWorkingMemoryEntries(_type,_subarch,_count,_component, _entries);
    }


  }


  void
  SubarchitectureWorkingMemory::getWorkingMemoryEntries(const std::string & _type,
							Ice::Int _count,
							const std::string & _component,
							cast::cdl::WorkingMemoryEntrySeq & _entries) {


    //get ids of the entries to return
    vector<string> ids;
    lockComponent();
    m_workingMemory.getIDsByType(_type,_count, ids);

    //now get each item in turn
    for(vector<string>::const_iterator i = ids.begin();
	i < ids.end(); ++i) {


      //check whether we need to block
      readBlock(*i, _component);

      WorkingMemoryEntryPtr entry(m_workingMemory.get(*i));

      //if deletion during read block then things might not be there
      //any more
      if(entry) {
	_entries.push_back(entry);
      }
    }

    unlockComponent();

  }


  void
  SubarchitectureWorkingMemory::receiveChangeEvent(const cdl::WorkingMemoryChange& wmc,
						   const Ice::Current & _ctx) {

    lockComponent();

    // if the filters require external changes, allow them to be
    // forwarded
    if (!m_componentFilters.localFiltersOnly()) {

      if(m_bDebugOutput) {
	ostringstream outStream;
	outStream<<"forwarding change: "<<wmc;
	debug(outStream.str());
      }

      for(vector<WorkingMemoryReaderComponentPrx>::iterator reader = m_readers.begin();
	  reader < m_readers.end(); ++ reader) {
	(*reader)->receiveChangeEvent(wmc);
      }

    }

    unlockComponent();

  }




  void
  SubarchitectureWorkingMemory::signalChange(cdl::WorkingMemoryOperation _op,
					     const string & _src,
					     const string &  _id,
					     const string &  _type,
					     const vector<string> & _typeHierarchy) {
	  

    cdl::WorkingMemoryChange wmc;
    wmc.operation = _op;
    wmc.src = _src;
    wmc.address.id = _id;
    wmc.address.subarchitecture = getSubarchitectureID();
    wmc.type = _type;
    wmc.superTypes = _typeHierarchy;

    if(m_bDebugOutput) {
      ostringstream outStream;
      outStream<<"SubarchitectureWorkingMemory::signalChange: "<<wmc<<endl;
      debug(outStream.str());
    }

    //signal change locally if allowed
    if (isAllowedChange(wmc)) {
      //send locally
      for(vector<WorkingMemoryReaderComponentPrx>::iterator reader = m_readers.begin();
	  reader < m_readers.end(); ++ reader) {
	(*reader)->receiveChangeEvent(wmc);
      }
    }

    // signal change across sub-architectures where appropriate
    if (isSendingXarchChangeNotifications()) {
      for(WMPrxMap::iterator i = m_workingMemories_oneway.begin();
	  i != m_workingMemories_oneway.end(); ++i) {

	if(isAllowedChange(i->first,wmc)) {

	  i->second->receiveChangeEvent(wmc);
	}
      }
    }

  }

  //   AbstractWorkingMemoryPullQuery * SubarchitectureWorkingMemory::parseWorkingMemoryQuery(const FrameworkQuery & _query) {
  //     return SubarchitectureWorkingMemoryProtocol::parseQuery(_query.getQuery());
  //   }


  //   void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query,
  // 						      FrameworkLocalData<vector<CASTWorkingMemoryEntry *> > *& _pData) {



  //     //println("received local pull");

  //     //create the query object
  //     AbstractWorkingMemoryPullQuery * pQuery
  //       = parseWorkingMemoryQuery(_query);

  //     //create result list has to be a pointer/new because the memory is
  //     //later returned to another component
  //     vector<CASTWorkingMemoryEntry *> *pWMEL =
  //       new vector<CASTWorkingMemoryEntry *>();

  //     handleWorkingMemoryQuery(pQuery,*pWMEL);

  //     //debug("query handled");

  //     //free up query memory
  //     delete pQuery;

  //     //vector<CASTWorkingMemoryEntry *> *pList = new vector<CASTWorkingMemoryEntry *>();

  //     //collect the ids first
  //     //for(unsigned int i = 0; i < pWMEL->size(); i++) {

  //     //doing this would send the stored copy... dangerous, although it could be consted
  //     //pList->push_back((*pWMEL)[i]);
  //     //pList->push_back(new CASTWorkingMemoryEntry(*(*pWMEL)[i]));
  //     //}

  //     //create data object
  //     _pData
  //       = new FrameworkLocalData<vector<CASTWorkingMemoryEntry *> >(getComponentIdentifier(),
  // 								  pWMEL);

  //     //if(_pData) {
  //     //println("returning non null pData");
  //     //}


  //     //logging
  //     //   if(m_bLogOutput) {
  //     //     for(unsigned int i = 0; i < pWMEL->size(); i++) {
  //     //       CASTWorkingMemoryEntry * pCWME = (*pWMEL)[i];
  //     //       if(pCWME->item()->getType() != "Vision:ImageBufferState") {
  //     // 	ostringstream outStream;
  //     // 	outStream<<"GET"<<" "<<BALTTimer::getBALTTime()<<" "<<_query.getSource()<<" "<<pCWME->item()->getType()<<" "<<pCWME->getAddress().m_id;
  //     // 	debug(outStream.str());
  //     //       }
  //     //     }
  //     //   }

  //     //m_workingMemory.debug();
  //   }

  //   void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query,
  // 						      FrameworkLocalData<cdl::WorkingMemoryEntryList> *& _pData) {


  //     //println("pull query received");

  //     //create the query object
  //     AbstractWorkingMemoryPullQuery * pQuery
  //       = parseWorkingMemoryQuery(_query);

  //     //create result list
  //     //cdl::WorkingMemoryEntryList *pWMEL = new cdl::WorkingMemoryEntryList();
  //     vector<CASTWorkingMemoryEntry *> wmel;

  //     handleWorkingMemoryQuery(pQuery,wmel);

  //     //debug("query handled");

  //     //free up query memory
  //     delete pQuery;

  //     cdl::WorkingMemoryEntryList *pList = new cdl::WorkingMemoryEntryList();
  //     pList->length(wmel.size());




  //     for(unsigned int i = 0; i < wmel.size(); i++) {
  //       CASTWorkingMemoryEntry * pCWME = wmel[i];
  //       (*pList)[i].m_operation = pCWME->getOperation();
  //       (*pList)[i].m_address = pCWME->getAddress();
  //       (*pList)[i].m_type = CORBA::string_dup(pCWME->item()->getType().c_str());
  //       (*pList)[i].m_version = pCWME->item()->getVersion();
  //       //if the data was stored as an any, the any specialisation will
  //       //ensure the following line does nothing
  //       pCWME->item()->toAny((*pList)[i].m_data);

  //       //     if(m_bLogOutput && pCWME->item()->getType() != "Vision:ImageBufferState") {
  //       //       ostringstream outStream;
  //       //       outStream<<"GET"<<" "<<BALTTimer::getBALTTime()<<" "<<_query.getSource()<<" "<<pCWME->item()->getType()<<" "<<pCWME->getAddress().m_id;
  //       //       debug(outStream.str());
  //       //     }

  //     }

  //     //create data object
  //     _pData
  //       = new FrameworkLocalData<cdl::WorkingMemoryEntryList>(getComponentIdentifier(),
  // 							    pList);

  //     //if(_pData) {
  //     //println("returning non null pData");
  //     //}

  //     //need to free entry memory
  //     for(vector<CASTWorkingMemoryEntry *>::iterator i = wmel.begin();
  // 	i < wmel.end(); ++i) {
  //       delete (*i);
  //       (*i) = NULL;
  //     }


  //     //m_workingMemory.debug();

  //   }


  //UPGRADE FROM HERE FIRST

  //   void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query,
  // 						      FrameworkLocalData< int > *& _pData) {

  //     //println("count query received");

  //     string id = _query.getQuery();
  //     int overwriteCount = -1;

  //     if(m_workingMemory.hasContained(id)) {
  //       overwriteCount = m_workingMemory.getOverwriteCount(id);
  //     }
  //     else {
  //       debug("called local: wm has not contained: %s", id.c_str());
  //     }

  //     //create data object
  //     _pData
  //       = new FrameworkLocalData< int >(getComponentIdentifier(),
  // 				      new int(overwriteCount));

  //     //println("and finished");
  //   }

  //   void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query,
  // 						      FrameworkLocalData< bool > *& _pData) {

  //     //println("exists query received");

  //     string id = _query.getQuery();
  //     bool exists = m_workingMemory.contains(id);

  //     //create data object
  //     _pData
  //       = new FrameworkLocalData< bool >(getComponentIdentifier(),
  // 				       new bool(exists));

  //     //println("and finished");
  //   }


  //   void SubarchitectureWorkingMemory::handleWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery,
  // 							      vector<CASTWorkingMemoryEntry *> & _wmel) {

  //     if(_pQuery->getSubarchitectureID() != m_subarchitectureID) {
  //       handleXarchWorkingMemoryQuery(_pQuery,_wmel);
  //     }
  //     else {
  //       handleLocalWorkingMemoryQuery(_pQuery,_wmel);
  //     }

  //   }

  //   void SubarchitectureWorkingMemory::handleLocalWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery, 								 vector<CASTWorkingMemoryEntry *> & _wmel) {


  //     //only lock local access, as xarch can look after itself
  //     lockComponent();

  //     if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID) {
  //       //println("ID query");
  //       WorkingMemoryPullQuery<string> * pQuery =
  // 	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

  //       logReadEvent(pQuery->getSourceID(),
  // 		   pQuery->getSourceSA(),
  // 		   *(pQuery->getQueryObject()));

  //       if(pQuery) {
  // 	getEntryByID(pQuery,_wmel);
  //       }
  //       else {
  // 	println("ERROR: casting query to ID");
  //       }

  //     }
  //     else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::EXISTS) {

  //       //println("ID query");
  //       WorkingMemoryPullQuery<string> * pQuery =
  // 	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

  //       //  logReadEvent(pQuery->getSourceID(),
  //       // 		 pQuery->getSourceSA(),
  //       // 		 *(pQuery->getQueryObject()));


  //       string queryID = *(pQuery->getQueryObject());
  //       bool exists = false;

  //       if(m_workingMemory.contains(queryID)) {
  // 	//println("exists TRUE");
  // 	exists = true;
  //       }
  //       else {
  // 	// println("exists FALSE");
  //       }

  //       shared_ptr< WorkingMemoryItem<bool> > pItem(new WorkingMemoryItem<bool>(queryID,"bool", new bool(exists)));

  //       // if true

  //       //make sure list only contains the new item
  //       _wmel.clear();

  //       cdl::WorkingMemoryAddress wma;
  //       wma.m_id = CORBA::string_dup(queryID.c_str());
  //       wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());

  //       _wmel.push_back(new CASTWorkingMemoryEntry(getComponentIdentifier(),
  // 						 cdl::GET,
  // 						 wma,
  // 						 pItem));


  //       //println("exists query, doing nothing");

  //       //     if(pQuery) {
  //       //       getEntryByID(pQuery,_wmel);
  //       //     }
  //       //     else {
  //       //       println("ERROR: casting query to ID");
  //       //     }

  //     }
  //     else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::OVERWRITE_COUNT) {

  //       WorkingMemoryPullQuery<string> * pQuery =
  // 	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

  //       string queryID = *(pQuery->getQueryObject());
  //       int overwriteCount = -1;

  //       if(m_workingMemory.hasContained(queryID)) {
  // 	overwriteCount = m_workingMemory.getOverwriteCount(queryID);
  //       }
  //       else {
  // 	debug("called xarch: wm has not contained: %s", queryID.c_str());
  //       }

  //       shared_ptr< WorkingMemoryItem<int> > pItem(new WorkingMemoryItem<int>(queryID,"int", new int(overwriteCount)));

  //       // if true

  //       //make sure list only contains the new item
  //       _wmel.clear();

  //       cdl::WorkingMemoryAddress wma;
  //       wma.m_id = CORBA::string_dup(queryID.c_str());
  //       wma.m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());

  //       _wmel.push_back(new CASTWorkingMemoryEntry(getComponentIdentifier(),
  // 						 cdl::GET,
  // 						 wma,
  // 						 pItem));

  //       //println("exists query, doing nothing");

  //       //     if(pQuery) {
  //       //       getEntryByID(pQuery,_wmel);
  //       //     }
  //       //     else {
  //       //       println("ERROR: casting query to ID");
  //       //     }

  //     }
  //     else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::ID_ARRAY) {
  //       //println("ID Array query");
  // //       WorkingMemoryPullQuery< vector<string> > * pQuery =
  // // 	dynamic_cast<WorkingMemoryPullQuery< vector<string> > *>(_pQuery);

  // //       logReadEvent(pQuery->getSourceID(),
  // // 		   pQuery->getSourceSA(),
  // // 		   "multi string query");

  // //       if(pQuery) {
  // // 	getEntriesByID(pQuery,_wmel);
  // //       }
  // //       else {
  // // 	println("ERROR: casting query to ID array");
  // //       }

  //       throw(SubarchitectureComponentException(__HERE__,
  // 					    "ID array query unsupported"));

  //     }
  //     else if(_pQuery->getType() == AbstractWorkingMemoryPullQuery::TYPE) {
  //       //println("Type query");
  //       WorkingMemoryPullQuery<string> * pQuery =
  // 	dynamic_cast<WorkingMemoryPullQuery<string> *>(_pQuery);

  //       logReadEvent(pQuery->getSourceID(),
  // 		   pQuery->getSourceSA(),
  // 		   *(pQuery->getQueryObject()),
  // 		   pQuery->getCount());

  //       if(pQuery) {
  // 	getEntriesByType(pQuery,_wmel);
  //       }
  //       else {
  // 	println("ERROR: casting query to type");
  //       }

  //     }
  //     else {
  //       println("Unknown query type");
  //     }

  //     unlockComponent();


  //   }

  //   void SubarchitectureWorkingMemory::handleXarchWorkingMemoryQuery(AbstractWorkingMemoryPullQuery *_pQuery,
  // 								   vector<CASTWorkingMemoryEntry *> & _wmel) {

  //     //TODO... reduce redundancy that this introduces

  //     WMEntryListPullConnectorMap::iterator i
  //       = m_xarchWMPullConnections.find(_pQuery->getSubarchitectureID());


  //     //log("using id for remote pull: %s",_pQuery->getSubarchitectureID().c_str());

  //     if(i != m_xarchWMPullConnections.end()) {

  //       PullConnectorOut<cdl::WorkingMemoryEntryList> * pOut = i->second;

  //       //wrap the input pointer in a local data structure for pull

  //       string query = SubarchitectureWorkingMemoryProtocol::createQuery(_pQuery);
  //       FrameworkQuery fq(getComponentIdentifier(),query);
  //       FrameworkLocalData<cdl::WorkingMemoryEntryList> *pFLD = NULL;

  //       //println("handleXarchWorkingMemoryQuery");

  //       //println("handleXarchWorkingMemoryQuery");
  //       //println(query);

  //       pOut->pull(fq,pFLD);

  //       //println("handleXarchWorkingMemoryQuery after pull");

  //       if(!pFLD) {
  // 	println("XArch pull returned null");
  // 	//_pCWMEL = NULL;
  //       }
  //       else {

  // 	//TODO some deleting!
  // 	//       _pWMEL = pFLD->data();
  // 	//       pFLD->data() = NULL;
  // 	//       delete pFLD;

  // 	cdl::WorkingMemoryEntryList * pWMEL = pFLD->data();
  // 	for(unsigned int j = 0; j < pWMEL->length(); j++) {

  // 	  //cdl::WorkingMemoryEntry *pWME = _pData->data();
  // 	  string type((*pWMEL)[j].m_type);
  // 	  string id((*pWMEL)[j].m_address.m_id);
  // 	  int version((*pWMEL)[j].m_version);


  // 	  //println("handleXarchWorkingMemoryQuery %s %s", type.c_str(), id.c_str());

  // 	  //FIXME why is this copying here???  what happens to
  // 	  //(*pWMEL)[j].m_data when delete pWMEL is called via delete
  // 	  //pFLD? It's on the heap so it probably goes... hence the
  // 	  //copying.... booo!

  // 	  CASTWorkingMemoryItem * pWMI
  // 	    = new WorkingMemoryItem<CORBA::Any>(id,
  // 						type,
  // 						version,
  // 						//this induces a copy
  // 						//and is equivalent to
  // 						//the line below
  // 						(*pWMEL)[j].m_data);
  // 	  //new CORBA::Any((*pWMEL)[j].m_data));

  // 	  CASTWorkingMemoryEntry *pCWME
  // 	    = new CASTWorkingMemoryEntry(getComponentIdentifier(),
  // 					 (*pWMEL)[j].m_operation,
  // 					 (*pWMEL)[j].m_address,
  // 					 pWMI);

  // 	  _wmel.push_back(pCWME);

  // 	}

  // 	delete pFLD;
  // 	pFLD = NULL;

  //       }

  //     }
  //     else {
  //       println("Unknown subarchitecture: "
  // 	      + _pQuery->getSubarchitectureID());
  //     }

  //   }

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

      unlockComponent();

      //lock entry, this will block
      m_permissions.lock(_id,_component, cdl::LOCKEDODR);

      debug("locked readBlock: %s %s", _id.c_str(), _component.c_str());

      // relock to finish the operation
      lockComponent();

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


  void SubarchitectureWorkingMemory::configureInternal(const map<string,string>& _config) {
    SubarchitectureComponent::configureInternal(_config);

    map<string,string>::const_iterator key = _config.find(cdl::IGNORESAKEY);
    if(key != _config.end()) {
      const string & ignore = key->second;
      vector<string> ignoreList;
      tokenizeString(ignore,
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

  void SubarchitectureWorkingMemory::buildIDLists(const std::map<std::string,std::string>& _config) {

    map<string,string>::const_iterator key = _config.find(cdl::WMIDSKEY);
    assert(key != _config.end());
    const string & wmIDs = key->second;
    vector<string> ids;
    tokenizeString(wmIDs, ids, ",");
    for(vector<string>::iterator id = ids.begin();
	id < ids.end();
	++id) {
      //debug(*id);
      m_wmIDs.insert(*id);
    }
  }



  bool
  SubarchitectureWorkingMemory::exists(const std::string & _id,
				       const std::string & _subarch,
				       const Ice::Current & _ctx)
    throw (UnknownSubarchitectureException) {
    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      return m_workingMemory.contains(_id);
    }
    else {
      //send on to the one that really cares
      return getWorkingMemory(_subarch)->exists(_id,_subarch);
    }
  }


  Ice::Int
  SubarchitectureWorkingMemory::getVersionNumber(const std::string & _id,
						 const std::string & _subarch,
						 const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {
    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //if it has once existed, then return a value
      if(m_workingMemory.hasContained(_id)) {
	return m_workingMemory.getOverwriteCount(_id);
      }
      //else kick up a fuss
      else {
	throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry has never existed on wm. Was looking in subarch %s for id %s",
							 _subarch.c_str(),_id.c_str()),
					makeWorkingMemoryAddress(_id,_subarch)));
      }
    }
    else {
      //send on to the one that really cares
      return getWorkingMemory(_subarch)->getVersionNumber(_id,_subarch);
    }
  }


  cdl::WorkingMemoryPermissions
  SubarchitectureWorkingMemory::getPermissions(const std::string & _id,
					       const std::string & _subarch,
					       const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //if it exists, then return a value
      if(m_workingMemory.contains(_id)) {
	WorkingMemoryPermissions permissions = m_permissions.getPermissions(_id);
	return permissions;
      }
      //else kick up a fuss
      else {
	throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry does not exist on wm. Was looking in subarch %s for id %s",
							 _subarch.c_str(),_id.c_str()),
					makeWorkingMemoryAddress(_id,_subarch)));
      }
    }
    else {
      //get the correct wm and query that instead
      //send on to the one that really cares
      return getWorkingMemory(_subarch)->getPermissions(_id,_subarch);
    }

  }


  void
  SubarchitectureWorkingMemory::lockEntry(const std::string & _id,
					  const std::string & _subarch,
					  const std::string & _component,
					  cdl::WorkingMemoryPermissions _perm,
					  const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //
      if (m_workingMemory.contains(_id)) {

	debug("%s locking: %s",_component.c_str(),_id.c_str());
	m_permissions.lock(_id, _component, _perm);
	//lockComponent();

	//now check that it still exists, because it could've been
	//deleted before the lock was released
	if (!m_workingMemory.contains(_id)) {
	  m_permissions.unlock(_id, _component);
	  throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							   "Entry deleted whiile waiting for lock. Component %s was looking in subarch %s for id %s",
							   _component.c_str(),
							   _subarch.c_str(),
							   _id.c_str()),
					  makeWorkingMemoryAddress(_id,_subarch)));
	}
	else {
	  assert(m_permissions.isLockHolder(_id,_component));
	  assert(m_permissions.getPermissions(_id) == _perm);
	  debug("%s locked: ok",_component.c_str());
	}
      }
      //else kick up a fuss
      else {
	throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry does not exist for locking. Was looking in subarch %s for id %s",
							 _subarch.c_str(),_id.c_str()),
					makeWorkingMemoryAddress(_id,_subarch)));
      }
    }
    else {
      getWorkingMemory(_subarch)->lockEntry(_id,_subarch, _component, _perm);
    }
  }




  bool
  SubarchitectureWorkingMemory::tryLockEntry(const std::string & _id,
					     const std::string & _subarch,
					     const std::string & _component,
					     cdl::WorkingMemoryPermissions _perm,
					     const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //
      if (m_workingMemory.contains(_id)) {
	if (m_permissions.tryLock(_id, _component, _perm)) {
	  debug("%s try locked: %s",_component.c_str(),_id.c_str());
	  assert(m_permissions.isLockHolder(_id,_component));
	  assert(m_permissions.getPermissions(_id) == _perm);
	  debug("%s try locked: ok",_component.c_str());
	  return true;
	}
	else {
	  return false;
	}
      }
      //else kick up a fuss
      else {
	throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry does not exist for try-locking. Was looking in subarch %s for id %s",
							 _subarch.c_str(),_id.c_str()),
					makeWorkingMemoryAddress(_id,_subarch)));
      }
    }
    else {
      //get the correct wm and query that instead
      return getWorkingMemory(_subarch)->tryLockEntry(_id,_subarch, _component, _perm);
    }
  }


  void
  SubarchitectureWorkingMemory::unlockEntry(const std::string & _id,
					    const std::string & _subarch,
					    const std::string & _component,
					    const Ice::Current & _ctx)
    throw (DoesNotExistOnWMException, ConsistencyException, UnknownSubarchitectureException) {

    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //
      if (m_workingMemory.contains(_id)) {

	//this wasn't in the original design, but seems necessary now
	//we can do this at wm level
	if(!m_permissions.isLocked(_id)) {
	  throw(ConsistencyException(exceptionMessage(__HERE__,
						      "Entry is not locked. %s was looking in subarch %s for id %s",
						      _component.c_str(),_subarch.c_str(),_id.c_str()),
				     makeWorkingMemoryAddress(_id,_subarch)));
	}
	else if(m_permissions.getLockHolder(_id) != _component) {
	  throw(ConsistencyException(exceptionMessage(__HERE__,
						      "Entry is not locked, but not by %s, who was looking in subarch %s for id %s",
						      _component.c_str(),_subarch.c_str(),_id.c_str()),
				     makeWorkingMemoryAddress(_id,_subarch)));
	}

	m_permissions.unlock(_id, _component);
	debug("%s unlocked %s",_component.c_str(),_id.c_str());
      }
      //else kick up a fuss
      else {
	throw(DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry does not exist for unlocking. Was looking in subarch %s for id %s",
							 _subarch.c_str(),_id.c_str()),
					makeWorkingMemoryAddress(_id,_subarch)));
      }
    }
    else {
      //get the correct wm and query that instead
      getWorkingMemory(_subarch)->unlockEntry(_id,_subarch, _component);
    }

  }


  bool
  SubarchitectureWorkingMemory::addToWorkingMemory(const string & _id,
						   WorkingMemoryEntryPtr _entry) {
    bool result = m_workingMemory.add(_id,_entry);
    if (result) {
      m_permissions.add(_id);
    }
    return result;
  }




  void
  SubarchitectureWorkingMemory::addToWorkingMemory(const std::string & _id,
						   const std::string & _subarch,
						   const std::string & _type,
						   const std::string & _component,
						   const Ice::ObjectPtr & _entry,
						   const Ice::Current & _ctx)
    throw (AlreadyExistsOnWMException, UnknownSubarchitectureException) {


    //if this is for me
    if(getSubarchitectureID() == _subarch) {
      //if it already exists complain bitterly
      if (m_workingMemory.contains(_id)) {
	throw(AlreadyExistsOnWMException(exceptionMessage(__HERE__,
							  "Entry already exists on WM. Was trying to write id %s in subarchitecture %s",
							  _id.c_str(),_subarch.c_str()),
					 makeWorkingMemoryAddress(_id,_subarch)));

      }
      //else get stuck in
      else {
	//fine then
	lockComponent();
	bool result = addToWorkingMemory(_id, createEntry(_id,_type,_entry));
	//sanity check
	assert(result);
	signalChange(cdl::ADD,_component,_id,_type, _entry->ice_ids());
	unlockComponent();
      }
    }
    else {

      //get the correct wm and query that instead
      getWorkingMemory(_subarch)->addToWorkingMemory(_id,_subarch, _type, _component, _entry);

    }
  }



  void
  SubarchitectureWorkingMemory::registerComponentFilter(const cdl::WorkingMemoryChangeFilter & _filter,
							const ::Ice::Current & _ctx) {

    debug("SubarchitectureWorkingMemory::registerComponentFilter()");
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());

    m_componentFilters[_filter].push_back(_filter.origin);

    //cout<<"new filters length: "<<m_componentFilters.size()<<endl;
    //cout<<"only local: "<<m_componentFilters.localFiltersOnly()<<endl;


    if (m_wmDistributedFiltering) {
      debug("fowarding filter");
      //sendFilter(_filter);
      for(WMPrxMap::iterator i = m_workingMemories.begin();
	  i != m_workingMemories.end(); ++i) {
	debug("forwarding to %s", i->first.c_str());
	debug("forwarding to %s", i->second->getID().c_str());
	i->second->registerWorkingMemoryFilter(_filter, getSubarchitectureID());
      }
    }


  }

  void
  SubarchitectureWorkingMemory::registerWorkingMemoryFilter(const cdl::WorkingMemoryChangeFilter & _filter,
							    const std::string & _subarch,
							    const ::Ice::Current & _ctx) {
    debug("SubarchitectureWorkingMemory::registerWorkingMemoryFilter()");
    ostringstream outStream;
    outStream<<_filter;
    debug(outStream.str());

    m_wmFilters[_filter].push_back(_subarch);

    //     cout<<"new filters length: "<<m_wmFilters.size()<<endl;
    //     cout<<"only local: "<<m_wmFilters.localFiltersOnly()<<endl;
  }


  void
  SubarchitectureWorkingMemory::removeComponentFilter(const cdl::WorkingMemoryChangeFilter & _filter,
						      const ::Ice::Current & _ctx) {


    //     debug("SubarchitectureWorkingMemory::deleteComponentChangeFilter()");
    //     debug(_src);
    //     ostringstream outStream;
    //     outStream<<_filter;
    //    debug(outStream.str());

    vector<string> removed;
    m_componentFilters.remove(_filter, removed);

    //cout<<"new filters length: "<<m_componentFilters.size()<<endl;
    //cout<<"only local: "<<m_componentFilters.localFiltersOnly()<<endl;

    for(WMPrxMap::iterator i = m_workingMemories.begin();
	i != m_workingMemories.end(); ++i) {
      i->second->removeWorkingMemoryFilter(_filter);
    }
  }


  void
  SubarchitectureWorkingMemory::removeWorkingMemoryFilter(const cdl::WorkingMemoryChangeFilter & _filter,
							  const ::Ice::Current & _ctx) {

    //     debug("SubarchitectureWorkingMemory::deleteWMChangeFilter()");
    //     debug(_src);
    //     ostringstream outStream;
    //     outStream<<_filter;
    //     debug(outStream.str());

    vector<string> removed;
    m_wmFilters.remove(_filter, removed);

    //cout<<"new filters length: "<<m_wmFilters.size()<<endl;
    //cout<<"only local: "<<m_wmFilters.localFiltersOnly()<<endl;

  }




  //   void SubarchitectureWorkingMemory::setPushConnector(const string & _connectionID,
  // 						      PushConnectorOut<cdl::WorkingMemoryChangeFilter> * _pOut) {
  //     assert(m_xarchWMCFPushConnector == NULL);
  //     m_xarchWMCFPushConnector = _pOut;
  //   }

  //   void SubarchitectureWorkingMemory::sendFilter(const cdl::WorkingMemoryChangeFilter & _data)  {
  //     if(m_xarchWMCFPushConnector != NULL) {
  //       m_xarchWMCFPushConnector->push(new FrameworkLocalData<cdl::WorkingMemoryChangeFilter>(getComponentIdentifier(), _data));

  //       unlockComponent();
  //       //it's deadlock-dangerous here
  //       m_xarchWMCFPushConnector->flush();
  //       lockComponent();

  //     }
  //   }


  //   bool SubarchitectureWorkingMemory::isWorkingMemoryID(const std::string & _id) const {
  //     StringSet::iterator i = m_wmIDs.find(_id);
  //     return i != m_wmIDs.end();
  //   }


  //   void SubarchitectureWorkingMemory::processWorkingMemoryFilter(const std::string & _src,
  // 								const cdl::WorkingMemoryChangeFilter & _filter)  {
  //     if (_filter.m_filterChange == cdl::ADD) {
  //       addWMChangeFilter(_src, _filter);
  //     }
  //     else {
  //       deleteWMChangeFilter(_src, _filter);
  //     }

  //   }

  //   void SubarchitectureWorkingMemory::processComponentFilter(const std::string & _src,
  // 							    const cdl::WorkingMemoryChangeFilter & _filter) {
  //     if (_filter.m_filterChange == cdl::ADD) {
  //       addComponentChangeFilter(_src, _filter);
  //     }
  //     else {
  //       deleteComponentChangeFilter(_src, _filter);
  //     }

  //     // if we're sharing our filters
  //     if (m_wmDistributedFiltering) {
  //       debug("fowarding filter");
  //       sendFilter(_filter);
  //     }


  //   }


  //   void SubarchitectureWorkingMemory::setPullConnector(const string & _connectionID,
  // 						      PullConnectorOut<cdl::WorkingMemoryPermissions> * _pOut) {

  //     if (_connectionID.substr(0,ArchitectureConfiguration::XARCH_PREFIX.length()) == ArchitectureConfiguration::XARCH_PREFIX) {
  //       //debug(_connectionID);
  //       string subarch = subarchitectureFromID(_connectionID.substr(ArchitectureConfiguration::XARCH_PREFIX.length()+1));
  //       m_xarchLockConnections[subarch] = _pOut;
  //     }

  //   }

  //   void SubarchitectureWorkingMemory::receivePullQuery(const FrameworkQuery & _query,
  // 						      FrameworkLocalData<cdl::WorkingMemoryPermissions> *& _pData) {


  //     vector<string> tokens;
  //     SubarchitectureWorkingMemoryProtocol::tokenizeString(_query.getQuery(),
  // 							 tokens,
  // 							 "\\");
  //     assert (tokens.size() == 3);
  //     string subarch = tokens[1];

  //     if (subarch == m_subarchitectureID) {

  //       lockComponent();

  //       string id = tokens[0];
  //       WorkingMemoryLockRequest request = (WorkingMemoryLockRequest) atoi(tokens[2].c_str());
  //       WorkingMemoryPermissions permissions = handleLocalPermissionsQuery(id, request, _query.getSource());

  //       ostringstream outStream;
  //       outStream<<permissions;
  //       debug("SubarchitectureWorkingMemory::receivePullQuery: %s", outStream.str().c_str());

  //       _pData
  // 	= new FrameworkLocalData<WorkingMemoryPermissions>(getComponentIdentifier(),
  // 							   permissions);

  //       unlockComponent();
  //     }
  //     else {

  //       PermissionsPullConnectorMap::iterator i
  // 	= m_xarchLockConnections.find(subarch);

  //       debug("forwarding permissions pull across subarch: %s", _query.getQuery().c_str());

  //       assert(i != m_xarchLockConnections.end());

  //       PullConnectorOut<cdl::WorkingMemoryPermissions> * pOut = i->second;

  //       FrameworkQuery fq(_query);
  //       pOut->pull(fq,_pData);


  //     }

  //   }

  //   cdl::WorkingMemoryPermissions
  //   SubarchitectureWorkingMemory::handleLocalPermissionsQuery(const std::string & _id,
  // 							    const cdl::WorkingMemoryLockRequest & _request,
  // 							    const std::string & _component)
  //     throw(SubarchitectureComponentException) {


  //     // sanity check up front
  //     if (!m_workingMemory.contains(_id)) {
  //       return DOES_NOT_EXIST;
  //     }

  //     // if it's a lock
  //     if (_request < 3) {
  //       WorkingMemoryPermissions permissions = toPermissions(_request);
  //       //unlock before trying to lock
  //       unlockComponent();
  //       debug("%s locking: %s",_component.c_str(),_id.c_str());
  //       m_permissions.lock(_id, _component, permissions);

  //       lockComponent();

  //       //now check that it still exists, because it could've been
  //       //deleted before the lock was released
  //       if (!m_workingMemory.contains(_id)) {
  // 	debug("entry %s deleted while %s waiting for lock", _id.c_str(), _component.c_str());
  // 	m_permissions.unlock(_id, _component);
  // 	permissions = DOES_NOT_EXIST;
  //       }
  //       else {
  // 	assert(m_permissions.isLockHolder(_id,_component));
  // 	assert(m_permissions.getPermissions(_id) == permissions);
  // 	debug("%s locked: ok",_component.c_str());
  //       }

  //       return permissions;
  //     }
  //     // it's a try-lock
  //     else if (_request < 6) {
  //       WorkingMemoryPermissions permissions = toPermissions(_request);

  //       if (m_permissions.tryLock(_id, _component, permissions)) {

  // 	debug("%s try locked: %s",_component.c_str(),_id.c_str());
  // 	assert(m_permissions.isLockHolder(_id,_component));
  // 	assert(m_permissions.getPermissions(_id) == permissions);
  // 	debug("%s try locked: ok",_component.c_str());
  // 	return permissions;
  //       }
  //       else {
  // 	return ALREADY_LOCKED;
  //       }
  //       // it's a status request
  //     }
  //     else if (_request == REQUEST_STATUS) {
  //       WorkingMemoryPermissions permissions = m_permissions.getPermissions(_id);
  //       ostringstream outStream;
  //       outStream<<permissions;
  //       debug("SubarchitectureWorkingMemory::handleLocalPermissionsQuery:REQUEST_STATUS: %s", outStream.str().c_str());
  //       return permissions;
  //     }
  //     // it's an unlock
  //     else if (_request == REQUEST_UNLOCK) {
  //       m_permissions.unlock(_id, _component);
  //       debug("%s unlocked %s",_component.c_str(),_id.c_str());
  //       return UNLOCKED;
  //     }

  //     //default if nothing happens

  //     throw(SubarchitectureComponentException(__HERE__,
  // 					  "Invalid permission request"));


  //   }


} //namespace cast
