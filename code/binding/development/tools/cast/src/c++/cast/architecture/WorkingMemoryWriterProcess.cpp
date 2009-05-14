/*
  he * CAST - The CoSy Architecture Schema Toolkit
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

#include "WorkingMemoryWriterProcess.hpp"
#include <sstream>

using namespace std;

namespace cast {

  WorkingMemoryWriterProcess::WorkingMemoryWriterProcess(const string &_id) 
    : //InspectableComponent(_id), 
    WorkingMemoryAttachedComponent(_id) {
    m_pInputToRemoteWorkingMemory = NULL;
    m_pInputToLocalWorkingMemory = NULL;
    m_dataCount = 0;
  }


  WorkingMemoryWriterProcess::~WorkingMemoryWriterProcess() {

  }

  const unsigned char ID_TABLE_SIZE = 62;
  const char id_table[ID_TABLE_SIZE + 1] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  // a bit safer perhaps:
  //  const unsigned char ID_TABLE_SIZE = 37;
  //  const char id_table[ID_TABLE_SIZE + 1] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  // generates short unique strings from ints
  inline
  string stringify(int _n) {
    
    if(_n == 0) 
      return string(1,id_table[0]);
    
    string ret;
    
    int i = 0;
    while(_n != 0) {
      ret.resize(i+1);
      unsigned char m = static_cast<unsigned char>(_n % ID_TABLE_SIZE);
      ret[i++] = id_table[m];
      _n -= m;
      assert(_n % ID_TABLE_SIZE == 0);
      _n /= ID_TABLE_SIZE;
    }
    return ret;
  }


  void WorkingMemoryWriterProcess::configure(map<string,string> & _config) {

    LocalWorkingMemoryAttachedComponent::configure(_config);

    map<string,string>::const_iterator i = _config.find(cdl::COMPONENT_NUMBER_KEY);

    assert(i != _config.end());

    istringstream configStream(i->second);
    configStream >> m_componentNumber;
    m_componentNumberString = stringify(m_componentNumber);
    
  }


  void WorkingMemoryWriterProcess::setPushConnector(const string & _connectionID, 
						    PushConnectorOut<cdl::WorkingMemoryEntry> * _pOut) {
    m_pInputToRemoteWorkingMemory = _pOut;
  }

  void WorkingMemoryWriterProcess::setPushConnector(const string & _connectionID, 
						    PushConnectorOut<CASTWorkingMemoryEntry> * _pOut) {
    m_pInputToLocalWorkingMemory = _pOut;
  }

  inline
  string WorkingMemoryWriterProcess::newDataID() {
  
    std::ostringstream o;
        
    //o << m_dataCount << ":";
    o << stringify(m_dataCount) << ":";
    if(m_bDebugOutput) {
      o << getProcessIdentifier();
      o << ":data";
    }
    else {
      o << m_componentNumberString;
    }
    
    m_dataCount++;

    return o.str();
    
  }

  void WorkingMemoryWriterProcess::pushToWorkingMemory(cdl::WorkingMemoryEntry * _pWME,
						       const cdl::OperationMode & _sync) {

    if(m_pInputToRemoteWorkingMemory) {
      FrameworkLocalData<cdl::WorkingMemoryEntry> * fld = new FrameworkLocalData<cdl::WorkingMemoryEntry>(getProcessIdentifier(),_pWME);
      m_pInputToRemoteWorkingMemory->push(fld);
      if(_sync == cdl::BLOCKING) {
	m_pInputToRemoteWorkingMemory->flush();
      }
    }
    else {
      //throw an exception?
      println("input to remote working memory not set");
    }
  }

  void WorkingMemoryWriterProcess::pushToWorkingMemory(CASTWorkingMemoryEntry * _pWME,
						       const cdl::OperationMode & _sync) {

    if(m_pInputToLocalWorkingMemory) {
      FrameworkLocalData<CASTWorkingMemoryEntry> * fld = new FrameworkLocalData<CASTWorkingMemoryEntry>(getProcessIdentifier(),_pWME);
      m_pInputToLocalWorkingMemory->push(fld);
      if(_sync == cdl::BLOCKING) {
	m_pInputToLocalWorkingMemory->flush();
      }
    }
    else {
      //throw an exception?
      println("input to local working memory not set");
    }
  }



  void WorkingMemoryWriterProcess::deleteFromWorkingMemory(const string &_id,
							   const cdl::OperationMode & _sync) 	   
    throw (DoesNotExistOnWMException, SubarchitectureProcessException) {
   
    deleteFromWorkingMemoryHelper(_id,m_subarchitectureID,_sync);
  }


  void WorkingMemoryWriterProcess::deleteFromWorkingMemoryHelper(const string &_id,
								 const string &_subarchitectureID,
								 const cdl::OperationMode & _sync) 
    throw (DoesNotExistOnWMException, SubarchitectureProcessException) {
    
    assert(!_id.empty());//id must not be empty
    assert(!_subarchitectureID.empty());//subarch must not be empty
    
    checkPrivileges(_subarchitectureID);
        
    if (!existsOnWorkingMemory(_id, _subarchitectureID)) {
      throw DoesNotExistOnWMException(makeAdress(_id,_subarchitectureID),
				      __HERE__,
				      "Entry does not exist for deletion. Was looking for id %s in sa %s",
				      _id.c_str(), _subarchitectureID.c_str());
    }

    // if we don't currently hold a lock on this item
    if (!holdsDeleteLock(_id, _subarchitectureID)) {
      // check that we can delete it
      if (!isDeletable(_id, _subarchitectureID)) {
	throw(PermissionException(makeAdress(_subarchitectureID,_id),
				  __HERE__,
				  "Delete not allowed on locked item: %s:%s",
				  _id.c_str(), _subarchitectureID.c_str()));
      }
    }


    logMemoryDelete(_id, _subarchitectureID);

    //always keep versioning... 
    //stopVersioning(_id);

    if(m_pInputToLocalWorkingMemory) {
      
      cdl::WorkingMemoryAddress wma;
      wma.m_id = CORBA::string_dup(_id.c_str());
      wma.m_subarchitecture = CORBA::string_dup(_subarchitectureID.c_str());

      CASTWorkingMemoryEntry *pCWME 
	= new CASTWorkingMemoryEntry(getProcessIdentifier(),
				     cdl::DELETE, 
				     wma, 
				     NULL);
      
      //log("sending delete");
      pushToWorkingMemory(pCWME,_sync); 
    }
    else {
      //translate data to an any
      CORBA::Any a; //null-valued any
      cdl::WorkingMemoryEntry * wme 
	= new cdl::WorkingMemoryEntry();
      wme->m_operation = cdl::DELETE;
      wme->m_address.m_id = CORBA::string_dup(_id.c_str());
      wme->m_address.m_subarchitecture = CORBA::string_dup(_subarchitectureID.c_str());
      wme->m_type = CORBA::string_dup("");
      wme->m_data = a; //need to send somethign to avoid null pointers
  
      pushToWorkingMemory(wme,_sync); 
    }
  }

  /**
   * 
   * Try to obtain a lock on a working memory entry with the given
   * permissions. This will block until the desired lock is obtained.
   * 
   * @param _id
   *            The id of the item on working memory.
   * @param _permissions
   *            The permissions to obtain.
   * @throws DoesNotExistOnWMException
   *             If the item does not exist on wm.
   */
  void  WorkingMemoryWriterProcess::lockEntry(const std::string & _id, 
					      const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    LocalWorkingMemoryAttachedComponent::lockEntry(_id,_permissions);

  }


  /**
   * Try to obtain a lock on a working memory entry with the given
   * permissions. This will block until the desired lock is obtained.
   * 
   * @param _id
   * @param _subarch
   * @param _permissions
   * @throws DoesNotExistOnWMException
   */
  void WorkingMemoryWriterProcess::lockEntry(const std::string & _id, 
					     const std::string & _subarch,
					     const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    WorkingMemoryAttachedComponent::lockEntry(_id,_subarch,_permissions);
  }



  /**
   * Try to obtain a lock on a working memory entry with the given
   * permissions. This will block until the desired lock is obtained.
   * 
   * @param _wma
   * @param _permissions
   * @throws DoesNotExistOnWMException
   */
  void WorkingMemoryWriterProcess::lockEntry(const cdl::WorkingMemoryAddress & _wma,
					     const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    WorkingMemoryAttachedComponent::lockEntry(_wma,_permissions);
  }

  /**
   * Try to obtain a lock on a working memory entry. This will return true if
   * the item is locked, or false if not. This method does not block.
   * 
   * @param _id
   *            The id of the item on working memory.
   * @param _permissions
   *            The permissions to obtain.
   * @throws DoesNotExistOnWMException
   *             If the item does not exist on wm.
   */
  bool WorkingMemoryWriterProcess::tryLockEntry(const std::string & _id,
						const cdl::WorkingMemoryPermissions & _permissions) 
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    return LocalWorkingMemoryAttachedComponent::tryLockEntry(_id,_permissions);
  }



  /**
   * Try to obtain a lock on a working memory entry. This will return true if
   * the item is locked, or false if not. This method does not block.
   * 
   * @param _id
   * @param _subarch
   * @param _permissions
   * @return
   * @throws DoesNotExistOnWMException
   */
  bool WorkingMemoryWriterProcess::tryLockEntry(const std::string & _id, 
						const std::string & _subarch,
						const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    return WorkingMemoryAttachedComponent::tryLockEntry(_id,_subarch,_permissions);
  }

    
  /**
   * Try to obtain a lock on a working memory entry. This will return true if
   * the item is locked, or false if not. This method does not block.
   * 
   * @param _wma
   * @param _permissions
   * @return
   * @throws SubarchitectureProcessException
   */
  bool WorkingMemoryWriterProcess::tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
						const cdl::WorkingMemoryPermissions & _permissions)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    return WorkingMemoryAttachedComponent::tryLockEntry(_wma,_permissions);
  }



  void WorkingMemoryWriterProcess::unlockEntry(const std::string & _id) 
    throw(DoesNotExistOnWMException){
    flushWriteConnections(); //TODO: inefficient, this is called too often
    LocalWorkingMemoryAttachedComponent::unlockEntry(_id);
  }

  /**
   * Unlock the given working memory entry.
   * 
   * @param _id
   * @param _subarch
   * @throws DoesNotExistOnWMException
   */
  void WorkingMemoryWriterProcess::unlockEntry(const std::string & _id, 
					       const std::string & _subarch)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    WorkingMemoryAttachedComponent::unlockEntry(_id,_subarch);
  }


  /**
   * Unlock the given working memory entry.
   * 
   * @param _wma
   * @throws DoesNotExistOnWMException
   */
  void WorkingMemoryWriterProcess::unlockEntry(const cdl::WorkingMemoryAddress & _wma)
    throw(DoesNotExistOnWMException) {
    flushWriteConnections(); //TODO: inefficient, this is called too often
    WorkingMemoryAttachedComponent::unlockEntry(_wma);
  }

  void WorkingMemoryWriterProcess::flushWriteConnections() {
    if (m_pInputToRemoteWorkingMemory) {
      m_pInputToRemoteWorkingMemory->flush();
    }
    if (m_pInputToLocalWorkingMemory) {
      m_pInputToLocalWorkingMemory->flush();
    }
  }


} //namespace cast
