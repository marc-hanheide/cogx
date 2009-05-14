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

#ifndef CAST_WORKING_MEMORY_WRITER_PROCESS_H_
#define CAST_WORKING_MEMORY_WRITER_PROCESS_H_

#include <cast/core/CASTCore.hpp>
#include <balt/interface/BALTInterface.hpp>

#include "WorkingMemoryAttachedComponent.hpp"
#include "AlreadyExistsOnWMException.hpp"
#include "PermissionException.hpp"

namespace cast {

  /**
   * Class to define an abstract class of sub-architecture processing
   * component that can write ontology-described entries to a working
   * memory.
   * 
   * @author nah
   */
  class WorkingMemoryWriterProcess : 
    public virtual WorkingMemoryAttachedComponent,
    public PushSender<cdl::WorkingMemoryEntry>,
    public PushSender<CASTWorkingMemoryEntry> {
  
  private:

    /**
     * @param _id
     * @param _type
     */
    void logMemoryAdd(const std::string & _id, 
		      const std::string & _subarchitectureID,
		      const std::string & _type) {
      logEvent(cdl::ui::ADD, getProcessIdentifier(), 
	       _subarchitectureID, _type, _id);
    }

    /**
     * @param _id
     */
    void logMemoryDelete(const std::string & _id,
			 const std::string & _subarchitectureID) {
      logEvent(cdl::ui::DELETE, getProcessIdentifier(), 
	       _subarchitectureID, "", _id);
    }
  
    /**
     * @param _id
     * @param _type
     */
    void logMemoryOverwrite(const std::string & _id, 
			    const std::string & _subarchitectureID,
			    const std::string & _type) {
      logEvent(cdl::ui::OVERWRITE, getProcessIdentifier(), 
	       _subarchitectureID, _type, _id);
    }


    /**
     * Push the input working memory entry to the working memory
     * component.
     * 
     * @param _pWME
     *            The working memory entry to send.
     */
    void pushToWorkingMemory(cdl::WorkingMemoryEntry * _pWME,
			     const cdl::OperationMode & _sync);

    /**
     * Push the input working memory entry to the working memory
     * component.
     * 
     * @param _pWME
     *            The working memory entry to send.
     */
    void pushToWorkingMemory(CASTWorkingMemoryEntry * _pWME,
			     const cdl::OperationMode & _sync);


    void flushWriteConnections();

    /**
     * @param _data The data to store. This is now owned by the working
     * memory.
     */
    template <class T> 
    void dataToWorkingMemory(const cdl::WorkingMemoryOperation & _op,
			     const std::string &_id, 
			     const std::string &_subarchitectureID, 
			     const std::string &_type,
			     T * _data,
			     const cdl::OperationMode & _sync) {

      try {

	if(m_pInputToLocalWorkingMemory) {
      
	  CASTWorkingMemoryItem * pWMI 
	    = new WorkingMemoryItem<T>(_id,_type, 0, _data);
      
	  cdl::WorkingMemoryAddress wma;
	  wma.m_id = CORBA::string_dup(_id.c_str());
	  wma.m_subarchitecture = CORBA::string_dup(_subarchitectureID.c_str());
      

	  CASTWorkingMemoryEntry *pCWME 
	    = new CASTWorkingMemoryEntry(getProcessIdentifier(),
					 _op, 
					 wma, 
					 //this creates a shared ptr that
					 //manages pWMI for us
					 pWMI); 
     

	  //       cout<<endl;
	  //       cout<<"WorkingMemoryWriterProcess: data ptr: count: "<<pCWME->item().use_count()<<endl;
	  //       cout<<endl;
	  
 
	  pushToWorkingMemory(pCWME, _sync); 
      
	}
	else {
	  const RemoteDataTranslator<T> * 
	    pTranslator = RemoteTranslatorManager::translator<T>();
      
	  //translate data to an any
	  CORBA::Any a;
	  pTranslator->translate(_data,a);
	  cdl::WorkingMemoryEntry * wme 
	    = new cdl::WorkingMemoryEntry();
      
	  wme->m_operation = _op;
	  wme->m_address.m_id = CORBA::string_dup(_id.c_str());
	  wme->m_address.m_subarchitecture = CORBA::string_dup(_subarchitectureID.c_str());
	  wme->m_type = CORBA::string_dup(_type.c_str());
	  wme->m_version = 0;
	  wme->m_data = a;
      
	  pushToWorkingMemory(wme, _sync); 
	}
      }
      catch(const CORBA::Exception &e){
	std::cerr<<"Caught CORBA::Exception in WorkingMemoryWriterProcess::dataToWorkingMemory"<<std::endl;
	std::cerr<<e._name()<<std::endl;
      }

    }

    /**
     * Connector used to write to a remote working memory.
     */
    PushConnectorOut<cdl::WorkingMemoryEntry> * m_pInputToRemoteWorkingMemory;

    /**
     * Connector used to write to a local working memory.
     */
    PushConnectorOut<CASTWorkingMemoryEntry> * m_pInputToLocalWorkingMemory;

    /**
     * Counter used to generate unique ids.
     */
    int m_dataCount;

    /**
     * Unique number assigned to this component.
     */
    int m_componentNumber;
    std::string m_componentNumberString;
  

  public:

    /**
     * Constructor.
     * 
     * @param _id
     *            Unique component id.
     */
    WorkingMemoryWriterProcess(const std::string &_id);
  
    /**
     * Empty destructor.
     */
    virtual ~WorkingMemoryWriterProcess();

    /**
     * Overrides the configure method from FrameworkProcess to use
     * _config to set the subarchitecture ID.
     * 
     * @param _config
     *            The ID of the subarchitecture which contains this
     *            component.
     */
    virtual void configure(std::map<std::string,std::string> & _config);


    /**
     * 
     * Sets the connector to the working memory component.
     * 
     * @param _connectionID The id of the connector.
     * @param _pOut The connector.
     * 
     */
    virtual void setPushConnector(const std::string & _connectionID, 
				  PushConnectorOut<cdl::WorkingMemoryEntry> * _pOut);
  
    /**
     * 
     * Sets the connector to the working memory component.
     * 
     * @param _connectionID The id of the connector.
     * @param _pOut The connector.
     * 
     */
    virtual void setPushConnector(const std::string & _connectionID,
				  PushConnectorOut<CASTWorkingMemoryEntry> * _pOut);
  



  protected:

    virtual void checkPrivileges(const std::string & _subarchitectureID) 
      throw (SubarchitectureProcessException) { 
      if(_subarchitectureID != m_subarchitectureID) {
	throw SubarchitectureProcessException(__HERE__,
					      "This component is not allowed to write to subarchitecture: %s",
					      + _subarchitectureID.c_str());
      }
    }

    /**
     * Add new data to working memory. The data will be stored with the given
     * id.
     * 
     * @param _id
     *            The id the data will be stored with.
     * @param _subarchitectureID
     *            The subarchitecture to write to.
     * @param _sync
     *            Whether to block until the write is complete.
     * @param _data
     *            The data itself
     * @throws SubarchitectureProcessException
     *             If this component does not have permission to write to the
     *             given subarchitecture, or if other errors occur.
     * @throws AlreadyExistsOnWMException
     *             If an entry exists at the given id.
     */
    template <class T>
    void addToWorkingMemoryHelper(const std::string &_id, 
				  const std::string &_subarchitectureID,
				  T * _data,
				  const cdl::OperationMode & _sync) 
      throw (AlreadyExistsOnWMException, SubarchitectureProcessException) { 

      assert(!_id.empty());//id must not be empty
      assert(!_subarchitectureID.empty());//subarch must not be empty
      assert(_data != NULL);//data must not be null

      checkPrivileges(_subarchitectureID);
        
      if (existsOnWorkingMemory(_id, _subarchitectureID)) {
	throw AlreadyExistsOnWMException(makeAdress(_subarchitectureID,_id),
					 __HERE__,
					 "Entry already exists for address for add.. Was looking for id %s in sa %s",
					 _id.c_str(), _subarchitectureID.c_str());
      }

      std::string type = typeName<T>();

      logMemoryAdd(_id, _subarchitectureID, type);

      //#bug 52, testcase 2: If we are already versioning this data
      //it's ok. This means we had previously written to this address.
      
      //if not versioned, start doing so
      if(!isVersioned(_id)) {
	debug("is not versioned: %s",_id.c_str());
	startVersioning(_id);
      }
      //if it's already versioned, then we need to update our numbers      
      else {
	debug("re-adding to working memory with id %s",_id.c_str());	
	//get the last known version number, which we store + 1
	storeVersionNumber(_id, (getVersionNumber(_id,_subarchitectureID) + 1));
      }
      
      dataToWorkingMemory(cdl::ADD, _id, _subarchitectureID, type, _data, _sync);    
      
      
      
    }


    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling process once the
     * method has been called. The data will be stored with the given
     * id.
     *
     * @param _id
     *            The id the data will be stored with
     * @param _subarchitectureID
     *            The subarchitecture to write to.
     * @param _sync
     *            Whether to block until the operation has completed.
     * @param _data
     *            The data itself
     * 
     * @throws SubarchitectureProcessException
     *             If a datatype error occurs or if this component does not have
     *             permission to write to the target subarchitecture.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist to be overwritten.
     * @throws ConsistencyException
     *             if this component does not have the most recent version of
     *             the data at the given id.
     */
    template <class T>
    void overwriteWorkingMemoryHelper(const std::string &_id, 
				      const std::string &_subarch,
				      T * _data,
				      const cdl::OperationMode & _sync) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException, ConsistencyException) { 

      assert(!_id.empty());//id must not be empty
      assert(!_subarch.empty());//subarch must not be empty
      assert(_data != NULL);//data must not be null

      checkPrivileges(_subarch);
        
      if (!existsOnWorkingMemory(_id, _subarch)) {
	throw DoesNotExistOnWMException(makeAdress(_subarch, _id),
					__HERE__,
					"Entry does not exist for overwriting. Was looking for id %s in sa %s",
					_id.c_str(), _subarch.c_str());
      }


      // if we don't have an overwrite lock
      if (!holdsOverwriteLock(_id, _subarch)) {
	
	if (!isOverwritable(_id, _subarch)) {
	  throw PermissionException(makeAdress(_subarch,_id),
				    __HERE__,
				    "Overwrite not allowed on locked item: %s:%s",
				    _id.c_str(), _subarch.c_str());
	}

	// then check the consistency of the overwrite
	checkConsistency(_id, _subarch);
      } 
      else {

	//if we need to do a one-time check of the consistecny
	if (needsConsistencyCheck(_id, _subarch)) {
	  debug("one-time consistency check for locked item: %s:%s",
		_id.c_str(), _subarch.c_str());

	  // then check the consistency of the overwrite
	  checkConsistency(_id, _subarch);
	  consistencyChecked(_id, _subarch);
	  
	} 
	else {
	  debug("skipping consistency check for locked item: %s:%s",
		_id.c_str(), _subarch.c_str());
	}
      }

      std::string type = typeName<T>();

      logMemoryOverwrite(_id,_subarch,type);

      dataToWorkingMemory(cdl::OVERWRITE, _id, _subarch, type, _data, _sync);    

      // if we got this far, then we're allowed to update our local
      // version number for the id
      increaseStoredVersion(_id);
    }


    /**
     * Delete data from working memory with given id.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @param _subarchitectureID
     *            The subarchitecture to write to.
     * @param _sync
     *            Whether to block until the operation completes.
     * @throws SubarchitectureProcessException
     *             if comms fail, if a datatype error occurs, or if this component does not have permission to write to the target subarchitecture.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */
    void deleteFromWorkingMemoryHelper(const std::string &_id, 
				       const std::string & _subarchitectureID,
				       const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException);
  

  public:


  

    /**
     * Add the data object to working memory with the given id and type
     * information. The object memory no longer belongs to the calling
     * process once the method has been called.  *The data will be
     * stored with the given id.
     * @param _id
     *            The id the data will be stored with.
     * @param _sync
     *            Whether to block until the write is complete.
     * @param _data
     *            The data itself
     * @throws SubarchitectureProcessException
     *             If this component does not have permission to write to the
     *             given subarchitecture, or if other errors occur.
     * @throws AlreadyExistsOnWMException
     *             If an entry exists at the given id.
     * 
     */
    template <class T>
    void addToWorkingMemory(const std::string &_id, 
			    T * _data,
			    const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (AlreadyExistsOnWMException, SubarchitectureProcessException) {    
      addToWorkingMemoryHelper(_id,m_subarchitectureID,_data,_sync);
    }







    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling process once the
     * method has been called. The data will be stored with the given
     * id.
     * 
     * @param _id
     *            The id the data will be stored with
     * @param _sync
     *            Whether to block until the operation has completed.
     * @param _data
     *            The data itself
     * 
     * @throws SubarchitectureProcessException
     *             If a datatype error occurs or if this component does not have
     *             permission to write to the target subarchitecture.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist to be overwritten.
     * @throws ConsistencyException
     *             if this component does not have the most recent version of
     *             the data at the given id.
     */
    template <class T>
    void overwriteWorkingMemory(const std::string &_id, 
				T * _data,
				const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException, ConsistencyException) {
      overwriteWorkingMemoryHelper(_id,m_subarchitectureID,_data,_sync);
    }  


    /**
     * Delete data from working memory with given id.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @param _sync
     *            Whether to block until the operation completes.
     * @throws SubarchitectureProcessException
     *             if comms fail.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */    
    virtual void deleteFromWorkingMemory(const std::string &_id,
					 const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException);



    /**
     * Generate a new unique id for a working memory entry.
     * 
     * @return A unique data id.
     */
    virtual std::string newDataID();



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
    void lockEntry(const std::string & _id, 
		   const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);


    /**
     * Try to obtain a lock on a working memory entry with the given
     * permissions. This will block until the desired lock is obtained.
     * 
     * @param _id
     * @param _subarch
     * @param _permissions
     * @throws DoesNotExistOnWMException
     */
    void lockEntry(const std::string & _id, 
		   const std::string & _subarch,
		   const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);



    /**
     * Try to obtain a lock on a working memory entry with the given
     * permissions. This will block until the desired lock is obtained.
     * 
     * @param _wma
     * @param _permissions
     * @throws DoesNotExistOnWMException
     */
    void lockEntry(const cdl::WorkingMemoryAddress & _wma,
		   const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);

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
    bool tryLockEntry(const std::string & _id,
		      const cdl::WorkingMemoryPermissions & _permissions) 
      throw(DoesNotExistOnWMException);



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
    bool tryLockEntry(const std::string & _id, 
		      const std::string & _subarch,
		      const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);

    
    /**
     * Try to obtain a lock on a working memory entry. This will return true if
     * the item is locked, or false if not. This method does not block.
     * 
     * @param _wma
     * @param _permissions
     * @return
     * @throws SubarchitectureProcessException
     */
    bool tryLockEntry(const cdl::WorkingMemoryAddress & _wma,
		      const cdl::WorkingMemoryPermissions & _permissions)
      throw(DoesNotExistOnWMException);



    void unlockEntry(const std::string & _id)
        throw(DoesNotExistOnWMException);;

    /**
     * Unlock the given working memory entry.
     * 
     * @param _id
     * @param _subarch
     * @throws DoesNotExistOnWMException
     */
    void unlockEntry(const std::string & _id, 
		     const std::string & _subarch)
      throw(DoesNotExistOnWMException);


    /**
     * Unlock the given working memory entry.
     * 
     * @param _wma
     * @throws DoesNotExistOnWMException
     */
    void unlockEntry(const cdl::WorkingMemoryAddress & _wma)
      throw(DoesNotExistOnWMException);



    
    


  };

} //namespace cast

#endif
