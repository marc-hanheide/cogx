/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2009 Nick Hawes
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

#ifndef CAST_WORKING_MEMORY_WRITER_COMPONENT_H_
#define CAST_WORKING_MEMORY_WRITER_COMPONENT_H_

#include <WorkingMemoryAttachedComponent.hpp>

namespace cast {

  /**
   * Class to define an abstract class of subarchitecture componenting
   * component that can write entries to a working
   * memory.
   * 
   * @author nah
   */
  class WorkingMemoryWriterComponent : 
    public virtual WorkingMemoryAttachedComponent
 {
  
  private:


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
    WorkingMemoryWriterComponent();
  
    /**
     * Empty destructor.
     */
    virtual ~WorkingMemoryWriterComponent();

    /**
     * Overrides the configure method from FrameworkComponent to use
     * _config to set the subarchitecture ID.
     * 
     * @param _config
     *            The ID of the subarchitecture which contains this
     *            component.
     */
    virtual 
    void 
    configureInternal(const std::map<std::string,std::string> & _config);



//     virtual void checkPrivileges(const std::string & _subarchitectureID) 
//       throw (SubarchitectureComponentException) { 
//       if(_subarchitectureID != getSubarchitectureID()) {
// 	throw SubarchitectureComponentException(__HERE__,
// 					      "This component is not allowed to write to subarchitecture: %s",
// 					      + _subarchitectureID.c_str());
//       }
//     }




    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling component once the
     * method has been called. The data will be stored with the given
     * id.
     * 
     * @param _id
     *            The id the data will be stored with
     * @param _data
     *            The data itself
     * 
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist to be overwritten.
     * @throws ConsistencyException
     *             if this component does not have the most recent version of
     *             the data at the given id.
     */
    template <class T>
    void 
    overwriteWorkingMemory(const std::string &_id, 
			   IceInternal::Handle<T>  _data) 
      throw (DoesNotExistOnWMException, ConsistencyException, PermissionException) {
      overwriteWorkingMemory(_id,getSubarchitectureID(),_data);
    }  

    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling component once the
     * method has been called. The data will be stored with the given
     * id.
     * 
     * @param _wma
     *            The address the data will be stored at
     * @param _data
     *            The data itself
     * 
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist to be overwritten.
     * @throws ConsistencyException
     *             if this component does not have the most recent version of
     *             the data at the given id.
     */
    template <class T>
    void 
    overwriteWorkingMemory(const cdl::WorkingMemoryAddress & _wma, 
			   IceInternal::Handle<T>  _data) 
      throw (DoesNotExistOnWMException, ConsistencyException, PermissionException, UnknownSubarchitectureException) {
      overwriteWorkingMemory(_wma.id,_wma.subarchitecture,_data);
    }  


    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling component once the
     * method has been called. The data will be stored with the given
     * id.
     *
     * @param _id
     *            The id the data will be stored with
     * @param _subarchitectureID
     *            The subarchitecture to write to.
     * @param _data
     *            The data itself
     * 
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist to be overwritten.
     * @throws ConsistencyException
     *             if this component does not have the most recent version of
     *             the data at the given id.
     */
    template <class T>
    void 
    overwriteWorkingMemory(const std::string &_id, 
			   const std::string &_subarch,
			   IceInternal::Handle<T>  _data) 
      throw (DoesNotExistOnWMException, ConsistencyException, PermissionException, UnknownSubarchitectureException) { 

      assert(!_id.empty());//id must not be empty
      assert(!_subarch.empty());//subarch must not be empty
      assert(_data);//data must not be null

      //checkPrivileges(_subarch);

      // do the check here as it may save a lot of hassle later
      if (!existsOnWorkingMemory(_id, _subarch)) {
	throw DoesNotExistOnWMException(exceptionMessage(__HERE__,
							 "Entry does not exist for overwriting. Was looking for id %s in sa %s",
							 _id.c_str(), _subarch.c_str()),
					makeWorkingMemoryAddress(_subarch, _id));
      }


      // if we don't have an overwrite lock
      if (!holdsOverwriteLock(_id, _subarch)) {
	
	if (!isOverwritable(_id, _subarch)) {
	  throw PermissionException(exceptionMessage(__HERE__,
						     "Overwrite not allowed on locked item: %s:%s",
						     _id.c_str(), _subarch.c_str()),
				    makeWorkingMemoryAddress(_subarch,_id));
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

      const std::string & type(typeName<T>());

      //logMemoryOverwrite(_id,_subarch,type);

      m_workingMemory->overwriteWorkingMemory(_id,_subarch, type, getComponentID(), _data);

      // if we got this far, then we're allowed to update our local
      // version number for the id
      increaseStoredVersion(_id);
    }


   /**
     * Delete data from working memory with given id.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */    
    virtual 
    void 
    deleteFromWorkingMemory(const std::string &_id) 
      throw (DoesNotExistOnWMException, PermissionException, PermissionException) {
      deleteFromWorkingMemory(_id, getSubarchitectureID());
    }

   /**
     * Delete data from working memory with given id.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */    
    virtual 
    void 
    deleteFromWorkingMemory(const cdl::WorkingMemoryAddress &_wma) 
      throw (DoesNotExistOnWMException, PermissionException, PermissionException, UnknownSubarchitectureException) {
      deleteFromWorkingMemory(_wma.id,_wma.subarchitecture);
    }


    /**
     * Delete data from working memory with given id.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @param _subarchitectureID
     *            The subarchitecture to write to.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */
   virtual
   void 
   deleteFromWorkingMemory(const std::string &_id, 
			   const std::string & _subarch) 
     throw (DoesNotExistOnWMException, PermissionException, PermissionException, UnknownSubarchitectureException);
  



  

//    /**
//     * Add the data object to working memory with the given id and type
//     * information. The object memory no longer belongs to the calling
//     * component once the method has been called.  *The data will be
//     * stored with the given id.
//     * @param _id
//     *            The id the data will be stored with.
//     * @param _sync
//     *            Whether to block until the write is complete.
//     * @param _data
//     *            The data itself
//     * @throws AlreadyExistsOnWMException
//     *             If an entry exists at the given id.
//     * 
//     */
//    template <class T>
//    void addToWorkingMemory(const std::string &_id, 
// 			   T * _data,
// 			   const cdl::OperationMode & _sync = cdl::NONBLOCKING) 
//      throw (AlreadyExistsOnWMException) {    
//      addToWorkingMemory(_id,getSubarchitectureID(),_data);
//    }
   

   /**
    * Add new data to working memory. The data will be stored with the given
    * id.
    * 
    * @param _id
    *            The id the data will be stored with.
    * @param _subarchitectureID
    *            The subarchitecture to write to.
    * @param _data
    *            The data itself. Must be a ref-counted pointer to an instance of an Ice class.
    * @throws AlreadyExistsOnWMException
    *             If an entry exists at the given id.
    */
   template <class T>
   void addToWorkingMemory(const std::string &_id, 
			   IceInternal::Handle<T>  _data) 
     throw (AlreadyExistsOnWMException) { 
     addToWorkingMemory(_id,getSubarchitectureID(),_data);
   }

   /**
    * Add new data to working memory. The data will be stored with the given
    * id.
    * 
    * @param _wma
    *            The address the data will be stored at.
    * @param _data
    *            The data itself. Must be a ref-counted pointer to an instance of an Ice class.
    * @throws AlreadyExistsOnWMException
    *             If an entry exists at the given id.
    */
   template <class T>
   void addToWorkingMemory(const cdl::WorkingMemoryAddress & _wma, 
			   IceInternal::Handle<T>  _data) 
     throw (AlreadyExistsOnWMException, UnknownSubarchitectureException) { 
     addToWorkingMemory(_wma.id,_wma.subarchitecture,_data);
   }
   
   /**
    * Add new data to working memory. The data will be stored with the given
    * id.
    * 
    * @param _id
    *            The id the data will be stored with.
    * @param _subarchitectureID
    *            The subarchitecture to write to.
    * @param _data
    *            The data itself. Must be a ref-counted pointer to an instance of an Ice class.
    * @throws AlreadyExistsOnWMException
    *             If an entry exists at the given id.
    */
   template <class T>
   void addToWorkingMemory(const std::string &_id, 
			   const std::string &_subarch,
			   IceInternal::Handle<T>  _data) 
     throw (AlreadyExistsOnWMException, UnknownSubarchitectureException) { 
     
     assert(!_id.empty());//id must not be empty
     assert(!_subarch.empty());//subarch must not be empty
     assert(_data);//data must not be null
     
     //UPGRADE not sure what to do with this now
     //checkPrivileges(_subarch);
     
     std::string type(typeName<T>());
     
     //logMemoryAdd(_id, _subarch, type);
     
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
       storeVersionNumber(_id, (getVersionNumber(_id,_subarch) + 1));
     }
     m_workingMemory->addToWorkingMemory(_id,_subarch,type,getComponentID(),_data); 
   }
  
 
   /**
    * Generate a new unique id for a working memory entry.
    * 
    * @return A unique data id.
    */
   virtual std::string newDataID();
  };
  
} //namespace cast

#endif
