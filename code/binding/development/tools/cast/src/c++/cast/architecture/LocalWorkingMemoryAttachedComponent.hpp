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

#ifndef CAST_LOCAL_WORKING_MEMORY_ATTACHED_COMPONENT_H_
#define CAST_LOCAL_WORKING_MEMORY_ATTACHED_COMPONENT_H_




#include <cast/core/CASTCore.hpp>
#include <cast/core/CASTComponentPermissionsMap.hpp>
#include <cast/architecture/ConsistencyException.hpp>
#include <balt/interface/BALTInterface.hpp>
#include <balt/core/StringMap.hpp>

#include "cast/architecture/DoesNotExistOnWMException.hpp"

#include <list>
#include <vector>

namespace cast {

  typedef StringMap<int>::map IntMap;

  /**
   * The absolute simplest component that can be attached to a working
   * memory. Just checks whether an entry exists on a local working memory or
   * not.
   * 
   * @author nah
   */
  class LocalWorkingMemoryAttachedComponent :
    public CASTProcessingComponent, 
    public PullSender< bool >, 
    public PullSender< int >,
    public PullSender< cdl::WorkingMemoryPermissions >
  {
    

  public:   

    /**
     * Constructor.
     *
     * @param _id The unique identifier of this process.
     */
    LocalWorkingMemoryAttachedComponent(const std::string &_id);  

    /**
     * Destructor.
     */
    virtual ~LocalWorkingMemoryAttachedComponent(){};

    virtual void 
    setPullConnector(const std::string & _connectionID, 
		     PullConnectorOut< bool > * _pOut);

    virtual void 
    setPullConnector(const std::string & _connectionID, 
		     PullConnectorOut< int > * _pOut);

    virtual void 
    setPullConnector(const std::string & _connectionID, 
		     PullConnectorOut< cdl::WorkingMemoryPermissions > * _pOut);


    /**
     * Determines whether an entry exists on working memory at the
     * given subarchitecture-local address.
     * 
     * @param _id
     *            The id for the entry in working memory.
     *
     * @return True if the entry exists, otherwise false.
     */

    bool existsOnWorkingMemory(const std::string & _id);

    /**
     * Get a count of the number of times the working memory entry at
     * the given local address has been overwritten.
     * 
     * @param _id
     *            The id for the entry in working memory.
     *
     * @return The version count of the entry. This will be 0 if it
     * has never been overwritten.
     *
     * @throws DoesNotExistOnWMException if the entry has never
     * existed on working memory.
     *
     * @remark Interface change: Renamed to reflect new role, behaviour still
     *         the same: getOverwriteCount -> getVersionNumber.
     */
    int getVersionNumber(const std::string & _id) throw (DoesNotExistOnWMException);

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
    virtual void lockEntry(const std::string & _id, 
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
    virtual bool tryLockEntry(const std::string & _id,
		      const cdl::WorkingMemoryPermissions & _permissions) 
      throw(DoesNotExistOnWMException);


    /**
     * Unlock the given working memory entry.
     * 
     * @param _id
     * @throws WMException
     */
    virtual void unlockEntry(const std::string & _id)
    throw(DoesNotExistOnWMException);


  private:
    /**
      Connection used to get exists queries
     */  
    PullConnectorOut< bool > * m_pExistsConnector;


    /**
     * Connection used for permissions queries.
     */
    PullConnectorOut< cdl::WorkingMemoryPermissions > * m_lockConnector;

    IntMap m_versionNumbers;

    //need to initialise later, so use smart ptr
    boost::shared_ptr<CASTComponentPermissionsMap> m_permissions;

  protected:

    /**
     * Connection used to get update numbers
     */  
    PullConnectorOut< int > * m_pVersionNumberConnector;

    /**
     * Associate the given id with the given version number. This should not be
     * called by user code.
     * 
     * @param _id
     * @param _version
     */
    void storeVersionNumber(const std::string & _id, int _version);


    /**
     * Get the version number currently stored in the component for the wm id.
     * 
     * @param _id
     * @return
     * @throws ConsistencyException
     *             if the id is not stored.
     */
    int getStoredVersionNumber(const std::string & _id) const throw(ConsistencyException);

    /**
     * Increment the stored version number by 1.
     * 
     * @param _id id of entry
     * @throws ConsistencyException if _id is not stored in the versioning system
     */
    void increaseStoredVersion(const std::string & _id) throw(ConsistencyException);

    /**
     * Add the given id to the versioning system with version 0.
     * 
     * @param _id
     */
    void startVersioning(const std::string & _id);

    /**
     * Updates the stored id with a new version number.
     * 
     * @param _id
     * @param _newVersion
     */
    void updateVersion(const std::string & _id, int _newVersion);      

    /**
     * Check whether the given id is versioned.
     * 
     * @param _id
     * @return
     */
    bool isVersioned(const std::string & _id) const;

    /**
     * Removes the given id from the versioning system.
     * 
     * @param _id
     */
    void stopVersioning(const std::string & _id);

    /**
     * Removes the given id from the internal versioning system storage.
     * 
     * @param _id
     */
    void removeVersionNumber(const std::string & _id);


    /**
     * Checks whether this component has read the more recent version of the
     * data at this working memory address.
     * 
     * @param _id
     * @return
     * @throws ConsistencyException
     *             if the id is not versioned.
     * @throws DoesNotExistOnWMException
     *             if the _id does not exist on wm
     */
    bool haveLatestVersion(const std::string & _id)
      throw (ConsistencyException, DoesNotExistOnWMException);

    /**
     * Checks whether this component has read the more recent version of the
     * data at this working memory address, and throws and exception if not.
     * 
     * @param _id
     * @throws ConsistencyException
     *     *             if the id is not versioned, or if the
     * @throws DoesNotExistOnWMException
     *             if the _id does not exist on wm
     */
    void checkConsistency(const std::string & _id) 
      throw(ConsistencyException, DoesNotExistOnWMException);



    /**
     * Ignore helper please!
     */
    bool lockEntryHelper(const std::string & _id, const std::string & _subarch,
			 const cdl::WorkingMemoryPermissions & _permissions, 
			 const cdl::OperationMode & _op) 
      throw(DoesNotExistOnWMException);




    void unlockEntryHelper(const std::string & _id, 
			   const std::string & _subarch)
    throw(DoesNotExistOnWMException);


    /**
     * Checks whether this component holds any kind of lock on the given working
     * memory item.
     * 
     * @param _id
     * @return
     */
    bool holdsLock(const std::string & _id) const;


    /**
     * Checks whether this component holds any kind of lock on the given working
     * memory item.
     * 
     * @param _id
     * @param _subarch
     * @return
     */
    bool holdsLock(const std::string & _id, const std::string & _subarch) const;

    /**
     * Checks whether this component holds an overwrite lock on the given
     * working memory item.
     * 
     * @param _id
     * @return
     */
    bool holdsOverwriteLock(const std::string & _id) const;

    /**
     * Checks whether this component holds a delete lock on the given working
     * memory item.
     * 
     * @param _id
     * @return
     */
    bool holdsDeleteLock(const std::string & _id) const;

    /**
     * Checks whether this component holds a read lock on the given working
     * memory item.
     * 
     * @param _id
     * @return
     */    
    bool holdsReadLock(const std::string & _id) const;


    /**
     * Checks whether this component holds an overwrite lock on the given
     * working memory item.
     * 
     * @param _id
     * @param _subarch
     * @return
     */
    bool holdsOverwriteLock(const std::string & _id, const std::string & _subarch) const;

    /**
     * Checks whether this component holds a delete lock on the given working
     * memory item.
     * 
     * @param _id
     * @param _subarch
     * @return
     */
    bool holdsDeleteLock(const std::string & _id, const std::string & _subarch) const;


    /**
     * Checks whether this component holds a read lock on the given working
     * memory item.
     * 
     * @param _id
     * @param _subarch
     * @return
     */
    bool holdsReadLock(const std::string & _id, const std::string & _subarch) const;

    
    bool needsConsistencyCheck(const std::string & _id) const;
    bool needsConsistencyCheck(const std::string & _id, 
			       const std::string & _subarch) const;

    void consistencyChecked(const std::string & _id);
    void consistencyChecked(const std::string & _id, const std::string & _subarch);
    

    /**
     * Ignore helper please!
     * 
     * @param _id
     * @param _subarch
     * @return
     * @throws WMException
     */
    cdl::WorkingMemoryPermissions getPermissionsHelper(const std::string & _id,
						       const std::string & _subarch) const
      throw(DoesNotExistOnWMException);
    
    
    /**
     * Gets the permissions currently set on the given working memory item.
     * 
     * @param _id
     * @return
     * @throws WMException
     */
    cdl::WorkingMemoryPermissions getPermissions(const std::string & _id) const
      throw(DoesNotExistOnWMException);

    
    /**
     * Checks whether a given item on working memory is currently overwritable
     * by this component.
     * 
     * @param _id
     * @return
     * @throws WMException
     */
    bool isOverwritable(const std::string & _id) const throw(DoesNotExistOnWMException);   


    /**
     * Checks whether a given item on working memory is currently deletable by
     * this component.
     * 
     * @param _id
     * @return
     * @throws WMException
     */
    bool isDeletable(const std::string & _id) const throw(DoesNotExistOnWMException);


    
    /**
     * Checks whether a given item on working memory is currently readable by
     * this component.
     * 
     * @param _id
     * @return
     * @throws WMException
     */
    bool isReadable(const std::string & _id) const throw(DoesNotExistOnWMException);

  };

} //namespace cast

#endif
