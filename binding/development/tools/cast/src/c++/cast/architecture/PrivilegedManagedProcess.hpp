
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

#ifndef CAST_PRIVILEGED_GOAL_DRIVEN_PROCESS_H_
#define CAST_PRIVILEGED_GOAL_DRIVEN_PROCESS_H_

#include "ManagedProcess.hpp"

#include <map>

namespace cast {

  typedef StringMap<cdl::InformationProcessingTask *>::map IPTaskMap;

  /**
   * An abstract class to represent a component in a subarchitecture that
   * can read and write to a working memory, and has its operations
   * controlled by a task manager.
   * 
   * @author nah
   */
  class PrivilegedManagedProcess : 
    public ManagedProcess {
  
  public:
  
    /**
     * Constructor.
     * 
     * @param _id Unique component id.
     */
    PrivilegedManagedProcess(const std::string &_id) :
      WorkingMemoryAttachedComponent(_id),
      ManagedProcess(_id) {};
  
    /**
     * Destructor, cleans up proposed tasks.
     */
    virtual ~PrivilegedManagedProcess() {};
  
  

    /**
     * Add data object to working memory with the given id. The object
     * memory no longer belongs to the calling process once the method
     * has been called. The data will be stored with the given id.
     * 
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
     */
    template <class T>
    void addToWorkingMemory(const std::string &_id, 
			    T * _data,
			    const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (AlreadyExistsOnWMException, SubarchitectureProcessException) { 
    
      addToWorkingMemory(_id,
			 m_subarchitectureID,
			 _data,
			 _sync);
    }
    /**
     * Add data object to working memory with the given id. The object
     * memory no longer belongs to the calling process once the method
     * has been called. The data will be stored with the given id.
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
    void addToWorkingMemory(const std::string &_id, 
			    const std::string &_subarchitectureID,
			    T * _data,
			    const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (AlreadyExistsOnWMException, SubarchitectureProcessException) { 
    
      addToWorkingMemoryHelper(_id,
			       _subarchitectureID,
			       _data,
			       _sync);
    }

    /**
     * Add data object to working memory with the given id. The object
     * memory no longer belongs to the calling process once the method
     * has been called. The data will be stored with the given id.
     * 
     * @param _wma The address  to write to.
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
    void addToWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
			    T * _data,
			    const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException) { 
    
      addToWorkingMemory(std::string(_wma.m_id),
			 std::string(_wma.m_subarchitecture),
			 _data,
			 _sync);
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
    
      overwriteWorkingMemory(_id,
			     m_subarchitectureID,
			     _data,
			     _sync);
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
    void overwriteWorkingMemory(const std::string &_id, 
				const std::string &_subarchitectureID,
				T * _data,
				const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException, ConsistencyException) { 
    
      overwriteWorkingMemoryHelper(_id,
				   _subarchitectureID,
				   _data,
				   _sync);
    }

    /**
     * Overwrite data object in working memory with the given id. The
     * object memory no longer belongs to the calling process once the
     * method has been called. The data will be stored with the given
     * id.
     *
     * @param _wma The address to write to.
     * @param _type
     *            The ontological type of the data.
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
    void overwriteWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
				T * _data,
				const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException, ConsistencyException) { 
    
      overwriteWorkingMemory(std::string(_wma.m_id),
			     std::string(_wma.m_subarchitecture),
			     _data,
			     _sync);
    }

    /**
     * Delete data from working memory with given id. Redefinition to
     * get around hiding.
     * 
     * @param _id
     *            The id of the working memory entry to be deleted.
     * @param _sync
     *            Whether to block until the operation completes.
     * @throws SubarchitectureProcessException
     *             if comms fail, if a datatype error occurs, or if this component does not have permission to write to the target subarchitecture.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */
    virtual void deleteFromWorkingMemory(const std::string &_id, 
					 const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException) { 
      //logf("deleteFromWorkingMemory: %s, %s", _id.c_str(), _subarchitectureID.c_str());
      deleteFromWorkingMemory(_id,m_subarchitectureID,_sync);
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
    virtual void deleteFromWorkingMemory(const std::string &_id, 
					 const std::string & _subarchitectureID,
					 const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException) { 
      //logf("deleteFromWorkingMemory: %s, %s", _id.c_str(), _subarchitectureID.c_str());
      deleteFromWorkingMemoryHelper(_id,_subarchitectureID,_sync);
    }


    /**
     * Delete data from working memory with given id.
     * 
     * @param _wma The address to write to.
     * @param _sync
     *            Whether to block until the operation completes.
     * @throws SubarchitectureProcessException
     *             if comms fail, if a datatype error occurs, or if this component does not have permission to write to the target subarchitecture.
     * @throws DoesNotExistOnWMException
     *             if the given id does not exist on working memory.
     */
    virtual void deleteFromWorkingMemory(const cdl::WorkingMemoryAddress & _wma,
					 const cdl::OperationMode & _sync = cdl::NON_BLOCKING) 
      throw (DoesNotExistOnWMException, SubarchitectureProcessException) { 
      deleteFromWorkingMemory(std::string(_wma.m_id),
			      std::string(_wma.m_subarchitecture),
			      _sync);
    }
  
  protected:

    /**
     * Allow all writes.
     */
    virtual void checkPrivileges(const std::string & _subarchitectureID) throw (SubarchitectureProcessException) {}

  
  
  };
  
} //namespace cast

#endif
