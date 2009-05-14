/*
 * CAST - The CoSy Architecture Schema Toolkit Copyright (C) 2006-2007
 * Nick Hawes This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either version
 * 2.1 of the License, or (at your option) any later version. This
 * library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package cast.architecture.subarchitecture;

import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;

/**
 * Extends {@link ManagedProcess} to add methods to write to any working memory
 * in the architecture.
 * 
 * @author nah
 */
public abstract class PrivilegedManagedProcess extends ManagedProcess {

	/**
	 * @param _id
	 */
	public PrivilegedManagedProcess(String _id) {
		super(_id);
	}

	/**
	 * Allow all writes to subarchitectures.
	 * 
	 * @throws SubarchitectureProcessException
	 *             never.
	 */
	@Override
	protected void checkPrivileges(String _subarchitectureID)
			throws SubarchitectureProcessException {
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 *            The id the data will be stored with.
	 * @param _subarchitectureID
	 *            The subarchitecture to write to.
	 * @param _type
	 *            The ontological type of the data.
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
	public <T> void addToWorkingMemory(String _id,
			String _subarchitectureID, T _data, OperationMode _sync)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {

		addToWorkingMemoryHelper(_id, _subarchitectureID, _data, _sync);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 *            The id the data will be stored with.
	 * @param _subarchitectureID
	 *            The subarchitecture to write to.
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself
	 * @throws SubarchitectureProcessException
	 *             If this component does not have permission to write to the
	 *             given subarchitecture, or if other errors occur.
	 * @throws AlreadyExistsOnWMException
	 *             If an entry exists at the given id.
	 */
	public <T> void addToWorkingMemory(String _id,
			String _subarchitectureID, T _data)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		addToWorkingMemory(_id, _subarchitectureID, _data,
				OperationMode.NON_BLOCKING);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _wma
	 *            The address to write the data to.
	 * @param _type
	 *            The ontological type of the data.
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
	public <T> void addToWorkingMemory(WorkingMemoryAddress _wma, T _data,
			OperationMode _sync) throws AlreadyExistsOnWMException,
			SubarchitectureProcessException {

		addToWorkingMemory(_wma.m_id, _wma.m_subarchitecture, _data, _sync);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 * @param _type
	 * @param _data
	 * @throws SubarchitectureProcessException
	 */
	public <T> void addToWorkingMemory(WorkingMemoryAddress _wma, T _data)
			throws SubarchitectureProcessException {
		addToWorkingMemory(_wma, _data, OperationMode.NON_BLOCKING);
	}

	/**
	 * Delete data from working memory with given id.
	 * 
	 * @param _id
	 *            The id of the working memory entry to be deleted.
	 * @param _subarchitectureID
	 *            The subarchitecture to write to.
	 * @throws SubarchitectureProcessException
	 *             If comms fail or if this component does not have permission
	 *             to write to the target subarchitecture.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	public void deleteFromWorkingMemory(String _id, String _subarchitectureID)
			throws SubarchitectureProcessException {
		deleteFromWorkingMemory(_id, _subarchitectureID,
				OperationMode.NON_BLOCKING);
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
	 *             If comms fail or if this component does not have permission
	 *             to write to the target subarchitecture.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	public void deleteFromWorkingMemory(String _id,
			String _subarchitectureID, OperationMode _sync)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		deleteFromWorkingMemoryHelper(_id, _subarchitectureID, _sync);
	}

	/**
	 * Delete data from working memory with given id.
	 * 
	 * @param _wma
	 *            The address to write to.
	 * @param _sync
	 *            Whether to block until the operation completes.
	 * @throws SubarchitectureProcessException
	 *             If comms fail or if this component does not have permission
	 *             to write to the target subarchitecture.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	public void deleteFromWorkingMemory(WorkingMemoryAddress _wma)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		deleteFromWorkingMemory(_wma, OperationMode.NON_BLOCKING);
	}

	/**
	 * Delete data from working memory with given id.
	 * 
	 * @param _wma
	 *            The address to write to.
	 * @param _sync
	 *            Whether to block until the operation completes.
	 * @throws SubarchitectureProcessException
	 *             If comms fail or if this component does not have permission
	 *             to write to the target subarchitecture.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	public void deleteFromWorkingMemory(WorkingMemoryAddress _wma,
			OperationMode _sync) throws DoesNotExistOnWMException,
			SubarchitectureProcessException {
		deleteFromWorkingMemory(_wma.m_id, _wma.m_subarchitecture, _sync);
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _subarchitectureID
	 *            The subarchitecture to write to.
	 * @param _type
	 *            The ontological type of the data.
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
	public <T> void overwriteWorkingMemory(String _id,
			String _subarchitectureID, T _data)
			throws DoesNotExistOnWMException, SubarchitectureProcessException,
			ConsistencyException {

		overwriteWorkingMemory(_id, _subarchitectureID, _data,
				OperationMode.NON_BLOCKING);
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _subarchitectureID
	 *            The subarchitecture to write to.
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
	public <T> void overwriteWorkingMemory(String _id,
			String _subarchitectureID, T _data, OperationMode _sync)
			throws SubarchitectureProcessException, ConsistencyException {

		overwriteWorkingMemoryHelper(_id, _subarchitectureID, _data, _sync);

	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _wma
	 *            The address to write to.
	 * @param _type
	 *            The ontological type of the data.
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
	public <T> void overwriteWorkingMemory(WorkingMemoryAddress _wma, T _data)
			throws DoesNotExistOnWMException, SubarchitectureProcessException,
			ConsistencyException {

		overwriteWorkingMemory(_wma, _data, OperationMode.NON_BLOCKING);
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _wma
	 *            The address to write to.
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
	public <T> void overwriteWorkingMemory(WorkingMemoryAddress _wma,
			T _data, OperationMode _sync) throws DoesNotExistOnWMException,
			SubarchitectureProcessException, ConsistencyException {

		overwriteWorkingMemory(_wma.m_id, _wma.m_subarchitecture, _data, _sync);

	}

}
