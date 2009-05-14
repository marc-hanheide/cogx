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
package cast.architecture.abstr;

import java.util.Properties;

import org.omg.CORBA.Any;
import org.omg.CORBA.ORB;

import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.PermissionException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.WMException;
import cast.cdl.COMPONENT_NUMBER_KEY;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.ui.ComponentEventType;
import cast.core.CASTUtils;
import cast.core.data.CASTWorkingMemoryEntry;
import cast.core.data.CASTWorkingMemoryItem;
import cast.core.interfaces.CWMEPushInterface.CWMEPushConnectorOut;
import cast.core.interfaces.CWMEPushInterface.CWMEPushSender;
import cast.core.interfaces.WMEPushInterface.WMEPushConnectorOut;
import cast.core.interfaces.WMEPushInterface.WMEPushSender;

/**
 * Defines a class of process that can both read from and write to a working
 * memory.
 * 
 * @author nah
 */
public abstract class WorkingMemoryReaderWriterProcess
		extends
			WorkingMemoryReaderProcess implements WMEPushSender, CWMEPushSender {

	protected int m_dataCount;

	protected WMEPushConnectorOut m_inputToRemoteWorkingMemory;

	protected CWMEPushConnectorOut m_inputToLocalWorkingMemory;

	private int m_componentNumber;

	private String m_componentNumberString;

	/**
	 * @param _id
	 */
	public WorkingMemoryReaderWriterProcess(String _id) {
		super(_id);
		m_dataCount = 0;
		m_inputToLocalWorkingMemory = null;
		m_inputToRemoteWorkingMemory = null;
		m_componentNumber = -1;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		String componentNumber = _config
				.getProperty(COMPONENT_NUMBER_KEY.value);
		assert (componentNumber != null);
		m_componentNumber = Integer.parseInt(componentNumber);
		m_componentNumberString = stringify(m_componentNumber);
	}

	/**
	 * @param _id
	 * @param _type
	 */
	protected void logMemoryAdd(String _id, String _subarchitectureID,
			String _type) {
		logEvent(ComponentEventType.ADD, getProcessIdentifier().toString(),
				_subarchitectureID, _type, _id);
	}

	/**
	 * @param _id
	 */
	private void logMemoryDelete(String _id, String _subarchitectureID) {
		logEvent(ComponentEventType.DELETE, getProcessIdentifier().toString(),
				_subarchitectureID, "", _id);
	}

	/**
	 * @param _id
	 * @param _type
	 */
	protected void logMemoryOverwrite(String _id, String _subarchitectureID,
			String _type) {
		logEvent(ComponentEventType.OVERWRITE, getProcessIdentifier()
				.toString(), _subarchitectureID, _type, _id);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 *            The id the data will be stored with.
	 * @param _data
	 *            The data itself
	 * @throws SubarchitectureProcessException
	 *             If various errors occur.
	 * @throws AlreadyExistsOnWMException
	 *             If an entry exists at the given id.
	 */
	public <T> void addToWorkingMemory(String _id, T _data)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {

		addToWorkingMemoryHelper(_id, m_subarchitectureID, _data,
				OperationMode.NON_BLOCKING);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 *            The id the data will be stored with.
	 * @param _data
	 *            The data itself
	 * @param _sync
	 *            Whether to block until the operation has completed.
	 * 
	 * @throws SubarchitectureProcessException
	 *             If various errors occur.
	 * @throws AlreadyExistsOnWMException
	 *             If an entry exists at the given id.
	 */
	public <T> void addToWorkingMemory(String _id, T _data, OperationMode _sync)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {

		addToWorkingMemoryHelper(_id, m_subarchitectureID, _data, _sync);
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
	protected <T> void addToWorkingMemoryHelper(String _id,
			String _subarchitectureID, T _data, OperationMode _sync)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarchitectureID.length() > 0 : "subarchitecture id must not be empty";

		checkPrivileges(_subarchitectureID);

		if (existsOnWorkingMemory(_id, _subarchitectureID)) {
			throw new AlreadyExistsOnWMException(new WorkingMemoryAddress(_id,
					_subarchitectureID),
					"Entry already exists for address for add.. Was looking for id "
							+ _id + " in sa " + _subarchitectureID);
		}

		String type = CASTUtils.typeName(_data);

		logMemoryAdd(_id, _subarchitectureID, type);

		// if not versioned, start doing so
		if (!isVersioned(_id)) {
			debug("is not versioned: " + _id);
			startVersioning(_id);
		}
		// if it's already versioned, then we need to update our numbers
		else {
			debug("re-adding to working memory with id: " + _id);
			// get the last known version number, which we store + 1
			storeVersionNumber(_id,
					(getVersionNumber(_id, _subarchitectureID) + 1));
		}

		dataToWorkingMemory(WorkingMemoryOperation.ADD, _id,
				_subarchitectureID, type, _data, _sync);
	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	protected void checkPrivileges(String _subarchitectureID)
			throws SubarchitectureProcessException {
		if (!m_subarchitectureID.equals(_subarchitectureID)) {
			throw new SubarchitectureProcessException(
					"This component is not allowed to write to subarchitecture: "
							+ _subarchitectureID);
		}
	}

	/**
	 * @param _id
	 * @param _type
	 * @param _data
	 * @param _sync
	 *            TODO
	 * @param <T>
	 * @throws SubarchitectureProcessException
	 */
	protected <T> void dataToWorkingMemory(WorkingMemoryOperation _op,
			String _id, String _subarchitectureID, String _type, T _data,
			OperationMode _sync) throws SubarchitectureProcessException {

		if (m_inputToLocalWorkingMemory != null) {
			CASTWorkingMemoryItem<T> wmi = new CASTWorkingMemoryItem<T>(_type,
					_data);
			CASTWorkingMemoryEntry<T> wme = new CASTWorkingMemoryEntry<T>(
					getProcessIdentifier(), _op, new WorkingMemoryAddress(_id,
							_subarchitectureID), wmi);
			pushToWorkingMemory(wme, _sync);
		}
		else if (m_inputToRemoteWorkingMemory != null) {
			try {
				Any anyData = RemoteDataTranslator.translateToAny(_data);
				WorkingMemoryEntry wme = new WorkingMemoryEntry(_op,
						new WorkingMemoryAddress(_id, _subarchitectureID),
						_type, 0, anyData);
				pushToWorkingMemory(wme, _sync);

			}
			catch (FrameworkDataTranslatorException e) {
				throw new SubarchitectureProcessException(
						"Unable to translate working memory input.", e);
			}
		}
		else {
			throw new SubarchitectureProcessException(
					"Connection to working memory has not been established");
		}

	}

	/**
	 * Delete data from working memory with given id.
	 * 
	 * @param _id
	 *            The id of the working memory entry to be deleted.
	 * @throws SubarchitectureProcessException
	 *             if comms fail.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	public void deleteFromWorkingMemory(String _id)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		deleteFromWorkingMemory(_id, OperationMode.NON_BLOCKING);
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
	public void deleteFromWorkingMemory(String _id, OperationMode _sync)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		deleteFromWorkingMemoryHelper(_id, m_subarchitectureID, _sync);
	}

	/**
	 * Delete data from working memory with given id.
	 * 
	 * @param _id
	 *            The id of the working memory entry to be deleted.
	 * @param _subarch
	 *            The subarchitecture to write to.
	 * @param _sync
	 *            Whether to block until the operation completes.
	 * @throws SubarchitectureProcessException
	 *             if comms fail.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist on working memory.
	 */
	protected void deleteFromWorkingMemoryHelper(String _id, String _subarch,
			OperationMode _sync) throws DoesNotExistOnWMException,
			SubarchitectureProcessException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		checkPrivileges(_subarch);

		if (!existsOnWorkingMemory(_id, _subarch)) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					_subarch),
					"Entry does not exist on wm to delete. Was looking for id "
							+ _id + " in sa " + _subarch);
		}

		// if we don't currently hold a lock on this item
		if (!holdsDeleteLock(_id, _subarch)) {
			// check that we can delete it
			if (!isDeletable(_id, _subarch)) {
				throw new PermissionException(new WorkingMemoryAddress(_id,
						_subarch), "Delete not allowed on locked item: " + _id
						+ ":" + _subarch);
			}
		}

		if (m_inputToLocalWorkingMemory != null
				&& m_inputToRemoteWorkingMemory != null) {
			throw new SubarchitectureProcessException(
					"Connection to working memory has not been established");
		}

		debug("deleting id = " + _id + " sa = " + _subarch);

		logMemoryDelete(_id, _subarch);

		// this isn't actually needed... versioning can happen forever... it's
		// just expensive
		// stopVersioning(_id);

		if (m_inputToLocalWorkingMemory != null) {

			debug("locally deleting id = " + _id + " sa = " + _subarch);

			CASTWorkingMemoryEntry<Object> wme = new CASTWorkingMemoryEntry<Object>(
					getProcessIdentifier(), WorkingMemoryOperation.DELETE,
					new WorkingMemoryAddress(_id, _subarch), null);

			pushToWorkingMemory(wme, _sync);
		}
		else if (m_inputToRemoteWorkingMemory != null) {
			WorkingMemoryEntry wme = new WorkingMemoryEntry(
					WorkingMemoryOperation.DELETE, new WorkingMemoryAddress(
							_id, _subarch), "", 0, ORB.init().create_any());
			pushToWorkingMemory(wme, _sync);
		}
	}

	private static final char[] id_table = {'0', '1', '2', '3', '4', '5', '6',
			'7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
			'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',
			'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j',
			'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w',
			'x', 'y', 'z'};

	private static final int ID_TABLE_SIZE = 62;

	/**
	 * generates short unique strings from ints
	 */
	protected String stringify(int _n) {
		if (_n == 0)
			return "0";

		String ret = new String();

		// int i = 0;
		while (_n != 0) {
			int m = _n % ID_TABLE_SIZE;
			ret += id_table[m];
			_n -= m;
			_n /= ID_TABLE_SIZE;
		}
		return ret;
	}

	public final String newDataID() {

		StringBuffer sb = new StringBuffer(stringify(m_dataCount++));
		sb.append(":");
		if (m_bDebugOutput) {
			sb.append(getProcessIdentifier());
			sb.append(":data");
		}
		else {
			sb.append(m_componentNumberString);
		}
		return sb.toString();

	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _data
	 *            The data itself
	 * 
	 * @throws SubarchitectureProcessException
	 *             If a datatype error occurs.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist to be overwritten.
	 * @throws ConsistencyException
	 *             if this component does not have the most recent version of
	 *             the data at the given id.
	 */
	protected <T> void overwriteWorkingMemory(String _id, T _data)
			throws DoesNotExistOnWMException, SubarchitectureProcessException,
			ConsistencyException {

		overwriteWorkingMemory(_id, _data, OperationMode.NON_BLOCKING);
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _sync
	 *            Whether to block until the operation has completed.
	 * @param _data
	 *            The data itself
	 * 
	 * @throws SubarchitectureProcessException
	 *             If a datatype error occurs.
	 * @throws DoesNotExistOnWMException
	 *             if the given id does not exist to be overwritten.
	 * @throws ConsistencyException
	 *             if this component does not have the most recent version of
	 *             the data at the given id.
	 */
	protected <T> void overwriteWorkingMemory(String _id, T _data,
			OperationMode _sync) throws DoesNotExistOnWMException,
			SubarchitectureProcessException, ConsistencyException {

		overwriteWorkingMemoryHelper(_id, m_subarchitectureID, _data, _sync);
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _subarch
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

	protected <T> void overwriteWorkingMemoryHelper(String _id,
			String _subarch, T _data, OperationMode _sync)
			throws DoesNotExistOnWMException, SubarchitectureProcessException,
			ConsistencyException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		// check that we can write to that subarch
		checkPrivileges(_subarch);

		if (!existsOnWorkingMemory(_id, _subarch)) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					_subarch),
					"Entry does not exist on wm to overwrite. Was looking for id "
							+ _id + " in sa " + _subarch);
		}

		// if we don't have an overwrite lock
		if (!holdsOverwriteLock(_id, _subarch)) {

			if (!isOverwritable(_id, _subarch)) {
				throw new PermissionException(new WorkingMemoryAddress(_id,
						_subarch), "Overwrite not allowed on locked item: "
						+ _id + ":" + _subarch);
			}

			// then check the consistency of the overwrite
			checkConsistency(_id, _subarch);
		}
		else {

			// if we need to do a one-time check of the consistecny
			if (needsConsistencyCheck(_id, _subarch)) {
				debug("one-time consistency check for locked item: " + _id
						+ ":" + _subarch);

				// then check the consistency of the overwrite
				checkConsistency(_id, _subarch);
				consistencyChecked(_id, _subarch);

			}
			else {
				debug("skipping consistency check for locked item: " + _id
						+ ":" + _subarch);
			}
		}

		String type = CASTUtils.typeName(_data);

		logMemoryOverwrite(_id, _subarch, type);

		dataToWorkingMemory(WorkingMemoryOperation.OVERWRITE, _id, _subarch,
				type, _data, _sync);

		// if we got this far, then we're allowed to update our local version
		// number for the id
		increaseStoredVersion(_id);
	}

	protected void pushToWorkingMemory(CASTWorkingMemoryEntry<?> _wme,
			OperationMode _sync) throws SubarchitectureProcessException {
		if (m_inputToLocalWorkingMemory != null) {

			// System.out.println("pushing local: " +
			// CASTUtils.toString(_wme.getAddress()));

			debug(_wme);
			m_inputToLocalWorkingMemory.push(getProcessIdentifier().toString(),
					_wme);
			if (_sync == OperationMode.BLOCKING) {
				m_inputToLocalWorkingMemory.flush();
			}
		}
		else {
			throw new SubarchitectureProcessException(
					"Connection to local working memory has not been established");
		}
	}

	protected void pushToWorkingMemory(WorkingMemoryEntry _wme,
			OperationMode _sync) throws SubarchitectureProcessException {

		if (m_inputToRemoteWorkingMemory != null) {

			// System.out.println("pushing: " +
			// CASTUtils.toString(_wme.m_address));

			m_inputToRemoteWorkingMemory.push(
					getProcessIdentifier().toString(), _wme);

			if (_sync == OperationMode.BLOCKING) {
				// System.out.println("flushing");
				// System.out.println("syncing remote");
				m_inputToRemoteWorkingMemory.flush();

			}
			// System.out.println("done");
		}
		else {
			throw new SubarchitectureProcessException(
					"Connection to remote working memory has not been established");
		}
	}

	// protected void pushToWorkingMemory(WorkingMemoryEntry[] _wmel)
	// throws SubarchitectureProcessException {
	// for (int i = 0; i < _wmel.length; i++) {
	// pushToWorkingMemory(_wmel[i], false);
	// }
	// }

	public void setPushConnector(String _connectionID, CWMEPushConnectorOut _out) {
		m_inputToLocalWorkingMemory = _out;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.framework.interfaces.WMEPushInterface.WMEPushSender#setPushConnector(java.lang.String,
	 *      caat.framework.interfaces.WMEPushInterface.WMEPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, WMEPushConnectorOut _out) {
		m_inputToRemoteWorkingMemory = _out;
	}

	@Override
	protected void lockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.lockEntry(_id, _subarch, _permissions);
	}

	@Override
	protected void lockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.lockEntry(_wma, _permissions);
	}

	@Override
	protected boolean tryLockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		return super.tryLockEntry(_id, _subarch, _permissions);
	}

	@Override
	protected boolean tryLockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		return super.tryLockEntry(_wma, _permissions);
	}

	@Override
	protected void unlockEntry(String _id, String _subarch) throws WMException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.unlockEntry(_id, _subarch);
	}

	@Override
	protected void unlockEntry(WorkingMemoryAddress _wma) throws WMException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.unlockEntry(_wma);
	}

	@Override
	protected void lockEntry(String _id, WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.lockEntry(_id, _permissions);
	}

	@Override
	protected boolean tryLockEntry(String _id,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		return super.tryLockEntry(_id, _permissions);
	}

	@Override
	protected void unlockEntry(String _id) throws WMException {
		flushWriteConnections(); // TODO: inefficient, this is called too
		// often
		super.unlockEntry(_id);
	}

	private void flushWriteConnections() {
		if (m_inputToRemoteWorkingMemory != null) {
			m_inputToRemoteWorkingMemory.flush();
		}
		if (m_inputToLocalWorkingMemory != null) {
			m_inputToLocalWorkingMemory.flush();
		}
	}

}
