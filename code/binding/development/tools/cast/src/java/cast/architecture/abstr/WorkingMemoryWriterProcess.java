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
import cast.cdl.COMPONENT_NUMBER_KEY;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.ui.ComponentEventType;
import cast.core.CASTUtils;
import cast.core.data.CASTWorkingMemoryEntry;
import cast.core.data.CASTWorkingMemoryItem;
import cast.core.interfaces.CWMEPushInterface.CWMEPushConnectorOut;
import cast.core.interfaces.CWMEPushInterface.CWMEPushSender;
import cast.core.interfaces.WMEPushInterface.WMEPushConnectorOut;
import cast.core.interfaces.WMEPushInterface.WMEPushSender;

/**
 * Class to define an abstract class of sub-architecture processing component
 * that can write ontology-described entries to a subarchitecture-local working
 * memory.
 * 
 * @author nah
 */
public abstract class WorkingMemoryWriterProcess
		extends
			LocalWorkingMemoryAttachedComponent
		implements
			WMEPushSender,
			CWMEPushSender {

	/**
	 * Counter used to generate unique ids.
	 */
	protected int m_dataCount;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configurere(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		String componentNumber = _config
				.getProperty(COMPONENT_NUMBER_KEY.value);
		assert (componentNumber != null);
	}

	/**
	 * Connector used to write to working memory.
	 */
	protected WMEPushConnectorOut m_inputToRemoteWorkingMemory;

	protected CWMEPushConnectorOut m_inputToLocalWorkingMemory;

	/**
	 * Constructor.
	 * 
	 * @param _id
	 *            Unique component id.
	 */
	public WorkingMemoryWriterProcess(String _id) {
		super(_id);
		m_dataCount = 0;
	}

	/**
	 * @param _id
	 * @param _type
	 */
	private void logMemoryAdd(String _id, String _type) {
		logEvent(ComponentEventType.ADD, getProcessIdentifier().toString(),
				m_subarchitectureID, _type, _id);
	}

	/**
	 * @param _id
	 */
	private void logMemoryDelete(String _id) {
		logEvent(ComponentEventType.DELETE, getProcessIdentifier().toString(),
				m_subarchitectureID, "", _id);
	}

	/**
	 * @param _id
	 * @param _type
	 */
	private void logMemoryOverwrite(String _id, String _type) {
		logEvent(ComponentEventType.OVERWRITE, getProcessIdentifier()
				.toString(), m_subarchitectureID, _type, _id);
	}

	/**
	 * Add new data to working memory. The data will be stored with the given
	 * id.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself
	 * @throws SubarchitectureProcessException
	 *             If a datatype error occurs.
	 * @thorws AlreadyExistsOnWMException If the given id already exists on
	 *         working memory.
	 */
	protected <T> void addToWorkingMemory(String _id, T _data)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		checkConnections();

		String type = CASTUtils.typeName(_data);

		// sanity check
		if (existsOnWorkingMemory(_id)) {
			throw new AlreadyExistsOnWMException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()), " Entry already exists at " + _id
					+ ". Not adding with type: " + type);
		}

		logMemoryAdd(_id, type);

		// if not versioned, start doing so
		if (!isVersioned(_id)) {
			debug("is not versioned: " + _id);
			startVersioning(_id);
		}
		// if it's already versioned, then we need to update our numbers
		else {
			debug("re-adding to working memory with id: " + _id);
			// get the last known version number, which we store + 1
			storeVersionNumber(_id, (getVersionNumber(_id) + 1));
		}

		if (m_inputToLocalWorkingMemory != null) {
			CASTWorkingMemoryItem<T> wmi = new CASTWorkingMemoryItem<T>(type,
					_data);
			CASTWorkingMemoryEntry<T> wme = new CASTWorkingMemoryEntry<T>(
					getProcessIdentifier(), WorkingMemoryOperation.ADD,
					new WorkingMemoryAddress(_id, m_subarchitectureID), wmi);
			pushToWorkingMemory(wme);
		}
		else if (m_inputToRemoteWorkingMemory != null) {
			try {
				Any anyData = RemoteDataTranslator.translateToAny(_data);
				WorkingMemoryEntry wme = new WorkingMemoryEntry(
						WorkingMemoryOperation.ADD, new WorkingMemoryAddress(
								_id, m_subarchitectureID), type, 0, anyData);
				pushToWorkingMemory(wme);

			}
			catch (FrameworkDataTranslatorException e) {
				throw new SubarchitectureProcessException(
						"Unable to translate working memory input.", e);
			}
		}

	}

	private void checkConnections() throws SubarchitectureProcessException {
		if (m_inputToLocalWorkingMemory != null
				&& m_inputToRemoteWorkingMemory != null) {
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

		checkConnections();

		// sanity check
		if (!existsOnWorkingMemory(_id)) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()), "Entry does not exist at " + _id
					+ ". Not deleting");
		}

		// // if we don't have an overwrite lock
		if (!holdsOverwriteLock(_id)) {

			if (!isOverwritable(_id)) {
				throw new PermissionException(new WorkingMemoryAddress(_id,
						getSubarchitectureID()),
						"Overwrite not allowed on locked item: " + _id + ":"
								+ getSubarchitectureID());
			}

			// then check the consistency of the overwrite
			checkConsistency(_id);
		}
		else {

			// if we need to do a one-time check of the consistecny
			if (needsConsistencyCheck(_id)) {
				debug("one-time consistency check for locked item: " + _id
						+ ":" + getSubarchitectureID());

				// then check the consistency of the overwrite
				checkConsistency(_id);
				consistencyChecked(_id);

			}
			else {
				debug("skipping consistency check for locked item: " + _id
						+ ":" + getSubarchitectureID());
			}
		}

		logMemoryDelete(_id);

		// stopVersioning(_id);

		if (m_inputToLocalWorkingMemory != null) {
			CASTWorkingMemoryEntry<Object> wme = new CASTWorkingMemoryEntry<Object>(
					getProcessIdentifier(), WorkingMemoryOperation.DELETE,
					new WorkingMemoryAddress(_id, m_subarchitectureID), null);
			pushToWorkingMemory(wme);
		}
		else if (m_inputToRemoteWorkingMemory != null) {

			WorkingMemoryEntry wme = new WorkingMemoryEntry(
					WorkingMemoryOperation.DELETE, new WorkingMemoryAddress(
							_id, m_subarchitectureID), "", 0, ORB.init()
							.create_any());
			pushToWorkingMemory(wme);
		}

	}

	protected void logMemoryWrite(WorkingMemoryOperation _op, String _id,
			String _type) {

	}

	/**
	 * Generate a new unique id for a working memory entry.
	 * 
	 * @return A unique data id.
	 */
	protected String newDataID() {
		return m_dataCount++ + ":" + getProcessIdentifier() + ":data";
	}

	/**
	 * Overwrite data in working memory. The data to be overwritten is indicated
	 * by the id, which is then used for the id of the new data.
	 * 
	 * @param _id
	 *            The id the data will be stored with
	 * @param _type
	 *            The ontological type of the data.
	 * @param _data
	 *            The data itself
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

		checkConnections();
		String type = CASTUtils.typeName(_data);

		// sanity check
		if (!existsOnWorkingMemory(_id)) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()), "Entry does not exist at " + _id
					+ ". Not overwriting with type: " + type);
		}

		// if we don't have an overwrite lock, check consistency
		if (!holdsOverwriteLock(_id)) {
			// check the item is not locked for overwrites
			if (!isOverwritable(_id)) {
				throw new PermissionException(new WorkingMemoryAddress(_id,
						getSubarchitectureID()),
						"Overwrite not allowed on locked item: " + _id + ":"
								+ getSubarchitectureID());
			}

			// the check consistency
			checkConsistency(_id);

		}
		else {
			debug("skipping consistency check for locked item: " + _id + ":"
					+ getSubarchitectureID());
		}

		logMemoryOverwrite(_id, type);

		if (m_inputToLocalWorkingMemory != null) {
			CASTWorkingMemoryItem<T> wmi = new CASTWorkingMemoryItem<T>(type,
					_data);
			CASTWorkingMemoryEntry<T> wme = new CASTWorkingMemoryEntry<T>(
					getProcessIdentifier(), WorkingMemoryOperation.OVERWRITE,
					new WorkingMemoryAddress(_id, m_subarchitectureID), wmi);
			pushToWorkingMemory(wme);
		}
		else if (m_inputToRemoteWorkingMemory != null) {
			try {
				Any anyData = RemoteDataTranslator.translateToAny(_data);
				WorkingMemoryEntry wme = new WorkingMemoryEntry(
						WorkingMemoryOperation.OVERWRITE,
						new WorkingMemoryAddress(_id, m_subarchitectureID),
						type, 0, anyData);
				pushToWorkingMemory(wme);
			}
			catch (FrameworkDataTranslatorException e) {
				throw new SubarchitectureProcessException(
						"Unable to translate working memory input.", e);
			}
		}

		// if we got this far, then we're allowed to update our local version
		// number for the id
		increaseStoredVersion(_id);

	}

	protected void pushToWorkingMemory(CASTWorkingMemoryEntry<?> _wme)
			throws SubarchitectureProcessException {
		if (m_inputToLocalWorkingMemory != null) {
			m_inputToLocalWorkingMemory.push(getProcessIdentifier().toString(),
					_wme);
		}
		else {
			throw new SubarchitectureProcessException(
					"Connection to local working memory has not been established");
		}
	}

	/**
	 * Push the input working memory entry to the working memory component.
	 * 
	 * @param _wme
	 *            The working memory entry to send.
	 * @throws SubarchitectureProcessException
	 *             if comms fail, or the connection to the working memory
	 *             component has not been established.
	 */
	protected void pushToWorkingMemory(WorkingMemoryEntry _wme)
			throws SubarchitectureProcessException {
		if (m_inputToRemoteWorkingMemory != null) {
			m_inputToRemoteWorkingMemory.push(
					getProcessIdentifier().toString(), _wme);
		}
		else {
			throw new SubarchitectureProcessException(
					"Connection to working memory has not been established");
		}
	}

	/**
	 * Push the input working memory entries to the working memory component.
	 * Currently just calls pushToWorkingMemory for each entry in the list.
	 * 
	 * @param _wme
	 *            The working memory entries to send.
	 * @throws SubarchitectureProcessException
	 *             if comms fail, or the connection to the working memory
	 *             component has not been established.
	 */

	protected void pushToWorkingMemory(WorkingMemoryEntry[] _wmel)
			throws SubarchitectureProcessException {
		for (int i = 0; i < _wmel.length; i++) {
			pushToWorkingMemory(_wmel[i]);
		}
	}

	public void setPushConnector(String _connectionID, CWMEPushConnectorOut _out) {
		m_inputToLocalWorkingMemory = _out;
	}

	/**
	 * Sets the connector to the working memory component.
	 * 
	 * @param _connectionID
	 *            The id of the connector.
	 * @param _out
	 *            The connector.
	 * @see caat.framework.interfaces.WMEPushInterface.WMEPushSender#setPushConnector(java.lang.String,
	 *      caat.framework.interfaces.WMEPushInterface.WMEPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, WMEPushConnectorOut _out) {
		m_inputToRemoteWorkingMemory = _out;
	}

}
