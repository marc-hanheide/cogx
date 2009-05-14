/**
 * 
 */
package cast.architecture.abstr;

import org.omg.CORBA.Any;

import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.primitive.interfaces.BoolPullInterface.BoolPullConnectorOut;
import balt.core.data.FrameworkQuery;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemoryProtocol;
import cast.architecture.subarchitecture.WMException;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import cast.core.data.CASTWorkingMemoryEntry;
import cast.core.data.CASTWorkingMemoryItem;
import cast.core.interfaces.CWMELPullInterface.CWMELPullConnectorOut;
import cast.core.interfaces.CWMELPullInterface.CWMELPullSender;
import cast.core.interfaces.WMELPullInterface.WMELPullConnectorOut;
import cast.core.interfaces.WMELPullInterface.WMELPullSender;

/**
 * Extends {@link LocalWorkingMemoryAttachedComponent} to provide the ability to
 * query any working memory.
 * 
 * @author nah
 */
public abstract class WorkingMemoryAttachedComponent
		extends
			LocalWorkingMemoryAttachedComponent
		implements
			WMELPullSender,
			CWMELPullSender {

	protected BoolPullConnectorOut m_existsConnector;

	/**
	 * Connection used to pull information from working memory.
	 */
	protected CWMELPullConnectorOut m_outputFromLocalWorkingMemory;

	/**
	 * Connection used to pull information from working memory.
	 */
	protected WMELPullConnectorOut m_outputFromRemoteWorkingMemory;

	/**
	 * @param _id
	 */
	public WorkingMemoryAttachedComponent(String _id) {
		super(_id);
		m_outputFromRemoteWorkingMemory = null;
		m_outputFromLocalWorkingMemory = null;
	}

	/**
	 * Determines whether an entry exists on working memory at the given
	 * address.
	 * 
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return True if the entry exists, otherwise false.
	 */
	public boolean existsOnWorkingMemory(String _id, String _subarch)
			throws SubarchitectureProcessException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		if (_subarch.equals(m_subarchitectureID)) {
			return existsOnWorkingMemory(_id);
		}
		else {
			return existsOnWorkingMemoryXarch(_id, _subarch);
		}
	}

	/**
	 * Determines whether an entry exists on working memory at the given
	 * address.
	 * 
	 * @param _wma
	 *            The address for the entry in working memory.
	 * @return True if the entry exists, otherwise false.
	 */
	public boolean existsOnWorkingMemory(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {
		return existsOnWorkingMemory(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * Get a count of the number of times the working memory entry at the given
	 * address has been overwritten.
	 * 
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return The overwrite count of the entry. This will be 0 if it has never
	 *         been overwritten.
	 * @throws DoesNotExistOnWMException
	 *             if the entry has never existed on working memory.
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 * 
	 * @remark Interface change: Renamed to reflect new role, behaviour still
	 *         the same: getOverwriteCount -> getVersionNumber.
	 * 
	 */
	public int getVersionNumber(String _id, String _subarch)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		// logReadEvent(_subarch, _id);
		if (!_subarch.equals(m_subarchitectureID)) {
			return getVersionNumberXarch(_id, _subarch);
		}
		else {
			return getVersionNumber(_id);
		}

	}

	/**
	 * Get a count of the number of times the working memory entry at the given
	 * address has been overwritten.
	 * 
	 * @param _wma
	 *            The address for the entry in working memory.
	 * @return The overwrite count of the entry. This will be 0 if it has never
	 *         been overwritten.
	 * @throws DoesNotExistOnWMException
	 *             if the entry does not exist on working memory.
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 * 
	 * @remark Interface change: Renamed to reflect new role, behaviour still
	 *         the same: getOverwriteCount -> getVersionNumber.
	 */
	public int getVersionNumber(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {
		return getVersionNumber(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * Set the pull connector used by the component to get entries from local
	 * working memory.
	 * 
	 * @param _connectionID
	 *            The id of the connector.
	 * @param _senderAdaptor
	 *            The connector itself.
	 * @see caat.core.interfaces.WMELPullInterface.CWMELPullSender#setPullConnector(java.lang.String,
	 *      caat.core.interfaces.WMELPullInterface.CWMELPullConnectorOut)
	 */
	public void setPullConnector(String _connectionID,
			CWMELPullConnectorOut _senderAdaptor) {
		m_outputFromLocalWorkingMemory = _senderAdaptor;
	}

	/**
	 * Set the pull connector used by the component to get entries from local
	 * working memory.
	 * 
	 * @param _connectionID
	 *            The id of the connector.
	 * @param _senderAdaptor
	 *            The connector itself.
	 * @see cast.core.interfaces.WMELPullInterface.WMELPullSender#setPullConnector(java.lang.String,
	 *      cast.core.interfaces.WMELPullInterface.WMELPullConnectorOut)
	 */
	public void setPullConnector(String _connectionID,
			WMELPullConnectorOut _senderAdaptor) {
		m_outputFromRemoteWorkingMemory = _senderAdaptor;
	}

	/**
	 * Query working memory with the given query string. In this instance the
	 * subarch parameter is ignored as it is not appropriate in subarchs. The
	 * parameter is present for later though. The query string must be generated
	 * using one of the SubarchitectureWorkingMemoryProtocol create query
	 * methods.
	 * 
	 * @param _query
	 *            The query string itself, generated by
	 *            SubarchitectureWorkingMemoryProtocol.
	 * @see SubarchitectureWorkingMemoryProtocol
	 * @return An array of data as requested.
	 * @throws SubarchitectureProcessException
	 */
	protected <Type> CASTData<Type>[] queryWorkingMemory(String _query)
			throws SubarchitectureProcessException {

		if (m_outputFromLocalWorkingMemory != null) {
			return queryLocalWorkingMemory(_query);
		}
		else if (m_outputFromRemoteWorkingMemory != null) {
			return queryRemoteWorkingMemory(_query);
		}
		else {
			throw new SubarchitectureProcessException(
					"Pull connection not to working memory not set.");
		}

	}

	/**
	 * Determines whether an entry exists on working memory at the given
	 * address.
	 * 
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return True if the entry exists, otherwise false.
	 */
	private boolean existsOnWorkingMemoryXarch(String _id, String _subarch)
			throws SubarchitectureProcessException {

		// logReadEvent(_subarch, _id);

		CASTData<Boolean>[] wmel = queryWorkingMemory(SubarchitectureWorkingMemoryProtocol
				.createExistsQuery(getProcessIdentifier().toString(),
						m_subarchitectureID, _subarch, _id));

		if (1 == wmel.length) {
			// should be ok
			Boolean bool = (Boolean) wmel[0].getData();
			return bool.booleanValue();
		}
		else {
			throw new SubarchitectureProcessException(
					"Incorrect count returned from working memory. Count = "
							+ wmel.length);
		}
	}

	/**
	 * Get a count of the number of times the working memory entry at the given
	 * address has been overwritten.
	 * 
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return The overwrite count of the entry. This will be 0 if it has never
	 *         been overwritten.
	 * @throws DoesNotExistOnWMException
	 *             if the entry does not exist on working memory.
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	private int getVersionNumberXarch(String _id, String _subarch)
			throws SubarchitectureProcessException, DoesNotExistOnWMException {
		CASTData<Integer>[] wmel = queryWorkingMemory(SubarchitectureWorkingMemoryProtocol
				.createOverwriteCountQuery(getProcessIdentifier().toString(),
						m_subarchitectureID, _subarch, _id));

		if (1 == wmel.length) {
			// should be ok
			int version = (Integer) wmel[0].getData();
			if (version < 0) {
				throw new DoesNotExistOnWMException(new WorkingMemoryAddress(
						_id, _subarch),
						"wm entry has never existed for version check: " + _id
								+ ":" + _subarch);
			}
			else {
				return version;
			}

		}
		else {
			throw new SubarchitectureProcessException(
					"Incorrect count returned from working memory. Count = "
							+ wmel.length);
		}
	}

	/**
	 * @param _query
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	@SuppressWarnings("unchecked")
	private <Type> CASTData<Type>[] queryLocalWorkingMemory(String _query)
			throws SubarchitectureProcessException {

		try {

			// println("WorkingMemoryAttachedComponent.queryLocalWorkingMemory():
			// " + CASTUtils.typeName(Class<Type>()) );

			CASTWorkingMemoryEntry<?> wmel[] = m_outputFromLocalWorkingMemory
					.pull(new FrameworkQuery(getProcessIdentifier(), _query));

			debug("wmel.length: " + wmel.length);

			CASTData<Type> results[] = new CASTData[wmel.length];
			for (int i = 0; i < wmel.length; i++) {
				CASTWorkingMemoryEntry<?> wme = wmel[i];
				CASTWorkingMemoryItem<?> wmi = wme.getItem();

				// println("wmi" + wmi.getData().getClass());

				// if the object is already is a local format
				if (wmi.isLocal()) {
					// TODO how do we stop this being altered!!!!
					results[i] = new CASTData(wme.getAddress().m_id, wme
							.getItem());
				}
				else {
					// bit of a (n in)sanity check
					// println("converting!");
					CASTWorkingMemoryItem<Any> wmiAny = (CASTWorkingMemoryItem<Any>) wmi;

					Object data = RemoteDataTranslator.translateFromAny(wmiAny
							.getData(), CASTUtils.classFromType(wmiAny
							.getType()));

					results[i] = new CASTData(wme.getAddress().m_id, wmiAny
							.getType(), wmiAny.getVersion(), data);
				}

			}

			return results;
		}
		catch (FrameworkConnectionException e) {
			throw new SubarchitectureProcessException(
					"Pull connection not configured correctly", e);
		}
		catch (FrameworkDataTranslatorException e) {
			throw new SubarchitectureProcessException(
					"Unable to translate from remote format", e);
		}
	}

	/**
	 * @param _query
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	@SuppressWarnings("unchecked")
	private <Type> CASTData<Type>[] queryRemoteWorkingMemory(String _query)
			throws SubarchitectureProcessException {
		try {

			WorkingMemoryEntry[] wmel = m_outputFromRemoteWorkingMemory
					.pull(new FrameworkQuery(getProcessIdentifier(), _query));

			CASTData<Type>[] itemList = new CASTData[wmel.length];
			String type;
			String id;
			int version;
			for (int i = 0; i < wmel.length; i++) {
				id = wmel[i].m_address.m_id;
				type = wmel[i].m_type;
				version = wmel[i].m_version;
				itemList[i] = new CASTData(id, type, version,
						RemoteDataTranslator.translateFromAny(wmel[i].m_data,
								CASTUtils.classFromType(type)));
			}

			return itemList;

		}
		catch (FrameworkConnectionException e) {
			throw new SubarchitectureProcessException(
					"Pull connection not configured correctly", e);
		}
		catch (FrameworkDataTranslatorException e) {
			throw new SubarchitectureProcessException(
					"Unable to translate from remote format", e);
		}
	}

	/**
	 * Checks whether this component has read the more recent version of the
	 * data at this working memory address, and throws and exception if not.
	 * 
	 * @param _id
	 * @param _subarch
	 * @throws ConsistencyException
	 *             if the id is not versioned, or if the
	 * @throws DoesNotExistOnWMException
	 *             if the _id does not exist on wm
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	protected void checkConsistency(String _id, String _subarch)
			throws ConsistencyException, SubarchitectureProcessException {

		if (!isVersioned(_id)) {
			throw new ConsistencyException(new WorkingMemoryAddress(_id,
					_subarch), "!isVersioned(" + _id + ") in subarch "
					+ _subarch);
		}

		if (!haveLatestVersion(_id, _subarch)) {
			throw new ConsistencyException(
					new WorkingMemoryAddress(_id, _subarch),
					"You have attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: "
							+ _id
							+ ":"
							+ _subarch
							+ ". Local version: "
							+ getStoredVersionNumber(_id)
							+ ". WM version: "
							+ getVersionNumber(_id, _subarch));
		}

	}

	/**
	 * Checks whether this component has read the more recent version of the
	 * data at this working memory address.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws ConsistencyException
	 *             if the id is not versioned.
	 * @throws DoesNotExistOnWMException
	 *             if the _id does not exist on wm
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	protected boolean haveLatestVersion(String _id, String _subarch)
			throws ConsistencyException, DoesNotExistOnWMException,
			SubarchitectureProcessException {
		assert (isVersioned(_id));

		int ownedVersion = getStoredVersionNumber(_id);
		int wmVersion = getVersionNumber(_id, _subarch);

		debug("id: " + _id + "own: " + ownedVersion + " wm: " + wmVersion);

		return wmVersion == ownedVersion;

	}

	/**
	 * Checks whether this component has read the more recent version of the
	 * data at this working memory address, and throws and exception if not.
	 * 
	 * @param _wma
	 * @throws ConsistencyException
	 *             if the id is not versioned, or if the
	 * @throws DoesNotExistOnWMException
	 *             if the _id does not exist on wm
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	protected void checkConsistency(WorkingMemoryAddress _wma)
			throws ConsistencyException, SubarchitectureProcessException {
		checkConsistency(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * Try to obtain a lock on a working memory entry with the given
	 * permissions. This will block until the desired lock is obtained.
	 * 
	 * @param _id
	 * @param _subarch
	 * @param _permissions
	 * @throws SubarchitectureProcessException
	 */
	protected void lockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		if (_subarch.equals(getSubarchitectureID())) {
			lockEntry(_id, _permissions);
		}
		else {
			lockEntryHelper(_id, _subarch, _permissions, OperationMode.BLOCKING);
		}
	}

	/**
	 * Try to obtain a lock on a working memory entry with the given
	 * permissions. This will block until the desired lock is obtained.
	 * 
	 * @param _wma
	 * @param _permissions
	 * @throws SubarchitectureProcessException
	 */
	protected void lockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		lockEntry(_wma.m_id, _wma.m_subarchitecture, _permissions);
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _wma
	 * @throws WMException
	 */
	protected void unlockEntry(WorkingMemoryAddress _wma) throws WMException {
		unlockEntry(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _id
	 * @param _subarch
	 * @throws WMException
	 */
	protected void unlockEntry(String _id, String _subarch) throws WMException {
		unlockEntryHelper(_id, _subarch);
	}

	/**
	 * Try to obtain a lock on a working memory entry. This will return true if
	 * the item is locked, or false if not. This method does not block.
	 * 
	 * @param _id
	 * @param _subarch
	 * @param _permissions
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	protected boolean tryLockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		if (_subarch.equals(getSubarchitectureID())) {
			return tryLockEntry(_id, _permissions);
		}
		else {
			return lockEntryHelper(_id, _subarch, _permissions,
					OperationMode.NON_BLOCKING);
		}
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
	protected boolean tryLockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		return tryLockEntry(_wma.m_id, _wma.m_subarchitecture, _permissions);
	}

	/**
	 * Gets the permissions currently set on the given working memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws WMException
	 */
	protected WorkingMemoryPermissions getPermissions(String _id,
			String _subarch) throws WMException {
		return getPermissionsHelper(_id, _subarch);
	}

	/**
	 * Gets the permissions currently set on the given working memory item.
	 * 
	 * @param _wma
	 * @return
	 * @throws WMException
	 */
	protected WorkingMemoryPermissions getPermissions(WorkingMemoryAddress _wma)
			throws WMException {
		return getPermissionsHelper(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * 
	 * Checks whether a given item on working memory is currently overwritable
	 * by this component.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws WMException
	 */
	protected boolean isOverwritable(String _id, String _subarch)
			throws WMException {
		if (holdsOverwriteLock(_id, _subarch)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
			// debug("isOverwritable: " + CASTUtils.toString(permissions) + " "
			// + CASTUtils.overwriteAllowed(permissions));
			return CASTUtils.overwriteAllowed(permissions);
		}
	}

	/**
	 * Checks whether a given item on working memory is currently deletable by
	 * this component.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws WMException
	 */
	protected boolean isDeletable(String _id, String _subarch)
			throws WMException {
		if (holdsDeleteLock(_id, _subarch)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
			return CASTUtils.deleteAllowed(permissions);
		}
	}

	/**
	 * Checks whether a given item on working memory is currently readable by
	 * this component.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws WMException
	 */
	protected boolean isReadable(String _id, String _subarch)
			throws WMException {
		if (holdsReadLock(_id, _subarch)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
			return CASTUtils.readAllowed(permissions);
		}
	}
}
