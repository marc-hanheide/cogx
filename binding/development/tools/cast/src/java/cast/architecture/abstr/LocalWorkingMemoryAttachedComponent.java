/**
 * 
 */
package cast.architecture.abstr;

import java.util.Hashtable;
import java.util.Properties;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.primitive.interfaces.BoolPullInterface.BoolPullConnectorOut;
import balt.core.connectors.pull.primitive.interfaces.BoolPullInterface.BoolPullSender;
import balt.core.connectors.pull.primitive.interfaces.IntPullInterface.IntPullConnectorOut;
import balt.core.data.FrameworkQuery;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.WMException;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import cast.core.components.CASTProcessingComponent;
import cast.core.data.CASTComponentPermissionsMap;
import cast.core.interfaces.PermissionsPullInterface.PermissionsPullConnectorOut;
import cast.core.interfaces.PermissionsPullInterface.PermissionsPullSender;

/**
 * The absolute simplest component that can be attached to a working memory.
 * This component is able to check whether an entry exists on a local working
 * memory or not, and check and record the version number of the entry. This
 * class also provides a facility for storing these version numbers.
 * 
 * @author nah
 */
public abstract class LocalWorkingMemoryAttachedComponent
		extends
			CASTProcessingComponent
		implements
			BoolPullSender,
			PermissionsPullSender {

	private BoolPullConnectorOut m_existsConnector;

	private PermissionsPullConnectorOut m_lockConnector;

	protected IntPullConnectorOut m_versionNumberConnector;

	private final Hashtable<String, Integer> m_versionNumbers;

	/**
	 * A map holding the version numbers of entries used in the past.
	 * 
	 * TODO how to garbage collect this?
	 */
	private final Hashtable<String, Integer> m_oldVersionNumbers;

	/**
	 * @param _id
	 */
	public LocalWorkingMemoryAttachedComponent(String _id) {
		super(_id);
		m_existsConnector = null;
		m_lockConnector = null;
		m_versionNumbers = new Hashtable<String, Integer>();
		m_oldVersionNumbers = new Hashtable<String, Integer>();
	}

	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		m_permissions = new CASTComponentPermissionsMap(getSubarchitectureID());
	}

	/**
	 * Determines whether an entry exists on working memory at the given local
	 * address.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return True if the entry exists, otherwise false.
	 */
	public final boolean existsOnWorkingMemory(String _id)
			throws SubarchitectureProcessException {

		assert _id.length() > 0 : "id must not be empty";

		assert (m_existsConnector != null);
		try {
			return m_existsConnector.pull(new FrameworkQuery(
					getProcessIdentifier(), _id));
		}
		catch (FrameworkConnectionException e) {
			throw new SubarchitectureProcessException(
					"Unable to check existance", e);
		}
	}

	public final void setPullConnector(String _connectionID,
			BoolPullConnectorOut _senderAdaptor) {
		m_existsConnector = _senderAdaptor;
	}

	/**
	 * Get a count of the number of times the working memory entry at the given
	 * address has been overwritten.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return The overwrite count of the entry. This will be 0 if it has never
	 *         been overwritten.
	 * @throws DoesNotExistOnWMException
	 *             if the entry has never existed on working memory.
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 * 
	 * 
	 * @remark Interface change: Renamed to reflect new role, behaviour still
	 *         the same: getOverwriteCount -> getVersionNumber.
	 */
	public final int getVersionNumber(String _id)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		assert (m_versionNumberConnector != null);
		assert _id.length() > 0 : "id must not be empty";
		int version = -1;
		try {
			version = m_versionNumberConnector.pull(new FrameworkQuery(
					getProcessIdentifier(), _id));
		}
		catch (FrameworkConnectionException e) {
			throw new SubarchitectureProcessException(
					"Unable to get overwrite count", e);
		}

		if (version < 0) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()),
					"id has never existed for version check: " + _id);
		}
		else {
			return version;
		}
	}

	public final void setPullConnector(String _connectionID,
			IntPullConnectorOut _senderAdaptor) {
		m_versionNumberConnector = _senderAdaptor;
	}

	/**
	 * Associate the given id with the given version number. This should not be
	 * called by user code.
	 * 
	 * @param _id
	 * @param _version
	 */
	protected final void storeVersionNumber(String _id, int _version) {
		m_versionNumbers.put(_id, _version);
	}

	/**
	 * Get the version number currently stored in the component for the wm id.
	 * 
	 * @param _id
	 * @return
	 * @throws ConsistencyException
	 *             if the id is not stored.
	 */
	protected final int getStoredVersionNumber(String _id)
			throws ConsistencyException {

		Integer v = m_versionNumbers.get(_id);
		if (v == null) {
			throw new ConsistencyException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()), "No stored version for id: " + _id);
		}
		else {
			return v;
		}
	}
	/**
	 * Add the given id to the versioning system with version 0.
	 * 
	 * @param _id
	 */
	protected final void startVersioning(String _id) {
		assert (!isVersioned(_id));
		storeVersionNumber(_id, 0);
	}

	/**
	 * Updates the stored id with a new version number.
	 * 
	 * @param _id
	 * @param _newVersion
	 */
	protected final void updateVersion(String _id, int _newVersion) {
		if (m_bDebugOutput) {
			try {
				debug("updateVersion: " + _id + " "
						+ getStoredVersionNumber(_id) + " -> " + _newVersion);
			}
			catch (ConsistencyException e) {
				debug(e.getLocalizedMessage());
			}
		}
		storeVersionNumber(_id, _newVersion);
	}

	/**
	 * Increment the stored version number by 1.
	 * 
	 * @param _id
	 *            id of entry
	 * @throws ConsistencyException
	 *             if _id is not stored in the versioning system
	 */
	protected final void increaseStoredVersion(String _id)
			throws ConsistencyException {
		int stored = getStoredVersionNumber(_id);
		updateVersion(_id, ++stored);
	}

	/**
	 * Check whether the given id is versioned.
	 * 
	 * @param _id
	 * @return
	 */
	protected final boolean isVersioned(String _id) {
		return m_versionNumbers.containsKey(_id);
	}

	/**
	 * Removes the given id from the versioning system.
	 * 
	 * @param _id
	 */
	protected final void stopVersioning(String _id) {
		if (isVersioned(_id)) {
			removeVersionNumber(_id);
		}
	}

	/**
	 * Removes the given id from the internal versioning system storage.
	 * 
	 * @param _id
	 */
	protected final void removeVersionNumber(String _id) {
		m_versionNumbers.remove(_id);
	}

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
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	protected final boolean haveLatestVersion(String _id)
			throws ConsistencyException, DoesNotExistOnWMException,
			SubarchitectureProcessException {
		assert (isVersioned(_id));

		int ownedVersion = getStoredVersionNumber(_id);
		int wmVersion = getVersionNumber(_id);

		return wmVersion == ownedVersion;
	}

	/**
	 * Checks whether this component has read the more recent version of the
	 * data at this working memory address, and throws and exception if not.
	 * 
	 * @param _id
	 * @throws ConsistencyException
	 *             if the id is not versioned, or if the
	 * @throws DoesNotExistOnWMException
	 *             if the _id does not exist on wm
	 * @throws SubarchitectureProcessException
	 *             if communication goes awry
	 */
	protected final void checkConsistency(String _id)
			throws ConsistencyException, DoesNotExistOnWMException,
			SubarchitectureProcessException {

		if (!isVersioned(_id)) {
			throw new ConsistencyException(new WorkingMemoryAddress(_id,
					getSubarchitectureID()), "!isVersioned(" + _id
					+ ") in subarch " + getSubarchitectureID());
		}

		if (!haveLatestVersion(_id)) {
			throw new ConsistencyException(
					new WorkingMemoryAddress(_id, getSubarchitectureID()),
					"You have attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: "
							+ _id
							+ ":"
							+ getSubarchitectureID()
							+ ". Local version: "
							+ getStoredVersionNumber(_id)
							+ ". WM version: " + getVersionNumber(_id));
		}

	}

	public void setPullConnector(String _connectionID,
			PermissionsPullConnectorOut _senderAdaptor) {
		m_lockConnector = _senderAdaptor;
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _id
	 * @throws WMException
	 */
	protected void unlockEntry(String _id) throws WMException {
		unlockEntryHelper(_id, getSubarchitectureID());
	}

	/**
	 * Please ignore helper!
	 * 
	 * @param _id
	 * @param _subarch
	 * @throws WMException
	 */
	protected void unlockEntryHelper(String _id, String _subarch)
			throws WMException {

		// if
		if (!holdsLock(_id, _subarch)) {
			debug("no lock held for: " + _id + ":" + _subarch);
			return;
		}

		try {
			WorkingMemoryPermissions set = m_lockConnector
					.pull(new FrameworkQuery(getProcessIdentifier(), CASTUtils
							.unlockQueryString(_id, _subarch)));

			// allow for DOES_NOT_EXIST as this could happen if unlock on
			// deletion occurs
			// boolean succeeded = (WorkingMemoryPermissions.UNLOCKED == set
			// ||
			// WorkingMemoryPermissions.DOES_NOT_EXIST == set);

			if (set == WorkingMemoryPermissions.DOES_NOT_EXIST) {
				throw new DoesNotExistOnWMException(new WorkingMemoryAddress(
						_subarch, _id), "ID " + _id
						+ " does not exist on WM in subarch " + _subarch);
			}

			boolean succeeded = (WorkingMemoryPermissions.UNLOCKED == set);

			// we should always succeed unlocking if we hold a lock
			assert (succeeded);

			m_permissions.removePermissions(_id, _subarch);
			assert (!holdsLock(_id, _subarch));

		}
		catch (FrameworkConnectionException e) {
			throw new WMException(new WorkingMemoryAddress(_id, _subarch),
					"Error locking entry", e);
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
	 * @throws SubarchitectureProcessException
	 *             If other errors occur.
	 */
	protected void lockEntry(String _id, WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		lockEntryHelper(_id, getSubarchitectureID(), _permissions,
				OperationMode.BLOCKING);
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
	 * @throws SubarchitectureProcessException
	 *             If other errors occur.
	 */
	protected boolean tryLockEntry(String _id,
			WorkingMemoryPermissions _permissions)
			throws SubarchitectureProcessException {
		return lockEntryHelper(_id, getSubarchitectureID(), _permissions,
				OperationMode.NON_BLOCKING);
	}

	/**
	 * Map representing the permissions currently held by this component.
	 */
	private CASTComponentPermissionsMap m_permissions;

	/**
	 * Ignore helper please!
	 * 
	 * @throws WMException
	 * @throws WMException
	 */
	protected final boolean lockEntryHelper(String _id, String _subarch,
			WorkingMemoryPermissions _permissions, OperationMode _op)
			throws WMException {
		try {
			WorkingMemoryPermissions set = m_lockConnector
					.pull(new FrameworkQuery(getProcessIdentifier(), CASTUtils
							.lockQueryString(_id, _subarch, _permissions, _op)));

			if (set == WorkingMemoryPermissions.DOES_NOT_EXIST) {
				throw new DoesNotExistOnWMException(new WorkingMemoryAddress(
						_id, _subarch), "ID " + _id
						+ " does not exist on WM in subarch " + _subarch);
			}

			boolean succeeded = (_permissions == set);

			// if we blocked then we should always succeed
			if (_op == OperationMode.BLOCKING) {
				assert (succeeded);
			}

			// if we succeeded, then let's store the permissions
			if (succeeded) {
				m_permissions.setPermissions(_id, _subarch, set);
			}

			return succeeded;

		}
		catch (FrameworkConnectionException e) {
			throw new WMException(new WorkingMemoryAddress(_id, _subarch),
					"Error locking entry", e);
		}
	}

	/**
	 * Ignore helper please!
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws WMException
	 */
	protected final WorkingMemoryPermissions getPermissionsHelper(String _id,
			String _subarch) throws WMException {
		try {
			WorkingMemoryPermissions permissions = m_lockConnector
					.pull(new FrameworkQuery(getProcessIdentifier(), CASTUtils
							.statusQueryString(_id, _subarch)));

			if (permissions == WorkingMemoryPermissions.DOES_NOT_EXIST) {
				throw new DoesNotExistOnWMException(new WorkingMemoryAddress(
						_id, _subarch), "ID " + _id
						+ " does not exist on WM in subarch " + _subarch);
			}

			return permissions;
		}
		catch (FrameworkConnectionException e) {
			throw new WMException(new WorkingMemoryAddress(_id, _subarch),
					"Error getting permissions", e);
		}
	}

	/**
	 * Checks whether a given item on working memory is currently overwritable
	 * by this component.
	 * 
	 * @param _id
	 * @return
	 * @throws WMException
	 */
	protected boolean isOverwritable(String _id) throws WMException {
		if (holdsOverwriteLock(_id)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id);
			return CASTUtils.overwriteAllowed(permissions);
		}
	}

	/**
	 * Checks whether a given item on working memory is currently deletable by
	 * this component.
	 * 
	 * @param _id
	 * @return
	 * @throws WMException
	 */
	protected boolean isDeletable(String _id) throws WMException {
		if (holdsDeleteLock(_id)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id);
			return CASTUtils.deleteAllowed(permissions);
		}
	}

	/**
	 * Checks whether a given item on working memory is currently readable by
	 * this component.
	 * 
	 * @param _id
	 * @return
	 * @throws WMException
	 */
	protected boolean isReadable(String _id) throws WMException {
		if (holdsReadLock(_id)) {
			return true;
		}
		else {
			WorkingMemoryPermissions permissions = getPermissions(_id);
			return CASTUtils.readAllowed(permissions);
		}
	}

	/**
	 * Gets the permissions currently set on the given working memory item.
	 * 
	 * @param _id
	 * @return
	 * @throws WMException
	 */
	protected WorkingMemoryPermissions getPermissions(String _id)
			throws WMException {
		return getPermissionsHelper(_id, getSubarchitectureID());
	}

	/**
	 * Checks whether this component holds any kind of lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @return
	 */
	protected boolean holdsLock(String _id) {
		return holdsOverwriteLock(_id);
	}

	/**
	 * Checks whether this component holds any kind of lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 */
	protected boolean holdsLock(String _id, String _subarch) {
		return holdsOverwriteLock(_id, _subarch);
	}

	/**
	 * Checks whether this component holds an overwrite lock on the given
	 * working memory item.
	 * 
	 * @param _id
	 * @return
	 */
	protected boolean holdsOverwriteLock(String _id) {
		// any lock is an overwrite lock
		return m_permissions.getPermissions(_id) != null;
	}

	/**
	 * Checks whether this component holds a delete lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @return
	 */
	protected boolean holdsDeleteLock(String _id) {
		WorkingMemoryPermissions permissions = m_permissions
				.getPermissions(_id);
		return permissions != null
				&& (permissions == WorkingMemoryPermissions.LOCKED_OD || permissions == WorkingMemoryPermissions.LOCKED_ODR);
	}

	/**
	 * Checks whether this component holds a read lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @return
	 */
	protected boolean holdsReadLock(String _id) {
		WorkingMemoryPermissions permissions = m_permissions
				.getPermissions(_id);
		return permissions != null
				&& permissions == WorkingMemoryPermissions.LOCKED_ODR;
	}

	/**
	 * Checks whether this component holds an overwrite lock on the given
	 * working memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 */
	protected boolean holdsOverwriteLock(String _id, String _subarch) {
		// any lock is an overwrite lock
		return m_permissions.getPermissions(_id, _subarch) != null;
	}

	/**
	 * Checks whether this component holds a delete lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 */
	protected boolean holdsDeleteLock(String _id, String _subarch) {
		WorkingMemoryPermissions permissions = m_permissions.getPermissions(
				_id, _subarch);
		return permissions != null
				&& (permissions == WorkingMemoryPermissions.LOCKED_OD || permissions == WorkingMemoryPermissions.LOCKED_ODR);
	}

	/**
	 * Checks whether this component holds a read lock on the given working
	 * memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 */
	protected boolean holdsReadLock(String _id, String _subarch) {
		WorkingMemoryPermissions permissions = m_permissions.getPermissions(
				_id, _subarch);
		return permissions != null
				&& permissions == WorkingMemoryPermissions.LOCKED_ODR;
	}

	protected boolean needsConsistencyCheck(String _id) {
		return m_permissions.needsConsistencyCheck(_id);
	}

	protected boolean needsConsistencyCheck(String _id, String _subarch) {
		return m_permissions.needsConsistencyCheck(_id, _subarch);
	}

	protected void consistencyChecked(String _id) {
		m_permissions.consistencyChecked(_id);
	}

	protected void consistencyChecked(String _id, String _subarch) {
		m_permissions.consistencyChecked(_id, _subarch);
	}

}
