/**
 * 
 */
package cast.architecture;

import java.util.Hashtable;
import java.util.Map;

import Ice.Current;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.WMException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTComponentPermissionsMap;
import cast.core.CASTUtils;
import cast.core.SubarchitectureComponent;
import cast.interfaces.WorkingMemoryPrx;
import cast.interfaces._WorkingMemoryAttachedComponentOperations;

/**
 * The absolute simplest component that can be attached to a working memory.
 * This component is able to check whether an entry exists on a local working
 * memory or not, and check and record the version number of the entry. This
 * class also provides a facility for storing these version numbers.
 * 
 * @author nah
 */
public abstract class WorkingMemoryAttachedComponent extends
		SubarchitectureComponent implements
		_WorkingMemoryAttachedComponentOperations {

	protected WorkingMemoryPrx m_workingMemory;

	public void setWorkingMemory(WorkingMemoryPrx _wm, Current __current) {
		m_workingMemory = _wm;
	}

	private final Hashtable<String, Integer> m_versionNumbers;

	private CASTComponentPermissionsMap m_permissions;

	/**
	 * @param _id
	 */
	public WorkingMemoryAttachedComponent() {
		m_versionNumbers = new Hashtable<String, Integer>();
	}

	//
	@Override
	protected void configureInternal(Map<String, String> _config) {
		super.configureInternal(_config);
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
	public final boolean existsOnWorkingMemory(String _id) {

		try {
			return existsOnWorkingMemory(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
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
	 * @throws UnknownSubarchitectureException
	 */
	public final boolean existsOnWorkingMemory(String _id, String _subarch)
			throws UnknownSubarchitectureException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";
		assert m_workingMemory != null;

		return m_workingMemory.exists(_id, _subarch);

	}

	/**
	 * Determines whether an entry exists on working memory at the given
	 * address.
	 * 
	 * @param _wma
	 *            The address for the entry in working memory.
	 * @return True if the entry exists, otherwise false.
	 * @throws UnknownSubarchitectureException
	 */
	public final boolean existsOnWorkingMemory(WorkingMemoryAddress _wma)
			throws UnknownSubarchitectureException {
		return existsOnWorkingMemory(_wma.id, _wma.subarchitecture);
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
	 * 
	 * 
	 * @remark Interface change: Renamed to reflect new role, behaviour still
	 *         the same: getOverwriteCount -> getVersionNumber.
	 */
	public final int getVersionNumber(String _id)
			throws DoesNotExistOnWMException {
		try {
			return getVersionNumber(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
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
	 *             if the entry has never existed on working memory.
	 * @throws UnknownSubarchitectureException
	 */
	public int getVersionNumber(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";
		assert m_workingMemory != null;

		return m_workingMemory.getVersionNumber(_id, _subarch);
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
	 * @throws UnknownSubarchitectureException
	 * @throws SubarchitectureComponentException
	 *             if communication goes awry
	 * 
	 * @remark Interface change: Renamed to reflect new role, behaviour still
	 *         the same: getOverwriteCount -> getVersionNumber.
	 */
	public int getVersionNumber(WorkingMemoryAddress _wma)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return getVersionNumber(_wma.id, _wma.subarchitecture);
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
			throw new ConsistencyException("No stored version for id: " + _id,
					new WorkingMemoryAddress(_id, getSubarchitectureID()));
		} else {
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
		// if (m_bDebugOutput) {
		// try {
		// debug("updateVersion: " + _id + " "
		// + getStoredVersionNumber(_id) + " -> " + _newVersion);
		// } catch (ConsistencyException e) {
		// debug(e.getMessage());
		// }
		// }
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
	 */
	protected final boolean haveLatestVersion(String _id)
			throws ConsistencyException, DoesNotExistOnWMException {
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
	 */
	protected final void checkConsistency(String _id)
			throws ConsistencyException, DoesNotExistOnWMException {
		try {
			checkConsistency(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	/**
	 * Checks whether this component has read the more recent version of the
	 * data at this working memory address, and throws and exception if not.
	 * 
	 * @param _id
	 * @param _subarch
	 * @throws ConsistencyException
	 *             if the id is not versioned
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected void checkConsistency(String _id, String _subarch)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		if (!isVersioned(_id)) {
			throw new ConsistencyException("!isVersioned(" + _id
					+ ") in subarch " + _subarch, new WorkingMemoryAddress(_id,
					_subarch));
		}

		if (!haveLatestVersion(_id, _subarch)) {
			throw new ConsistencyException(
					"You have attempted to overwrite an outdated working memory entry. Please reread and try again. WMA: "
							+ _id
							+ ":"
							+ _subarch
							+ ". Local version: "
							+ getStoredVersionNumber(_id)
							+ ". WM version: "
							+ getVersionNumber(_id, _subarch),
					new WorkingMemoryAddress(_id, _subarch));
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
	 * @throws UnknownSubarchitectureException
	 */
	protected boolean haveLatestVersion(String _id, String _subarch)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
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
	 * @throws UnknownSubarchitectureException
	 * @throws SubarchitectureComponentException
	 *             if communication goes awry
	 */
	protected void checkConsistency(WorkingMemoryAddress _wma)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		checkConsistency(_wma.id, _wma.subarchitecture);
	}

	/**
	 * Try to obtain a lock on a working memory entry with the given
	 * permissions. This will block until the desired lock is obtained.
	 * 
	 * @param _id
	 * @param _subarch
	 * @param _permissions
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected void lockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		assert _id.length() != 0 : "id must not be empty";
		assert _subarch.length() != 0 : "sa must not be empty";
		assert m_workingMemory != null;

		// will throw here if doesn't exist
		m_workingMemory
				.lockEntry(_id, _subarch, getComponentID(), _permissions);
		m_permissions.setPermissions(_id, _subarch, _permissions);
	}

	/**
	 * Try to obtain a lock on a working memory entry with the given
	 * permissions. This will block until the desired lock is obtained.
	 * 
	 * @param _wma
	 * @param _permissions
	 * @throws UnknownSubarchitectureException
	 */
	protected void lockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		lockEntry(_wma.id, _wma.subarchitecture, _permissions);
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _wma
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws UnknownSubarchitectureException
	 */
	protected void unlockEntry(WorkingMemoryAddress _wma)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		unlockEntry(_wma.id, _wma.subarchitecture);
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _id
	 * @param _subarch
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws UnknownSubarchitectureException
	 * @throws WMException
	 */
	protected void unlockEntry(String _id, String _subarch)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		assert _id.length() != 0 : "id must not be empty";
		assert _subarch.length() != 0 : "sa must not be empty";
		assert m_workingMemory != null;

		// if we don't hold the lock, then don't do anything
		if (!holdsLock(_id, _subarch)) {
			debug("no lock held for: " + _id + ":" + _subarch);
			return;
		}

		assert (_id.length() != 0);// id must not be empty

		// unlock entry... will throw if entry does not exist or locks are wrong
		m_workingMemory.unlockEntry(_id, _subarch, getComponentID());

		m_permissions.removePermissions(_id, _subarch);

		// just check up on ourselves
		assert (!holdsLock(_id, _subarch));

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
	 * @throws UnknownSubarchitectureException
	 * @throws SubarchitectureComponentException
	 */
	protected boolean tryLockEntry(String _id, String _subarch,
			WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		assert _id.length() != 0 : "id must not be empty";
		assert _subarch.length() != 0 : "sa must not be empty";
		assert m_workingMemory != null;

		// will throw here if doesn't exist
		boolean succeeded = m_workingMemory.tryLockEntry(_id, _subarch,
				getComponentID(), _permissions);

		// if we succeeded, then let's store the permissions
		if (succeeded) {
			m_permissions.setPermissions(_id, _subarch, _permissions);
		}

		return succeeded;

	}

	/**
	 * Try to obtain a lock on a working memory entry. This will return true if
	 * the item is locked, or false if not. This method does not block.
	 * 
	 * @param _wma
	 * @param _permissions
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected boolean tryLockEntry(WorkingMemoryAddress _wma,
			WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return tryLockEntry(_wma.id, _wma.subarchitecture, _permissions);
	}

	/**
	 * Gets the permissions currently set on the given working memory item.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected WorkingMemoryPermissions getPermissions(String _id,
			String _subarch) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		assert _id.length() != 0 : "_id must not be empty";
		assert _subarch.length() != 0 : "_subarch must not be empty";
		return m_workingMemory.getPermissions(_id, _subarch);
	}

	/**
	 * Gets the permissions currently set on the given working memory item.
	 * 
	 * @param _wma
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws WMException
	 */
	protected WorkingMemoryPermissions getPermissions(WorkingMemoryAddress _wma)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return getPermissions(_wma.id, _wma.subarchitecture);
	}

	/**
	 * 
	 * Checks whether a given item on working memory is currently overwritable
	 * by this component.
	 * 
	 * @param _id
	 * @param _subarch
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws WMException
	 */
	protected boolean isOverwritable(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		if (holdsOverwriteLock(_id, _subarch)) {
			return true;
		} else {
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
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected boolean isDeletable(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		if (holdsDeleteLock(_id, _subarch)) {
			return true;
		} else {
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
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected boolean isReadable(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		if (holdsReadLock(_id, _subarch)) {
			return true;
		} else {
			WorkingMemoryPermissions permissions = getPermissions(_id, _subarch);
			return CASTUtils.readAllowed(permissions);
		}
	}

	/**
	 * Unlock the given working memory entry.
	 * 
	 * @param _id
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 */
	protected void unlockEntry(String _id) throws ConsistencyException,
			DoesNotExistOnWMException {
		try {
			unlockEntry(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	/**
	 * 
	 * Obtain a lock on a working memory entry with the given permissions. This
	 * will block until the desired lock is obtained.
	 * 
	 * @param _id
	 *            The id of the item on working memory.
	 * @param _permissions
	 *            The permissions to obtain.
	 * @throws DoesNotExistOnWMException
	 *             If the item does not exist on wm.
	 */
	protected void lockEntry(String _id, WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException {
		try {
			lockEntry(_id, getSubarchitectureID(), _permissions);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
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
	 * @throws DoesNotExistOnWMException
	 *             If the item does not exist on wm.
	 */
	protected void tryLockEntry(String _id,
			WorkingMemoryPermissions _permissions)
			throws DoesNotExistOnWMException {
		try {
			lockEntry(_id, getSubarchitectureID(), _permissions);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	//

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
		} else {
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
		} else {
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
		} else {
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
		try {
			return getPermissions(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
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
				&& (permissions == WorkingMemoryPermissions.LOCKEDOD || permissions == WorkingMemoryPermissions.LOCKEDODR);
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
				&& permissions == WorkingMemoryPermissions.LOCKEDODR;
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
				&& (permissions == WorkingMemoryPermissions.LOCKEDOD || permissions == WorkingMemoryPermissions.LOCKEDODR);
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
				&& permissions == WorkingMemoryPermissions.LOCKEDODR;
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
