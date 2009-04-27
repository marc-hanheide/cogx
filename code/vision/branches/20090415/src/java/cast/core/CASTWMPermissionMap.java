package cast.core;

import java.util.HashMap;
import java.util.concurrent.Semaphore;

import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;

/**
 * A class to manage the locking, unlocking and setting of permissions for
 * working memory entries.
 * 
 * @author nah
 * 
 */
public class CASTWMPermissionMap {

	/**
	 * Holder class for all related items
	 * 
	 * @author nah
	 * 
	 */
	private static class PermissionsStruct {
		WorkingMemoryPermissions m_permissions;
		Semaphore m_mutex;
		String m_owner;
		int m_lockCount;
		boolean m_scheduledForDeletion;

		public PermissionsStruct(WorkingMemoryPermissions _permissions,
				Semaphore _mutex, String _owner, int _lockCount,
				boolean _scheduledForDeletion) {
			super();
			m_permissions = _permissions;
			m_mutex = _mutex;
			m_owner = _owner;
			m_lockCount = _lockCount;
			m_scheduledForDeletion = _scheduledForDeletion;
		}

	};

	private final HashMap<String, PermissionsStruct> m_permissions;

	private final Semaphore m_access;

	public CASTWMPermissionMap() {
		m_permissions = new HashMap<String, PermissionsStruct>();
		m_access = new Semaphore(1);
	}

	private final void lockMap() {
		try {
			m_access.acquire();
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	private final void unlockMap() {
		m_access.release();
	}

	/**
	 * Add an entry to the map. The entry is unlocked by default.
	 * 
	 * @param _id
	 */
	public void add(String _id) {
		lockMap();
		assert (!m_permissions.containsKey(_id));
		m_permissions.put(_id, new PermissionsStruct(
				WorkingMemoryPermissions.UNLOCKED, new Semaphore(1), "", 0,
				false));
		unlockMap();
	}
	/**
	 * Acquires the lock for the entry given by the id. Blocks until the lock is
	 * available.
	 * 
	 * @param _id
	 * @throws InterruptedException
	 */
	public void lock(String _id, String _component,
			WorkingMemoryPermissions _permission) throws InterruptedException {

		Semaphore mutex = null;

		lockMap();
		// if this is an invalid entry, return;
		if (!live(_id)) {
			return;
		}

		PermissionsStruct ps = m_permissions.get(_id);

		assert (ps != null);
		// if this lock is already owned by the locking component
		if (ps.m_owner.equals(_component)) {
			assert (_permission == ps.m_permissions);
			assert (ps.m_lockCount > 0);
			ps.m_lockCount++;
			unlockMap();
			return;
		}

		mutex = ps.m_mutex;

		unlockMap();

		// block until mutex is locked
		mutex.acquire();

		lockMap();
		// if this is now invalid entry, return;
		if (!live(_id)) {
			assert (mutex != null);
			mutex.release();
		}
		else {
			ps = m_permissions.get(_id);
			ps.m_permissions = _permission;
			ps.m_owner = _component;
			ps.m_lockCount = 1;
			ps.m_scheduledForDeletion = false;
			assert (m_permissions.get(_id).m_owner.equals(_component));
			assert (m_permissions.get(_id).m_permissions.equals(_permission));
		}
		unlockMap();

	}

	private boolean live(String _id) {
		PermissionsStruct ps = m_permissions.get(_id);
		if (ps != null && !ps.m_scheduledForDeletion) {
			return true;
		}
		else {
			return false;
		}

	}

	/**
	 * Release the lock for the entry given by the id.
	 * 
	 * @param _id
	 */
	public void unlock(String _id, String _component) {
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		if (ps == null) {
			// cout<<"CASTWMPermissionsMap::unlock leaving deleted item:
			// "<<_id<<" "<<_component<<endl;
		}
		else if (ps.m_lockCount == 0) {
			// cout<<"CASTWMPermissionsMap::unlock leaving unlocked item:
			// "<<_id<<" "<<_component<<endl;
		}
		else if (ps.m_lockCount > 1) {
			assert (ps.m_owner.equals(_component));
			// cout<<"CASTWMPermissionsMap::unlock reduce recursive lock:
			// "<<_id<<" "<<_component<<endl;
			ps.m_lockCount--;
		}
		else {
			if (!CASTUtils.deleteAllowed(ps.m_permissions)) {
				assert (ps.m_owner.equals(_component));
			}
			ps.m_permissions = WorkingMemoryPermissions.UNLOCKED;
			ps.m_owner = "";
			ps.m_lockCount = 0;
			ps.m_mutex.release();
		}
		unlockMap();
	}

	/**
	 * Trys to acquire the lock for the entry given by the id. Only obtains one
	 * if one is available.
	 * 
	 * @param _id
	 * @throws InterruptedException
	 */
	public boolean tryLock(String _id, String _component,
			WorkingMemoryPermissions _permission) throws InterruptedException {

		boolean ret = false;

		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);

		// non existant or scheduled for deletion, disallow
		if (ps == null || ps.m_scheduledForDeletion) {
			ret = false;
		}
		// if already locked by someone else, say not
		else if (ps.m_lockCount > 0 && !(ps.m_owner.equals(_component))) {
			ret = false;
		}
		// else we know that locking is fine here
		else {
			// if we already actually hold the lock
			if (ps.m_lockCount > 0) {
				assert (ps.m_permissions == _permission);
				ps.m_lockCount++;
				// cout<<"CASTWMPermissionsMap::tryLock recursive lock:
				// "<<_id<<"
				// "<<_component<<endl;
			}
			// otherwise setup the details and lock away
			else {
				// cout<<"CASTWMPermissionsMap::tryLock normal lock:
				// "<<_id<<"
				// "<<_component<<endl;
				System.out.println("CASTWMPermissionMap.tryLock()");
				ps.m_mutex.acquire();
				ps.m_permissions = _permission;
				ps.m_owner = _component;
				ps.m_lockCount = 1;
				ps.m_scheduledForDeletion = false;
			}
			ret = true;
		}

		unlockMap();
		return ret;
	}

	/**
	 * Checks whether the given entry is locked.
	 * 
	 * @param _id
	 * @return
	 */
	public boolean isLocked(String _id) {
		boolean ret = false;
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		// if in the map, it's locked if lock count > 0
		if (ps != null) {
			ret = ps.m_lockCount > 0;
		}
		unlockMap();
		return ret;
	}

	/**
	 * Checks whether the given entry is locked.
	 * 
	 * @param _id
	 * @return
	 */
	public boolean isLockHolder(String _id, String _component) {
		boolean ret = false;
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		if (ps != null) {
			ret = (_component.equals(ps.m_owner));
		}
		unlockMap();
		return ret;
	}

	public boolean contains(String _id) {
		// default false
		boolean ret = false;
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		// if it's still in the map then only return true if not scheduled
		// for deletion
		if (ps != null) {
			ret = !(ps.m_scheduledForDeletion);
		}
		unlockMap();
		return ret;
	}

	/**
	 * Gets the permissions for the given entry.
	 * 
	 * @param _id
	 * @return
	 */
	public WorkingMemoryPermissions getPermissions(String _id) {
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		WorkingMemoryPermissions perm = null;
		if (ps == null || ps.m_scheduledForDeletion) {
			// cout<<"CASTWMPermissionsMap::getPermissions: returning on
			// missing
			// entry"<<_id<<endl;
			perm = WorkingMemoryPermissions.DOESNOTEXIST;
		}
		else {
			perm = ps.m_permissions;
		}
		unlockMap();
		return perm;
	}

	public void remove(String _id) throws InterruptedException {

		PermissionsStruct ps = null;
		Semaphore mutex = null;
		lockMap();

		ps = m_permissions.get(_id);

		if (ps == null) {
			// cout<<"CASTWMPermissionsMap::remove: returning on missing
			// entry"<<_id<<endl;
			unlockMap();
			return;
		}
		// if it's not locked, then no problem
		else if (ps.m_lockCount == 0) {
			m_permissions.remove(_id);
			unlockMap();
			return;
		}
		// if it is locked, then schedule it for deletion
		else {
			ps.m_scheduledForDeletion = true;
		}

		// if we get here, then the mutex is locked by someone... uhoh
		// get the mutex and clean up while we are in lock
		mutex = ps.m_mutex;

		while (ps.m_lockCount > 0) {
			// unlock for the block

			unlockMap();

			// try to lock it
			mutex.acquire();

			// then lock up again when we have it
			lockMap();

			mutex.release();
			// need to get a new iterator
			ps = m_permissions.get(_id);

		}

		// once we hget here the map is locked and the lock count is 0, so
		// we're good to go again
		m_permissions.remove(_id);
		unlockMap();
	}

	public String getLockHolder(String _id) {
		lockMap();
		PermissionsStruct ps = m_permissions.get(_id);
		assert (ps != null);
		String owner = ps.m_owner;
		unlockMap();
		return owner;
	}
}
