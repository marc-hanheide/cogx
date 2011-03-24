package castutils.castextensions;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.slice.WMMutex;

public class WMLock implements Lock {

	WMMutex wmObject;
	WorkingMemoryAddress address;
	ManagedComponent component;
	private String name;
	private boolean slave;
	
	@Override
	public void lock() {
		try {
			component.lockEntry(address, WorkingMemoryPermissions.LOCKEDO);
			wmObject = component.getMemoryEntry(address, WMMutex.class);
			wmObject.holderName = component.getComponentID();
			component.overwriteWorkingMemory(address, wmObject);
		} catch (CASTException e) {
			component.println("WMLock::lock: " + e);
		}
	}

	@Override
	public void lockInterruptibly() throws InterruptedException {
		throw (new IllegalArgumentException(
				"WMLock::lockInterruptibly: unimplemented method"));
	}

	@Override
	public Condition newCondition() {
		throw (new IllegalArgumentException(
				"WMLock::newCondition: unimplemented method"));
	}

	@Override
	public boolean tryLock() {
		try {
			return component.tryLockEntry(address,
					WorkingMemoryPermissions.LOCKEDODR);
		} catch (CASTException e) {
			component.println("WMLock::lock: " + e);
		}
		return false;
	}

	public boolean tryLock(long arg0, TimeUnit arg1)
			throws InterruptedException {
		throw (new IllegalArgumentException(
				"WMLock::tryLock: unimplemented method"));
	}

	@Override
	public void unlock() {
		try {
			wmObject = component.getMemoryEntry(address, WMMutex.class);
			wmObject.holderName = "";
			component.overwriteWorkingMemory(address, wmObject);
			component.unlockEntry(address);
		} catch (CASTException e) {
			component.println("WMLock::unlock: " + e);
		}

	}

	/**
	 * @param wmObject
	 * @param component
	 * @throws CASTException
	 */
	public WMLock(ManagedComponent component, String name) {
		super();
		this.name = name;
		this.component = component;
		this.address = null;
		this.slave=false;
	}

	/**
	 * @param wmObject
	 * @param component
	 * @throws CASTException
	 */
	public WMLock(ManagedComponent component, String name, boolean slave)  {
		super();
		this.name = name;
		this.component = component;
		this.address = null;
		this.slave=slave;
	}
	
	public void initialize() throws CASTException {
		List<WMMutex> allMutexes = new LinkedList<WMMutex>();
		if (slave)
			while (allMutexes.size()==0)
				component.getMemoryEntries(WMMutex.class, allMutexes);
		else
			component.getMemoryEntries(WMMutex.class, allMutexes);
		for (WMMutex m : allMutexes) {
			if (m.name.equals(name)) {
				component.log("WMLock: using existing Mutex with name " + name
						+ "and address " + CASTUtils.toString(m.addr));
				this.wmObject = m;
				this.address = m.addr;
				break;
			}
		}
		if (this.address == null) { // if we have not found the mutex already on
			// WM we have to create it
			component.log("WMLock: created new Mutex with name " + name);
			this.address = new WorkingMemoryAddress(component.newDataID(),
					component.getSubarchitectureID());
			this.wmObject = new WMMutex(name, "", this.address);
			component.addToWorkingMemory(this.address, this.wmObject);
		}

	}

}


