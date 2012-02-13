package motivation.util.castextensions;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;

import motivation.slice.WMMutex;

import Ice.Object;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;

public class WMLock implements Lock {

	WMMutex wmObject;
	WorkingMemoryAddress address;
	ManagedComponent component;
	private String name;

	@Override
	public void lock() {
		try {
			component.lockEntry(address, WorkingMemoryPermissions.LOCKEDO);
			wmObject=component.getMemoryEntry(address, WMMutex.class);
			wmObject.holderName=component.getComponentID();
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
			wmObject=component.getMemoryEntry(address, WMMutex.class);
			wmObject.holderName="";
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
	public WMLock(ManagedComponent component, String name)  {
		super();
		this.name = name;
		this.component = component;
		this.address = null;
	}
	
	public void initialize() throws CASTException {
		List<WMMutex> allMutexes = new LinkedList<WMMutex>();
		component.getMemoryEntries(WMMutex.class, allMutexes);
		for (WMMutex m : allMutexes) {
			if (m.name.equals(name)) {
				component.log("WMLock: using existing Mutex with name " + name + "and address " + CASTUtils.toString(m.addr));
				this.wmObject = m;
				this.address = m.addr;
				break;
			}
		}
		if (this.address == null) { // if we have not found the mutex already on
									// WM we have to create it
			component.log("WMLock: created new Mutex with name " + name);
			this.address=new WorkingMemoryAddress(component.newDataID(), component.getSubarchitectureID());
			this.wmObject = new WMMutex(name, "", this.address);
			component.addToWorkingMemory(this.address, this.wmObject);
		}

	}

}