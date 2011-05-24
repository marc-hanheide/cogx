package castutils.castextensions;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTData;
import cast.core.CASTUtils;
import cast.interfaces.WorkingMemory;
import castutils.slice.WMMap;

/**
 * is a delegate for a Map<WorkingMemoryAddress,WorkingMemoryAddress> of
 * {@link WorkingMemoryAddress} on a {@link WorkingMemory}. Provides
 * transparent, safe, and synchronized access to such a map. Takes a type
 * parameter to implement specific maps derived from the basic slice type
 * {@link WMMap}.
 * 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 * @param <T>
 *            the specification of the map this class delegates to
 */
public class PointerMap<T extends WMMap> extends CASTHelper implements
		Map<WorkingMemoryAddress, WorkingMemoryAddress> {
	private T map;
	private WorkingMemoryAddress addr = null;
	private Class<? extends T> type;
	private String subarchitectureID;
	private boolean initialized = false;

	/**
	 * try to find an existing specialization of {@link WMMap} in a given
	 * subarchitecture and create its delegate. A warning is logged if more than
	 * one matching entry is being found.
	 * 
	 * @param <T2>
	 * @param component
	 *            the component to operate through
	 * @param subarchitecture
	 *            the subarchitecture to look in
	 * @param type
	 *            the type we are looking for
	 * @return null, if non has been found or the reference to the new delegate
	 *         object.
	 * @throws CASTException
	 */
	public static <T2 extends WMMap> PointerMap<T2> find(
			ManagedComponent component, String subarchitecture, Class<T2> type)
			throws CASTException {
		List<CASTData<T2>> entries = new ArrayList<CASTData<T2>>();
		component.getMemoryEntriesWithData(type, entries, subarchitecture, 0);
		if (entries.size() == 0)
			return null;
		else if (entries.size() > 1)
			Logger
					.getLogger(PointerMap.class)
					.warn(
							"found more than one matching entry in "
									+ subarchitecture
									+ " while looking for "
									+ type.getName()
									+ ". Taking the first one, but this shouldn't happen!");
		return new PointerMap<T2>(component, new WorkingMemoryAddress(entries
				.get(0).getID(), subarchitecture), type);
	}

	/**
	 * creates a new instance of a map of the given type. This instance is not
	 * written to WM already on construction but only on first access. It is
	 * then created in the subarchitecture the given component is a member of.
	 * It is initialized as an empty map.
	 * 
	 * @param component
	 *            the {@link ManagedComponent} we a working with
	 * @param type
	 *            the type of the delegated map
	 * @throws InstantiationException
	 * @throws IllegalAccessException
	 */
	public PointerMap(ManagedComponent component, Class<? extends T> type)
			throws InstantiationException, IllegalAccessException {
		super(component);
		this.type = type;
		this.subarchitectureID = null;
		this.map = this.type.newInstance();
		this.map.map = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	}

	/**
	 * creates a new instance of a map of the given type. This instance is not
	 * written to WM already on construction but only on first access. It is
	 * then created in the given SA. It is initialized as an empty map.
	 * 
	 * @param component
	 *            the {@link ManagedComponent} we a working with
	 * @param subarchitectureID
	 *            the subarchitecture we create this map in.
	 * @param type
	 *            the type of the delegated map
	 * @throws InstantiationException
	 * @throws IllegalAccessException
	 */
	public PointerMap(ManagedComponent component, String subarchitectureID,
			Class<? extends T> type) throws InstantiationException,
			IllegalAccessException {
		super(component);
		this.type = type;
		this.subarchitectureID = subarchitectureID;
		this.map = this.type.newInstance();
		this.map.map = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	}

	/**
	 * create a new delegate for an existing map on {@link WorkingMemory}. This
	 * is useful to access a map that someone else has written.
	 * 
	 * @param component
	 *            the {@link ManagedComponent} we a working with
	 * @param addr
	 *            the address of the existing map.
	 * @param type
	 *            the type of the delegated map
	 * @throws CASTException
	 */
	public PointerMap(ManagedComponent component, WorkingMemoryAddress addr,
			Class<? extends T> type) throws CASTException {
		super(component);
		this.type = type;
		this.subarchitectureID = addr.subarchitecture;
		this.addr = addr;
	}

	/**
	 * 
	 * @see java.util.Map#clear()
	 */
	public void clear() {
		try {
			lock();
			read();
			map.map.clear();
			write();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
		} finally {
			unlock();
		}
	}

	/**
	 * @param key
	 * @return
	 * @see java.util.Map#containsKey(java.lang.Object)
	 */
	public boolean containsKey(Object key) {
		try {
			read();
			return map.map.containsKey(key);
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return false;
		}
	}

	/**
	 * @param value
	 * @return
	 * @see java.util.Map#containsValue(java.lang.Object)
	 */
	public boolean containsValue(Object value) {
		try {
			read();
			return map.map.containsValue(value);
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return false;
		}
	}

	/**
	 * @return
	 * @see java.util.Map#entrySet()
	 */
	public Set<Entry<WorkingMemoryAddress, WorkingMemoryAddress>> entrySet() {
		try {
			read();
			return map.map.entrySet();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		}
	}

	/**
	 * @param o
	 * @return
	 * @see java.util.Map#equals(java.lang.Object)
	 */
	public boolean equals(Object o) {
		try {
			read();
			return map.map.equals(o);
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return false;
		}
	}

	public WorkingMemoryAddress waitFor(WorkingMemoryAddress key) throws InterruptedException, ValueNotAvailableException {
		WorkingMemoryAddress result = null;
		while (result == null) {
			//getLogger().debug("content " + this.toString());
			result = get(key);
			if (result == null) {
				getLogger().debug("have to wait for key "
						+ CASTUtils.toString((WorkingMemoryAddress) key));

				synchronized (addr) {
					long ms=System.currentTimeMillis();
					addr.wait(11000);
					if (System.currentTimeMillis()-ms>10000) {
						getLogger().warn("already for more than 10 seconds for a value for "+CASTUtils.toString(key));
						throw(new ValueNotAvailableException());
					}
				}

			}
		}
		getLogger().debug("got value for key "
				+ CASTUtils.toString((WorkingMemoryAddress) key) + ": "
				+ CASTUtils.toString((WorkingMemoryAddress) result));
		return result;
	}

	/**
	 * @param key
	 * @return
	 * @see java.util.Map#get(java.lang.Object)
	 */
	public WorkingMemoryAddress get(Object key) {
		try {
			read();
			return map.map.get(key);
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		}

	}

	/**
	 * @return
	 * @see java.util.Map#hashCode()
	 */
	public int hashCode() {
		try {
			read();
			return map.map.hashCode();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return 0;
		}
	}

	/**
	 * @return
	 * @see java.util.Map#isEmpty()
	 */
	public boolean isEmpty() {
		try {
			read();
			return map.map.isEmpty();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return true;
		}
	}

	/**
	 * @return
	 * @see java.util.Map#keySet()
	 */
	public Set<WorkingMemoryAddress> keySet() {
		try {
			read();
			return map.map.keySet();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		}
	}

	/**
	 * @param key
	 * @param value
	 * @return
	 * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	 */
	public WorkingMemoryAddress put(WorkingMemoryAddress key,
			WorkingMemoryAddress value) {
		try {
			lock();
			read();
			WorkingMemoryAddress result = map.map.put(key, value);
			write();
			return result;
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		} finally {
			unlock();
		}
	}

	/**
	 * @param m
	 * @see java.util.Map#putAll(java.util.Map)
	 */
	public void putAll(
			Map<? extends WorkingMemoryAddress, ? extends WorkingMemoryAddress> m) {
		try {
			lock();
			read();
			map.map.putAll(m);
			write();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
		} finally {
			unlock();
		}
	}

	/**
	 * @param key
	 * @return
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	public WorkingMemoryAddress remove(Object key) {
		try {
			lock();
			read();
			WorkingMemoryAddress result = map.map.remove(key);
			write();
			return result;
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		} finally {
			unlock();
		}
	}

	/**
	 * @return
	 * @see java.util.Map#size()
	 */
	public int size() {
		try {
			read();
			return map.map.size();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return 0;
		}
	}

	/**
	 * @return
	 * @see java.util.Map#values()
	 */
	public Collection<WorkingMemoryAddress> values() {
		try {
			read();
			return map.map.values();
		} catch (CASTException e) {
			getLogger().error("error in PointerMap ", e);
			return null;
		}
	}

	private void init() throws CASTException {
		if (!initialized) {
			if (addr == null) {
				String sa = subarchitectureID;
				if (subarchitectureID == null)
					sa = component.getSubarchitectureID();
				addr = new WorkingMemoryAddress(component.newDataID(), sa);
				component.addToWorkingMemory(addr, map);
			}
			component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					addr, WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {

						@Override
						public void workingMemoryChanged(WorkingMemoryChange wmc) {
							getLogger().debug("MAP changed, notify waiting threads");
							synchronized (addr) {
								addr.notifyAll();
							}

						}
					});
			initialized = true;
		}
	}

	private void lock() throws CASTException {
		init();
		getLogger().debug("wait for lock " + addr.id);
		component.lockComponent();
		component.lockEntry(addr, WorkingMemoryPermissions.LOCKEDOD);
		getLogger().debug("have lock " + addr.id);
	}

	private void read() throws CASTException {
		init();
		map = component.getMemoryEntry(addr, type);
	}

	private void unlock() {
		try {
			component.unlockEntry(addr);
			component.unlockComponent();
			getLogger().debug("unlocked " + addr.id + "size: " + map.map.size());
		} catch (CASTException e) {
			getLogger().error("error in unlocking PointerMap ", e);
		}
	}

	public String toString() {
		String result = "";
		try {
			lock();
			read();
			for (Entry<WorkingMemoryAddress, WorkingMemoryAddress> e : map.map
					.entrySet()) {
				result += CASTUtils.toString(e.getKey()) + " => "
						+ CASTUtils.toString(e.getValue()) + "\n";
			}
			return result;
		} catch (CASTException e) {
			return "*** invalid due to CASTException: " + e.message + " ***";
		} finally {
			unlock();
		}
	}

	private void write() throws CASTException {
		component.overwriteWorkingMemory(addr, map);
	}

}
