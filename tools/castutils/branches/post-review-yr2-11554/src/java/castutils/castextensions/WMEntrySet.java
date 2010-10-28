/**
 * 
 */
package castutils.castextensions;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import Ice.ObjectImpl;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public class WMEntrySet implements Map<WorkingMemoryAddress, Ice.ObjectImpl> {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	protected ManagedComponent component;

	protected ChangeHandler updateHandler;

	Collection<Class<? extends Ice.ObjectImpl>> specificTypes;

	private Map<WorkingMemoryAddress, Ice.ObjectImpl> map;

	public interface ChangeHandler {
		void entryChanged(Map<WorkingMemoryAddress, Ice.ObjectImpl> map,
				WorkingMemoryChange wmc, Ice.ObjectImpl newEntry,
				Ice.ObjectImpl oldEntry) throws CASTException;
	}

	public class WMChangeReceiver implements WorkingMemoryChangeReceiver {

		public WMChangeReceiver(Class<? extends ObjectImpl> specClass) {
			super();
			this.specClass = specClass;
		}

		Class<? extends Ice.ObjectImpl> specClass;

		@Override
		public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			WorkingMemoryChange newWmc = (WorkingMemoryChange) _wmc.clone();
			Ice.ObjectImpl oldEntry = map.get(_wmc.address);
//			try {
//				component.lockEntry(_wmc.address,
//						WorkingMemoryPermissions.LOCKEDOD);
				switch (_wmc.operation) {
				case ADD:
					try {
						Ice.ObjectImpl m = component.getMemoryEntry(
								newWmc.address, specClass);
						map.put(_wmc.address, (Ice.ObjectImpl) m.clone());
						if (oldEntry != null)
							newWmc.operation = WorkingMemoryOperation.OVERWRITE;
						if (updateHandler != null)
							updateHandler.entryChanged(map, newWmc, map
									.get(newWmc.address), oldEntry);
					} catch (DoesNotExistOnWMException e) {
						// it's fine... if it's been deleted already, we have
						// nothing to do here
					}

					break;
				case OVERWRITE:
					try {
						map.put(newWmc.address, (Ice.ObjectImpl) component
								.getMemoryEntry(newWmc.address, specClass)
								.clone());
						if (updateHandler != null)
							updateHandler.entryChanged(map, newWmc, map
									.get(newWmc.address), oldEntry);
					} catch (DoesNotExistOnWMException e) {
						// remove it locally
						component
								.log("we expected to overwrite, but actually it has gone...");
						newWmc.operation = WorkingMemoryOperation.DELETE;
						Ice.ObjectImpl o = map.remove(newWmc.address);
						map.remove(newWmc.address);
						if (o != null)
							if (updateHandler != null)
								updateHandler.entryChanged(map, newWmc, o,
										oldEntry);
					}

					break;
				case DELETE:
					Ice.ObjectImpl o = map.remove(newWmc.address);
					if (o != null)
						if (updateHandler != null)
							updateHandler.entryChanged(map, newWmc, o, o);

					break;
				}
//			} finally {
//				component.unlockEntry(_wmc.address);
//			}
		}

	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMEntrySet create(ManagedComponent c) {
		return new WMEntrySet(c);
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMEntrySet create(ManagedComponent c,
			final Class<? extends Ice.ObjectImpl> specificType) {
		WMEntrySet s = new WMEntrySet(c);
		s.addType(specificType);
		return s;
	}

	protected WMEntrySet(ManagedComponent c) {
		specificTypes = new HashSet<Class<? extends Ice.ObjectImpl>>();
		this.component = c;
		// create a synchronized hashmap
		map = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, Ice.ObjectImpl>());
	}

	public void addType(Class<? extends Ice.ObjectImpl> specificType) {
		this.specificTypes.add(specificType);
	}

	public void register(Class<? extends Ice.ObjectImpl> type) {
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.ADD), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.DELETE), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE), new WMChangeReceiver(type));

	}

	public void start() {
		// register the addition
		for (Class<? extends Ice.ObjectImpl> t : specificTypes) {
			register(t);
		}
	}

	/**
	 * @param addReceiver
	 *            the addReceiver to set
	 */
	public void setHandler(ChangeHandler handler) {
		this.updateHandler = handler;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#clear()
	 */
	@Override
	public void clear() {
		for (WorkingMemoryAddress key : map.keySet()) {
			try {
				component.deleteFromWorkingMemory(key);
			} catch (DoesNotExistOnWMException e) {
				// safely ignored
			} catch (PermissionException e) {
				component.println("permission execption when clearing map");
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				component
						.println("UnknownSubarchitectureException execption when clearing map");
				e.printStackTrace();
			}
		}
		map.clear();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#containsKey(java.lang.Object)
	 */
	@Override
	public boolean containsKey(Object key) {
		return map.containsKey(key);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#containsValue(java.lang.Object)
	 */
	@Override
	public boolean containsValue(Object value) {
		return map.containsValue(value);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#entrySet()
	 */
	@Override
	public Set<java.util.Map.Entry<WorkingMemoryAddress, Ice.ObjectImpl>> entrySet() {
		return map.entrySet();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#get(java.lang.Object)
	 */
	@Override
	public Ice.ObjectImpl get(Object key) {
		return map.get(key);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#isEmpty()
	 */
	@Override
	public boolean isEmpty() {
		return map.isEmpty();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#keySet()
	 */
	@Override
	public Set<WorkingMemoryAddress> keySet() {
		return map.keySet();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	 */
	@Override
	public Ice.ObjectImpl put(WorkingMemoryAddress key, Ice.ObjectImpl value) {
		try {
			if (map.containsKey(key)) {
				try {
					component.lockEntry(key, WorkingMemoryPermissions.LOCKEDO);
					component.overwriteWorkingMemory(key, value);
				} catch (CASTException e) {
					e.printStackTrace();
				} finally {
					component.unlockEntry(key);
				}
			} else { // if it is not yet in there
				component.addToWorkingMemory(key, value);
			}
		} catch (AlreadyExistsOnWMException e) {
			component.println(WMEntrySet.class + ": key " + key.id
					+ " already exists in WM");
			e.printStackTrace();
		} catch (CASTException e) {
			e.printStackTrace();
		}

		// put to the the map tough the event handler will do it again...
		return map.put(key, value);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#putAll(java.util.Map)
	 */
	@Override
	public void putAll(
			Map<? extends WorkingMemoryAddress, ? extends Ice.ObjectImpl> m) {
		for (Entry<? extends WorkingMemoryAddress, ? extends ObjectImpl> e : m
				.entrySet()) {
			put(e.getKey(), e.getValue());
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	@Override
	public Ice.ObjectImpl remove(Object key) {
		try {
			if (map.containsKey(key)) {
				WorkingMemoryAddress addr = (WorkingMemoryAddress) key;
				component.deleteFromWorkingMemory(addr);
			} else { // if it is not yet in there
				// skip it
			}
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			component.println(WMEntrySet.class + ": key does not exist in WM ");
			e.printStackTrace();
		} catch (PermissionException e) {
			e.printStackTrace();
		}
		return map.remove(key);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#size()
	 */
	@Override
	public int size() {
		return map.size();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#values()
	 */
	@Override
	public Collection<Ice.ObjectImpl> values() {
		return map.values();
	}

}
