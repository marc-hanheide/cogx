/**
 * 
 */
package motivation.util;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import motivation.slice.Motive;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class WMMotiveSet implements Map<WorkingMemoryAddress, Motive> {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	private ManagedComponent component;

	private ChangeHandler updateHandler;

	private Map<WorkingMemoryAddress, Motive> map;

	public interface ChangeHandler {
		void motiveChanged(WorkingMemoryChange wmc, Motive motive);
	}

	Class<? extends Motive> specificType;

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveSet create(ManagedComponent c) {
		return new WMMotiveSet(c, Motive.class);
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static WMMotiveSet create(ManagedComponent c,
			final Class<? extends Motive> specificType) {
		return new WMMotiveSet(c, specificType);
	}

	protected WMMotiveSet(ManagedComponent c,
			final Class<? extends Motive> specificType) {
		this.specificType = specificType;
		this.component = c;
		// create a synchronised hashmap
		map = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, Motive>());
	}

	public void start() {
		// register the addition
		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				specificType, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							Motive m = component.getMemoryEntry(_wmc.address,
									specificType);
							map.put(_wmc.address, m);
							updateHandler.motiveChanged(_wmc, map
									.get(_wmc.address));
						} catch (DoesNotExistOnWMException e) {
							// safely ignored if it is already gone
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});

		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				specificType, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						updateHandler
								.motiveChanged(_wmc, map.get(_wmc.address));
						map.remove(_wmc.address);
					}
				});

		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				specificType, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							map.put(_wmc.address, component.getMemoryEntry(
									_wmc.address, specificType));
							updateHandler.motiveChanged(_wmc, map
									.get(_wmc.address));
						} catch (DoesNotExistOnWMException e) {
							// remove it locally
							_wmc.operation = WorkingMemoryOperation.DELETE;
							updateHandler.motiveChanged(_wmc, map
									.get(_wmc.address));
							map.remove(_wmc.address);
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});

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
	public Set<java.util.Map.Entry<WorkingMemoryAddress, Motive>> entrySet() {
		return map.entrySet();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#get(java.lang.Object)
	 */
	@Override
	public Motive get(Object key) {
		return get(key);
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
	public Motive put(WorkingMemoryAddress key, Motive value) {
		try {
			if (map.containsKey(key)) {
				Motive entry = component.getMemoryEntry(key, specificType);
				entry = value;
				component.overwriteWorkingMemory(key, entry);
			} else { // if it is not yet in there
				component.addToWorkingMemory(key, value);
			}
		} catch (AlreadyExistsOnWMException e) {
			component.println(WMMotiveSet.class + ": key " + key.id
					+ " already exists in WM");
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			component
					.println(WMMotiveSet.class + ": key does not exist in WM ");
			e.printStackTrace();
		} catch (ConsistencyException e) {
			e.printStackTrace();
		} catch (PermissionException e) {
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
	public void putAll(Map<? extends WorkingMemoryAddress, ? extends Motive> m) {
		throw (new UnsupportedOperationException("this method is not implemented in WMMotiveSet"));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	@Override
	public Motive remove(Object key) {
		try {
			if (map.containsKey(key)) {
				WorkingMemoryAddress addr = (WorkingMemoryAddress) key;
				component.getMemoryEntry(addr, specificType);
				component.deleteFromWorkingMemory(addr);
			} else { // if it is not yet in there
				// skip it
			}
		}  catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			component
					.println(WMMotiveSet.class + ": key does not exist in WM ");
			e.printStackTrace();
		}  catch (PermissionException e) {
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
	public Collection<Motive> values() {
		return map.values();
	}

}
