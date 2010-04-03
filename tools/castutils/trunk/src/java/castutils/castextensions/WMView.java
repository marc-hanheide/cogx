/**
 * 
 */
package castutils.castextensions;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
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
import cast.core.CASTData;

/**
 * @author marc
 * 
 */
public class WMView<T extends Ice.ObjectImpl> extends CASTHelper implements
		Map<WorkingMemoryAddress, T> {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	private Class<T> type;

	protected Set<ChangeHandler<T>> updateHandler;

	private Map<WorkingMemoryAddress, T> map;

	private String subarchitecturId;

	private boolean shouldInitialize;

	public interface ChangeHandler<T2 extends Ice.ObjectImpl> {
		void entryChanged(Map<WorkingMemoryAddress, T2> map,
				WorkingMemoryChange wmc, T2 newEntry, T2 oldEntry)
				throws CASTException;

	}

	synchronized void dispatchChangeEvent(Map<WorkingMemoryAddress, T> map,
			WorkingMemoryChange wmc, T newEntry, T oldEntry)
			throws CASTException {
		for (ChangeHandler<T> ch : updateHandler) {
			ch.entryChanged(map, wmc, newEntry, oldEntry);
		}
	}

	public class WMChangeReceiver implements WorkingMemoryChangeReceiver {

		public WMChangeReceiver(Class<? extends T> specClass) {
			super();
			this.specClass = specClass;
		}

		Class<? extends T> specClass;

		@SuppressWarnings("unchecked")
		@Override
		public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			WorkingMemoryChange newWmc = (WorkingMemoryChange) _wmc.clone();
			T oldEntry = map.get(_wmc.address);
			switch (_wmc.operation) {
			case ADD:
				try {
					T m = component.getMemoryEntry(newWmc.address, specClass);
					map.put(_wmc.address, (T) m.clone());
					if (oldEntry != null)
						newWmc.operation = WorkingMemoryOperation.OVERWRITE;
					dispatchChangeEvent(map, newWmc, map.get(newWmc.address),
							oldEntry);
				} catch (DoesNotExistOnWMException e) {
					// it's fine... if it's been deleted already, we have
					// nothing to do here
				}

				break;
			case OVERWRITE:
				try {
					map.put(newWmc.address, (T) component.getMemoryEntry(
							newWmc.address, specClass).clone());
					dispatchChangeEvent(map, newWmc, map.get(newWmc.address),
							oldEntry);
				} catch (DoesNotExistOnWMException e) {
					// remove it locally
					log("we expected to overwrite, but actually it has gone...");
					newWmc.operation = WorkingMemoryOperation.DELETE;
					T o = map.remove(newWmc.address);
					map.remove(newWmc.address);
					if (o != null)
						dispatchChangeEvent(map, newWmc, o, oldEntry);
				}

				break;
			case DELETE:
				T o = map.remove(newWmc.address);
				if (o != null)
					dispatchChangeEvent(map, newWmc, o, o);

				break;
			}
		}

	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static <T2 extends ObjectImpl> WMView<T2> create(ManagedComponent c,
			Class<T2> cls) {
		WMView<T2> s = new WMView<T2>(c, cls);
		return s;
	}

	/**
	 * Factory method
	 * 
	 * @param c
	 *            the management component this WMSet is in
	 * @return
	 */
	public static <T2 extends ObjectImpl> WMView<T2> create(ManagedComponent c,
			Class<T2> cls, String subarchitectureId) {
		WMView<T2> s = new WMView<T2>(c, cls, subarchitectureId);
		return s;
	}

	protected WMView(ManagedComponent c, Class<T> cls, String subarchitectureId) {
		super(c);
		this.updateHandler = Collections.synchronizedSet(new HashSet<ChangeHandler<T>>());
		
		this.subarchitecturId = subarchitectureId;
		shouldInitialize = true;
		// specificTypes = new HashSet<Class<? extends T>>();
		this.component = c;
		type = cls;
		// create a synchronized hashmap
		map = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, T>());
	}

	protected WMView(ManagedComponent c, Class<T> cls) {
		super(c);
		this.updateHandler = Collections.synchronizedSet(new HashSet<ChangeHandler<T>>());

		this.subarchitecturId = null;
		shouldInitialize = false;
		// specificTypes = new HashSet<Class<? extends T>>();
		this.component = c;
		type = cls;
		// create a synchronized hashmap
		map = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, T>());
	}

	public void register() {
		log("register listeners");
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.ADD), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.DELETE), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE), new WMChangeReceiver(type));

	}

	public void start() throws UnknownSubarchitectureException {
		// register the addition
		register();
		if (shouldInitialize) {
			log("retrieve existing entries");
			List<CASTData<T>> entries = new LinkedList<CASTData<T>>();
			component.getMemoryEntriesWithData(type, entries, subarchitecturId,
					0);
			log("found " + entries.size() + " of type" + type.getSimpleName()
					+ "entries in SA " + subarchitecturId);
			for (CASTData<T> entry : entries) {
				WorkingMemoryAddress wma = new WorkingMemoryAddress(entry
						.getID(), component.getSubarchitectureID());
				map.put(wma, entry.getData());
			}
		}
	}

	/**
	 * @param handler
	 *            the only handler to set
	 */
	@Deprecated
	public synchronized void setHandler(ChangeHandler<T> handler) {
		this.registerHandler(handler);
	}

	/**
	 * @param handler
	 *            the handler to add
	 */
	public synchronized void registerHandler(ChangeHandler<T> handler) {
		this.updateHandler.add(handler);
	}

	/**
	 * @param addReceiver
	 *            the addReceiver to set
	 */
	public synchronized void unregisterHandler(ChangeHandler<T> handler) {
		this.updateHandler.remove(handler);
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
	public Set<java.util.Map.Entry<WorkingMemoryAddress, T>> entrySet() {
		return map.entrySet();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Map#get(java.lang.Object)
	 */
	@Override
	public T get(Object key) {
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
	public T put(WorkingMemoryAddress key, T value) {
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
			component.println(WMView.class + ": key " + key.id
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
	public void putAll(Map<? extends WorkingMemoryAddress, ? extends T> m) {
		for (Entry<? extends WorkingMemoryAddress, ? extends T> e : m
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
	public T remove(Object key) {
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
			component.println(WMView.class + ": key does not exist in WM ");
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
	public Collection<T> values() {
		return map.values();
	}

}
