/**
 * 
 */
package castutils.castextensions;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryReaderComponent.ChangeReceiverPriority;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.interfaces.WorkingMemory;

/**
 * A generic and abstract monitor to observe and monitor events in the working
 * memory that are assumed to be causally related. This monitor is useful for
 * modeling any kind of dependent event propagation in a system. For instance,
 * the derived class {@link PlaceUnionEventRelation} implements a relation
 * between {@link Place} updates and {@link UnionConfiguration} updates.
 * 
 * {@link CausalEventMonitor} has only one abstract method, namely, compare,
 * that has to be implemented by deriving classes. Once started, an instance of
 * this Monitor will listen for changed using the registered filters. Two
 * different roles of events are known to the monitor. <b>Triggers</b> are
 * events that are expected to cause an <b>Implication</b> in the system. In
 * other words, the Monitor expects another implicated event to occur for each
 * trigger event. (Note: one implication can, however, be the result of several
 * triggers, depending on the specialization of the compare() method).
 * 
 * The Monitor has bounded maps that preserve the history of triggers. For now
 * the boundaries of these maps are set to 10000. What does bounded mean here?
 * Well, these boundaries define the event horizon for the monitor. An
 * implication has to occur within this horizon to be matched with its
 * particular triggering event. As the Monitor only listens to very specific
 * event, this should impose no problems in any real systems, as long as for
 * every trigger event sooner or later a matching implication event is received!
 * The Monitor also manages a bounded map of successfully matched events. It
 * allows to look up past propagations also up to the defined event horizon
 * size.
 * 
 * To effectively use the Monitor, it provides methods to wait (blocking or for
 * a specific time) for the propagation of all or specific events or memory
 * entries.
 * 
 * @author marc
 * 
 */
public abstract class CausalEventMonitor<TypeTrigger extends Ice.Object, TypeImplication extends Ice.Object> {
	/**
	 * this is the maximum number of entries allowed in the change history. so
	 * the change history is a limited buffer containing only the last N
	 * changes.
	 */
	protected static final int EVENT_HORIZON = 10000;

	/** the component to be used for memory access */
	protected ManagedComponent component;

	/**
	 * map of pending changes. I'is instantiated as a synchronized
	 * {@link PendingChangeMap}. The values of this map are always null, so it's
	 * actually a set, but for re-use reasons it is a map :-) It's a bounded
	 * synchronized map.
	 */
	protected Map<WorkingMemoryChange, WorkingMemoryChange> pendingChanges;

	/**
	 * map of propagated events. It contains all successfully matched events
	 * with the trigger {@link WorkingMemoryChange} as key and the implication
	 * {@link WorkingMemoryChange} as value. It's a bounded synchronized map.
	 */
	protected Map<WorkingMemoryChange, WorkingMemoryChange> lastPropagatedEvents;

	/**
	 * map of propagated entries. It contains all successfully matched events
	 * with the {@link WorkingMemoryAddress} of the trigger
	 * {@link WorkingMemoryChange} as the key and the
	 * {@link ImplicationReceiver} event as value. It's a bounded synchronized
	 * map.
	 */
	protected Map<WorkingMemoryAddress, WorkingMemoryChange> lastPropagatedEntries;

	/**
	 * bounded map of pending changes. In contrast to {@link BoundHistoryMap} it
	 * prints a warning when there is an overflow in the map.
	 * 
	 * @author marc
	 * 
	 * @param <K>
	 *            any key
	 * @param <V>
	 *            any value
	 * @see LinkedHashMap
	 */
	private class PendingChangeMap<K, V> extends LinkedHashMap<K, V> {
		/**
		 * 
		 */
		private static final long serialVersionUID = -1319116490891500277L;

		/**
		 * make sure we have a bounded map with only a limited number of
		 * entries.
		 * 
		 * @see EVENT_HORIZON
		 */
		@Override
		protected boolean removeEldestEntry(Entry<K, V> arg0) {
			if (size() > EVENT_HORIZON) {
				component
						.println("PendingChangeMap overflow: too many pending events");
				return true;
			} else
				return false;
		}

	}

	/**
	 * a bounded map of history. Used to store the past propagation w.r.t.
	 * {@link WorkingMemoryChange}s and {@link WorkingMemoryAddress}es, usually.
	 * 
	 * @author marc
	 * 
	 * @param <K>
	 *            any key (here: either {@link WorkingMemoryChange} or
	 *            {@link WorkingMemoryAddress})
	 * @param <V>
	 *            any value (here: {@link WorkingMemoryChange})
	 */
	private class BoundHistoryMap<K, V> extends LinkedHashMap<K, V> {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1085792942275537695L;

		/**
		 * make sure we have a bounded map with only a limited number of
		 * entries.
		 * 
		 * @see EVENT_HORIZON
		 */
		@Override
		protected boolean removeEldestEntry(Entry<K, V> arg0) {
			return size() > EVENT_HORIZON;
		}

	}

	/**
	 * receiver for any trigger event. put the event in the pending list and
	 * adds empty values for this event in the history maps.
	 * 
	 * @author marc
	 * 
	 */
	class TriggerReceiver implements WorkingMemoryChangeReceiver {
		@Override
		public void workingMemoryChanged(WorkingMemoryChange wmc)
				throws CASTException {
			component.debug("received trigger on " + wmc.address.id
					+ ", store as pending (number of pending events is "
					+ pendingChanges.size());
			pendingChanges.put(wmc, null);
			lastPropagatedEvents.put(wmc, null);
			lastPropagatedEntries.put(wmc.address, null);
		}

	}

	/**
	 * receiver for any implication event. calls the abstract method compare to
	 * check this implication against all pending triggers.
	 * 
	 * @author marc
	 * 
	 */
	class ImplicationReceiver implements WorkingMemoryChangeReceiver {
		@Override
		public void workingMemoryChanged(WorkingMemoryChange wmc)
				throws CASTException {
			// only if we await a certain change
			component.debug("testing for trigger leading to implication "
					+ wmc.address.id);
			if (pendingChanges.size() > 0) {
				// copy to allow removal from original list
				Set<WorkingMemoryChange> copySet = new HashSet<WorkingMemoryChange>(
						pendingChanges.keySet());
				for (WorkingMemoryChange triggerChange : copySet) {
					component.debug("comparing trigger "
							+ triggerChange.address.id
							+ " with current implication" + wmc.address.id);

					if (compare(triggerChange, wmc)) {
						component
								.debug("  there is a match, we fire all pending processes");
						// the corresponding event has occured!
						fireCorrespondence();
						pendingChanges.remove(triggerChange);
						lastPropagatedEvents.put(triggerChange, wmc);
						lastPropagatedEntries.put(triggerChange.address, wmc);
						component.debug("there remain " + pendingChanges.size()
								+ " pending changes and "
								+ lastPropagatedEvents.size()
								+ " propagated changes");
					}

				}
			}
		}
	}

	/**
	 * instance of the trigger receiver to be bound to triggerFilters
	 * 
	 */
	WorkingMemoryChangeReceiver triggerReceiver;
	/**
	 * instance of the implication receiver to be bound to triggerFilters
	 * 
	 */
	WorkingMemoryChangeReceiver implicationReceiver;
	/**
	 * all registered trigger filters
	 * 
	 */
	Set<WorkingMemoryChangeFilter> triggerFilters;
	/**
	 * all registered implication filters
	 * 
	 */
	Set<WorkingMemoryChangeFilter> implicationFilters;

	/**
	 * Constructor
	 * 
	 * @param component
	 *            the component to be used for memory access
	 */
	public CausalEventMonitor(ManagedComponent component) {
		super();
		this.pendingChanges = Collections
				.synchronizedMap(new PendingChangeMap<WorkingMemoryChange, WorkingMemoryChange>());
		this.triggerFilters = new HashSet<WorkingMemoryChangeFilter>();
		this.implicationFilters = new HashSet<WorkingMemoryChangeFilter>();
		this.component = component;

		triggerReceiver = new TriggerReceiver();
		implicationReceiver = new ImplicationReceiver();
		lastPropagatedEvents = Collections
				.synchronizedMap(new BoundHistoryMap<WorkingMemoryChange, WorkingMemoryChange>());
		lastPropagatedEntries = Collections
				.synchronizedMap(new BoundHistoryMap<WorkingMemoryAddress, WorkingMemoryChange>());
	}

	/**
	 * synchronized method is called when a new matching trigger/implication
	 * pair has been found. This method notifies all waiting methods to check if
	 * their condition is now fulfilled.
	 * 
	 */
	synchronized void fireCorrespondence() {
		this.notifyAll();
	}

	/**
	 * the abstract compare method is called whenever an implication has to be
	 * checked against a trigger.
	 * 
	 * @param wmcTrigger
	 *            the memory event of the trigger to check for (obtained from
	 *            {@link PendingChangeMap})
	 * @param wmcImplication
	 *            the implication that should be matched
	 * @return true iff the Monitor shall consider the wmcImplication event to
	 *         be a causal implication of the wmcTrigger event; false otherwise
	 * @throws CASTException
	 *             this method is allowed to throw {@link CASTException} as it
	 *             usually needs to access the {@link WorkingMemory}.
	 */
	protected abstract boolean compare(WorkingMemoryChange wmcTrigger,
			WorkingMemoryChange wmcImplication) throws CASTException;

	/**
	 * add a filter to the set of trigger filters <b>Note: filters can only be
	 * added before calling start():</b>
	 * 
	 * @param wmcf
	 *            filter to be added
	 */
	public void addTriggerFilter(WorkingMemoryChangeFilter wmcf) {
		triggerFilters.add(wmcf);
	}

	/**
	 * add a filter to the set of implication filters. <b>Note: filters can only
	 * be added before calling start():</b>
	 * 
	 * @param wmcf
	 *            filter to be added
	 */
	public void addImplicationFilter(WorkingMemoryChangeFilter wmcf) {
		implicationFilters.add(wmcf);
	}

	/**
	 * start registers the added filters. Any filters added after start are not
	 * registered! Deriving class should add their specific listeners in the
	 * constructor and leave start() untouched.
	 * 
	 */
	public void start() {
		// we register the triggers with HIGH priority to ensure they are put on
		// the list before any other receiver in this components receives them
		for (WorkingMemoryChangeFilter wmcf : triggerFilters)
			component.addChangeFilter(wmcf, triggerReceiver,
					ChangeReceiverPriority.HIGH);

		// we register the implication receiver with LOW priority in order to
		// make sure that all other receivers receive it before we actually
		// release the stuff here
		for (WorkingMemoryChangeFilter wmcf : implicationFilters)
			component.addChangeFilter(wmcf, implicationReceiver,
					ChangeReceiverPriority.LOW);

	}

	/**
	 * check if a certain event is pending, saying its implication is not yet
	 * there
	 * 
	 * @param changeToPropagate
	 *            the change we like to check for (yes, it should be a trigger
	 *            event)
	 * @return true if this is on the list of pending events, false if it either
	 *         has been propagated already or if it non was every submitted.
	 */
	synchronized public boolean isPending(WorkingMemoryChange changeToPropagate) {
		return pendingChanges.containsKey(changeToPropagate);
	}

	// synchronized public Map<WorkingMemoryChange, WorkingMemoryChange>
	// getLastPropagatedChanges() {
	// Map<WorkingMemoryChange, WorkingMemoryChange> result = new
	// HashMap<WorkingMemoryChange, WorkingMemoryChange>();
	// for (Entry<WorkingMemoryChange, WorkingMemoryChange> entry :
	// lastPropagatedEvents
	// .entrySet()) {
	// if (entry.getValue() != null)
	// result.put(entry.getKey(), entry.getValue());
	// }
	// return result;
	// }

	/**
	 * get pending changes. This set is empty if no changes are to be
	 * propagated.
	 * 
	 * @return (mutable) set of pending changes. Useful to check the size for
	 *         instance.
	 */
	synchronized public Set<WorkingMemoryChange> getPendingChanges() {
		return pendingChanges.keySet();
	}

	/**
	 * wait that a specific change has been propagated through the system.
	 * 
	 * @param changeToPropagate
	 *            trigger event to be propagated
	 * @param timeout
	 *            timeout in milliseconds
	 * @return the implicated memory change or null
	 * @throws InterruptedException
	 */
	synchronized public WorkingMemoryChange waitForPropagation(
			WorkingMemoryChange changeToPropagate, long timeout)
			throws InterruptedException {
		WorkingMemoryChange result = null;
		long startTime = System.currentTimeMillis();
		while (result == null) {
			result = lastPropagatedEvents.get(changeToPropagate);
			if (result == null) // if not yet there... wait until it is...
				this.wait(timeout);
			// if we consumed our time...
			if (System.currentTimeMillis() > startTime + timeout)
				return null;
		}
		return result;
	}

	/**
	 * wait that a specific change has been propagated through the system.
	 * 
	 * @param changeToPropagate
	 *            trigger event to be propagated
	 * @return the implicated memory change or null
	 * @throws InterruptedException
	 */
	public WorkingMemoryChange waitForPropagation(
			WorkingMemoryChange changeToPropagate) throws InterruptedException {
		return waitForPropagation(changeToPropagate, 0);
	}

	/**
	 * wait that a specific change w.r.t a {@link WorkingMemoryAddress} has been
	 * propagated through the system.
	 * 
	 * @param addrToPropagate
	 *            memory entry (adsress) to be propagated
	 * @param timeout
	 *            timeout in milliseconds
	 * @return the implicated memory change or null
	 * @throws InterruptedException
	 */
	synchronized public WorkingMemoryChange waitForPropagation(
			WorkingMemoryAddress addrToPropagate, long timeout)
			throws InterruptedException {
		WorkingMemoryChange result = null;
		long startTime = System.currentTimeMillis();
		while (result == null) {
			if (!lastPropagatedEntries.containsKey(addrToPropagate)) {
				component
						.debug("  waitForPropagation: nothing to wait for address "
								+ addrToPropagate);
				// we know nothing about this entry, so assume it's propagated
				result = null;
				break;
			}
			result = lastPropagatedEntries.get(addrToPropagate);
			if (result == null) {// if not yet there... wait until it is...
				component.debug("  waitForPropagation: still waiting for "
						+ addrToPropagate);
				this.wait(timeout);
				// if we consumed our time...
				if (System.currentTimeMillis() > startTime + timeout)
					return null;
			} else {
				lastPropagatedEntries.remove(addrToPropagate);
				component.debug("  waitForPropagation: propagated for "
						+ addrToPropagate);
			}
		}
		return result;
	}

	/**
	 * wait that a specific change w.r.t a {@link WorkingMemoryAddress} has been
	 * propagated through the system.
	 * 
	 * @param addrToPropagate
	 *            memory entry (adsress) to be propagated
	 * @return the implicated memory change or null
	 * @throws InterruptedException
	 */
	public WorkingMemoryChange waitForPropagation(
			WorkingMemoryAddress addrToPropagate) throws InterruptedException {
		return waitForPropagation(addrToPropagate, 0);
	}

	/**
	 * wait that all pending changes have been propagated through the system.
	 * 
	 * @param timeout
	 *            timeout in milliseconds (zero for infinite waiting)
	 * @throws InterruptedException
	 */
	public synchronized boolean waitForPropagation(long timeout)
			throws InterruptedException {
		long startTime = System.currentTimeMillis();

		while (pendingChanges.size() > 0) {
			component.log(this.getClass().getSimpleName() + ": waiting for "
					+ pendingChanges.size() + " pending triggers");
			wait(timeout);
			if (timeout > 0)
				if (System.currentTimeMillis() > startTime + timeout) {
					component.log(this.getClass().getSimpleName()
							+ ": timeout while waiting for propagation of "
							+ pendingChanges.size() + "triggers");
					return false;
				}
		}
		return true;
	}

	/**
	 * wait that all pending changes have been propagated through the system.
	 * 
	 * @throws InterruptedException
	 */
	public synchronized boolean waitForPropagation()
			throws InterruptedException {
		return waitForPropagation(0);
	}
}
