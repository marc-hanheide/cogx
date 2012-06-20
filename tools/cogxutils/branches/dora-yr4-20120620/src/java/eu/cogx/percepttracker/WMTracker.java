package eu.cogx.percepttracker;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.Map.Entry;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.interfaces.WorkingMemory;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.PointerMap;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * This implements a generic tracker for {@link Belief}s. It listens for the
 * addition or overwrite of a "From" belief tries to match it to existing "To"
 * beliefs and if find a matching one it updates the corresponding "To" belief.
 * In order to do this job it requires a {@link MatcherFunction}.
 * 
 * @author marc
 * 
 * @param <From>
 *            the generic type to listen for and generate from
 * @param <To>
 *            the generic that is generated and synchronized to the From type
 */
/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 * @param <From>
 * @param <To>
 */
public class WMTracker<From extends dBelief, To extends dBelief> extends
		CASTHelper implements Runnable {

	/**
	 * an interface to be implemented by classes that should serve as a
	 * {@link MatcherFunction} for {@link WMTracker}.
	 * 
	 * @author marc
	 * 
	 * @param <From2>
	 *            the generic type of the source of the transfer
	 * @param <To2>
	 *            the generic type of the sink of the transfer
	 */
	public interface MatcherFunction<From2 extends Ice.ObjectImpl, To2 extends Ice.ObjectImpl> {
		/**
		 * @param from
		 * @return true if this matching function can actually handle this type
		 *         of belief
		 */
		public boolean canHandle(From2 from);

		/**
		 * create a new "To" belief from an unmatched "From" belief.
		 * 
		 * @param idToCreate
		 *            the new ID to create (@see
		 *            {@link WorkingMemoryWriterComponent})
		 * @param wmc
		 *            the {@link WorkingMemoryChange} that caused the creation
		 * @param from
		 *            the element that is used to initialize the new one
		 * @return the new instance, ready to be written to
		 *         {@link WorkingMemory}
		 * @throws IncompatibleAssignmentException
		 */
		public To2 create(WorkingMemoryAddress idToCreate,
				WorkingMemoryChange wmc, From2 from)
				throws IncompatibleAssignmentException;

		/**
		 * match a "From" to a "To" belief
		 * 
		 * @param wmc
		 *            the {@link WorkingMemoryChange} that caused the match
		 * @param from
		 *            the belief that is compared
		 * @param to
		 *            the belief to compare to
		 * @return a likelihood of matching between 0.0 and 1.0
		 */
		public double match(WorkingMemoryChange wmc, From2 from, To2 to);

		/**
		 * update an existing belief with a new observation
		 * 
		 * @param wmc
		 *            the {@link WorkingMemoryChange} that caused the update
		 * @param from
		 *            the belief that is used to update the existing one
		 * @param to
		 *            the belief to be updated
		 * @throws IncompatibleAssignmentException
		 */
		public void update(WorkingMemoryChange wmc, From2 from, To2 to)
				throws IncompatibleAssignmentException;

	}

	/**
	 * create a new instance of the WMTracker, usually propagating
	 * {@link PerceptBelief} to {@link GroundedBelief}.
	 * 
	 * @param <FromS>
	 * @param <ToS>
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param transferFunction
	 * @param wm2wm
	 * @return
	 */
	public static <FromS extends dBelief, ToS extends dBelief> WMTracker<FromS, ToS> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<ToS> toType, MatcherFunction<FromS, ToS> transferFunction,
			PointerMap<?> wm2wm) {

		return new WMTracker<FromS, ToS>(component, fromType, toType,
				transferFunction, wm2wm, null);
	}

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param <FromS>
	 *            From type
	 * @param <ToS>
	 *            To type
	 * @param component
	 *            the underlying component employed for memory operations
	 * @param fromType
	 *            the from class object
	 * @param toType
	 *            the to class object
	 * @param transferFunction
	 *            an instance of a {@link MatcherFunction}
	 * @param wm2wm
	 * @param inSA
	 *            the subarchitecture id used to create the grounded beliefs in
	 * @return a new object
	 */
	public static <FromS extends dBelief, ToS extends dBelief> WMTracker<FromS, ToS> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<ToS> toType, MatcherFunction<FromS, ToS> transferFunction,
			PointerMap<?> wm2wm, String inSA) {

		return new WMTracker<FromS, ToS>(component, fromType, toType,
				transferFunction, wm2wm, inSA);
	}

	final Set<WorkingMemoryAddress> lockedAddresses = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());

	private PointerMap<?> wm2wmMap;

	/**
	 * a synchronized event queue that ensure that any new insert notification
	 * returns immediately while the worker thread can sequentially work on all
	 * the events.
	 */
	private WMEventQueue entryQueue;

	/**
	 * the from type
	 */
	protected final Class<From> fromType;

	/**
	 * the to type
	 */
	protected final Class<To> toType;

	/** the registered TransferFunction for PerceptMonitor */
	protected MatcherFunction<From, To> matcherFunction;

	// private ExecutorService executor; " sa: " + toWMA.subarchitecture + 

	/**
	 * a map to keep track of all the relations between From and To beliefs
	 */
	protected WMView<To> allTrackedBeliefs;

	protected String createInSA = null;

	protected boolean shouldPropagateDeletion = false;

	private ExecutorService executor = Executors.newCachedThreadPool();

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param transferFunction
	 * @param wm2wmMap
	 * @param inSA
	 *            the SA this should be created in.
	 */
	protected WMTracker(ManagedComponent component, Class<From> fromType,
			Class<To> toType, MatcherFunction<From, To> transferFunction,
			PointerMap<?> wm2wmMap, String inSA) {
		super(component);
		this.fromType = fromType;
		this.toType = toType;
		this.matcherFunction = transferFunction;
		entryQueue = new WMEventQueue();
		this.wm2wmMap = wm2wmMap;
		this.allTrackedBeliefs = WMView.create(this.component, toType);
		this.createInSA = inSA;
		// this.executor = Executors.newCachedThreadPool();
	}

	protected WorkingMemoryAddress bestMatch(WorkingMemoryChange wmc, From from) {
		double bestProb = 0.0;
		WorkingMemoryAddress result = null;
		HashSet<Entry<WorkingMemoryAddress, To>> copy = new HashSet<Entry<WorkingMemoryAddress, To>>(
				allTrackedBeliefs.entrySet());
		for (Entry<WorkingMemoryAddress, To> i : copy) {
			double m = matcherFunction.match(wmc, from, i.getValue());
			if (m > bestProb) {
				result = i.getKey();
				bestProb = m;
			}
		}
		return result;
	}

	private void handleNewObservation(final WorkingMemoryChange ev)
			throws CASTException, IncompatibleAssignmentException {
		final From from = component.getMemoryEntry(ev.address, fromType);
		if (!matcherFunction.canHandle(from))
			return;
		final WorkingMemoryAddress matchingWMA = bestMatch(ev, from);
//		executor.submit(new Runnable() {

//			@Override
//			public void run() {
				try {
					if (matchingWMA == null) {
						log("found no match for observation "
								+ CASTUtils.toString(ev.address));
						String id = component.newDataID();

						WorkingMemoryAddress toWMA = new WorkingMemoryAddress(
								id, createInSA);
						log("creating new tracked belief with "
								+ CASTUtils.toString(toWMA));

						To to = matcherFunction.create(toWMA, ev, from);
						if (to != null) {
							log("have created new belief with type=" + to.type
									+ " (" + toWMA.id + ")");
							matcherFunction.update(ev, from, to);
							manageHistory(ev, from, to);
							log("have filled with values:" + to.type + " ("
									+ toWMA.id + " sa: " + toWMA.subarchitecture + "), ready to write to WM now.");
							component.addToWorkingMemory(toWMA, to);
							log("written to WM, insert into map now:" + to.type
									+ " (" + toWMA.id + ")");
							wm2wmMap.put(ev.address, toWMA);
						} else {
							getLogger().warn(
									"failed to create a corresponding belief for "
											+ from.type);
						}
					} else {
						try {

							try {
								log("found match "
										+ CASTUtils.toString(matchingWMA)
										+ " for observation "
										+ CASTUtils.toString(ev.address));
								// make sure we have exclusive access
								lock(matchingWMA);
								//
								// component.lockEntry(matchingWMA,
								// WorkingMemoryPermissions.LOCKEDODR);
								To to = component.getMemoryEntry(matchingWMA,
										toType);
								log("updating belief "
										+ CASTUtils.toString(matchingWMA));
								matcherFunction.update(ev, from, to);
								manageHistory(ev, from, to);
								component.overwriteWorkingMemory(matchingWMA,
										to);
								wm2wmMap.put(ev.address, matchingWMA);
							} finally {
								unlock(matchingWMA);
								//
								// component.unlockEntry(matchingWMA);
							}
						} catch (InterruptedException e) {
							getLogger().error("interrupted: ", e);
						}
					}
				} catch (IncompatibleAssignmentException e) {
					getLogger().error("during update:", e);
				} catch (CASTException e) {
					getLogger().error("during update:", e);
				}
//			}
//		});

	}

	private void handleOverwriteObservation(final WorkingMemoryChange ev,
			final WorkingMemoryAddress matchingWMA) throws CASTException,
			IncompatibleAssignmentException {
		final From from = component.getMemoryEntry(ev.address, fromType);
		if (!matcherFunction.canHandle(from))
			return;

		executor.submit(new Runnable() {

			@Override
			public void run() {
				try {
					// make sure we have exclusive access
					// try {
					// component.lockComponent();
					// component.lockEntry(matchingWMA,
					// WorkingMemoryPermissions.LOCKEDOD);
					// } finally {
					// component.unlockComponent();
					// }
					lock(matchingWMA);
					To to = component.getMemoryEntry(matchingWMA, toType);
					matcherFunction.update(ev, from, to);
					manageHistory(ev, from, to);
					try {
						// component.lockComponent();
						component.overwriteWorkingMemory(matchingWMA, to);
					} finally {
						// component.unlockComponent();
						unlock(matchingWMA);
					}
				} catch (IncompatibleAssignmentException e) {
					getLogger().error("during update:", e);
				} catch (InterruptedException e) {
					getLogger().error("during update:", e);
				} catch (CASTException e) {
					getLogger().error("during update:", e);
					// } finally {
					// try {
					// try {
					// component.lockComponent();
					// component.unlockEntry(matchingWMA);
					// } finally {
					// component.unlockComponent();
					// }
					// } catch (CASTException e) {
					// getLogger().error("during unlock of " + matchingWMA, e);
					// }
				}
			}
		});
	}

	/**
	 * @return the shouldPropagateDeletion
	 */
	public boolean isShouldPropagateDeletion() {
		return shouldPropagateDeletion;
	}

	private void lock(WorkingMemoryAddress wma) throws InterruptedException {

		while (lockedAddresses.contains(wma)) {
			lockedAddresses.wait();
		}
		lockedAddresses.add(wma);
	}

	private void manageHistory(WorkingMemoryChange ev, From from, To to) {
		if (!(to.hist instanceof CASTBeliefHistory)) {
			to.hist = new CASTBeliefHistory(
					new ArrayList<WorkingMemoryPointer>(1),
					new ArrayList<WorkingMemoryPointer>(1));
		}
		CASTBeliefHistory bh = (CASTBeliefHistory) to.hist;
		WorkingMemoryPointer ancesterPointer = new WorkingMemoryPointer(
				ev.address, ev.type);
		if (!bh.ancestors.contains(ancesterPointer)) {
			bh.ancestors.add(ancesterPointer);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		log("register listeners for type " + fromType.getSimpleName());
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				fromType, WorkingMemoryOperation.ADD), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				fromType, WorkingMemoryOperation.OVERWRITE), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				fromType, WorkingMemoryOperation.DELETE), entryQueue);

		if (createInSA == null)
			createInSA = component.getSubarchitectureID();

		try {
			allTrackedBeliefs.start();
		} catch (UnknownSubarchitectureException e1) {
			getLogger().fatal("cannot start WMView", e1);
			return;
		}

		start();
		try {
			while (component.isRunning()) {
				try {
					final WorkingMemoryChange ev = entryQueue.take();
					switch (ev.operation) {
					case ADD: {
						handleNewObservation(ev);
						break;
					}
					case OVERWRITE: {
						WorkingMemoryAddress toWMA = wm2wmMap.get(ev.address);
						if (toWMA == null)
							handleNewObservation(ev);
						else {
							// we assume a static assignment here. if an
							// observation has once been linked to a tracked
							// belief, overwrite will only be propagated to
							// this one!
							handleOverwriteObservation(ev, toWMA);
						}
						break;
					}
					case DELETE: {

						if (shouldPropagateDeletion) {
							WorkingMemoryAddress adr = wm2wmMap.get(ev.address);
							
							//nah: this could be null if we don't track this entry
							if (adr != null) {
								try {
									component.lockEntry(adr,
											WorkingMemoryPermissions.LOCKEDODR);
									component.deleteFromWorkingMemory(adr);
									log("deleted belief "
											+ CASTUtils.toString(adr));
								} catch (CASTException e) {
									component.unlockEntry(adr);
								}
							}

						}
						wm2wmMap.remove(ev.address);
						break;
					}
					}
				} catch (CASTException e) {
					getLogger().error("in run: ", e);
				} catch (IncompatibleAssignmentException e) {
					getLogger().error("in run", e);
				}
			}
		} catch (InterruptedException e) {
			getLogger().warn("interrupted in run, leaving thread: ", e);
			return;
		} finally {
			try {
				component.removeChangeFilter(entryQueue);
			} catch (SubarchitectureComponentException e) {
				getLogger().error("while removing change filter: ", e);
			}
		}
	}

	/**
	 * @param shouldPropagateDeletion
	 *            the shouldPropagateDeletion to set
	 */
	public void setShouldPropagateDeletion(boolean shouldPropagateDeletion) {
		this.shouldPropagateDeletion = shouldPropagateDeletion;
	}

	/**
	 * to be overwritten by any subclass if required. Is called after the
	 * registration of ChangeReceivers and before actually entering the run loop
	 * (@see {@link Runnable}).
	 */
	protected void start() {
		// to be overwritten

	}

	private void unlock(WorkingMemoryAddress wma) throws InterruptedException {
		lockedAddresses.remove(wma);
		lockedAddresses.notifyAll();
	}

	// }

}
