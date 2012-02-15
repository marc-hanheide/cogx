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
 * This implements a generic Merger for {@link Belief}s. It listens for the
 * addition or overwrite of a "From" belief tries to match it to existing "To"
 * beliefs and if find a matching one it updates the corresponding "To" belief.
 * In order to do this job it requires a {@link MatcherFunction}.
 * 
 * @author marc
 * agnostic 
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
public class WMMerger<Src1 extends dBelief, Src2 extends dBelief, Dest extends dBelief> extends
		CASTHelper implements Runnable {

	/**
	 * an interface to be implemented by classes that should serve as a
	 * {@link MatcherFunction} for {@link WMMerger}.
	 * 
	 * @author marc
	 * 
	 * @param <From2>
	 *            the generic type of the source of the transfer
	 * @param <To2>
	 *            the generic type of the sink of the transfer
	 */
	public interface MatcherFunction<From2 extends Ice.ObjectImpl, Via extends Ice.ObjectImpl, To2 extends Ice.ObjectImpl> {
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
		public void update(WorkingMemoryChange wmc, From2 from, Via via, To2 to)
				throws IncompatibleAssignmentException;
				
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
				
		public WorkingMemoryAddress bestMatch(WorkingMemoryChange wmc, From2 from,
				WMView beliefView);

	}

	/**
	 * create a new instance of the WMMerger, usually propagating
	 * {@link PerceptBelief} to {@link GroundedBelief}.
	 * 
	 * @param <SrcS1>
	 * @param <SrcS2>
	 * @param <DestS>
	 * @param component
	 * @param srcType1
	 * @param srcType2
	 * @param destType
	 * @param transferFunction
	 * @param src1d
	 * @param dests2
	 * @param src2s
	 * @return
	 */
	public static <SrcS1 extends dBelief, SrcS2 extends dBelief, DestS extends dBelief> WMMerger<SrcS1, SrcS2, DestS> create(
			ManagedComponent component, Class<SrcS1> srcType1, Class<SrcS2> srcType2,
			Class<DestS> destType, MatcherFunction<SrcS1, SrcS2, DestS> transferFunction1, MatcherFunction<SrcS2, SrcS1, DestS> transferFunction2,
			PointerMap<?> srcDest, PointerMap<?> srcSrc) {

		return new WMMerger<SrcS1, SrcS2, DestS>(component, srcType1, srcType2, destType,
				transferFunction1, transferFunction2, srcDest, srcSrc, null);
	}

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param <SrcS1>
	 *            SrcS1 type
	 * @param <SrcS2>
	 							SrcS2 type	 
	 * @param <DestS>
	 *            DestS type
	 * @param component
	 *            the underlying component employed for memory operations
	 * @param srcType1
	 *            the src1 class object
	 * @param srcType2
	 *						the src2 class object	 
	 * @param destType
	 *            the dest class object
	 * @param transferFunction
	 *            an instance of a {@link MatcherFunction}
	 * @param src1d
	 							src1 to dest wm map
	 * @param dests2
	 							dest to src2 wm map
	 * @param src2s1
	 							src2 to src1 wm map
	 * @param inSA
	 *            the subarchitecture id used to create the grounded beliefs in
	 * @return a new object
	 */
	public static  <SrcS1 extends dBelief, SrcS2 extends dBelief, DestS extends dBelief> WMMerger<SrcS1, SrcS2, DestS> create(
			ManagedComponent component, Class<SrcS1> srcType1, Class<SrcS2> srcType2,
			Class<DestS> destType, MatcherFunction<SrcS1, SrcS2, DestS> transferFunction1,  MatcherFunction<SrcS2, SrcS1, DestS> transferFunction2,
			PointerMap<?> srcDest, PointerMap<?> srcSrc, String inSA) {

		return new WMMerger<SrcS1, SrcS2, DestS>(component, srcType1, srcType2, destType,
				transferFunction1, transferFunction2, srcDest, srcSrc, inSA);
	}

	final Set<WorkingMemoryAddress> lockedAddresses = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());

	private PointerMap<?> srcDestMap;
	private PointerMap<?> srcSrcMap;

	/**
	 * a synchronized event queue that ensure that any new insert notification
	 * returns immediately while the worker thread can sequentially work on all
	 * the events.
	 */
	private WMEventQueue entryQueue;

	/**
	 * the src types
	 */
	protected final Class<Src1> srcType1;
	protected final Class<Src2> srcType2;

	/**
	 * the to type
	 */
	protected final Class<Dest> destType;

	/** the registered TransferFunction for PerceptMonitor */
	protected MatcherFunction<Src1, Src2, Dest> matcherFunction1;
	protected MatcherFunction<Src2, Src1, Dest> matcherFunction2;

	// private ExecutorService executor;

	/**
	 * a map to keep track of all the relations between From and To beliefs
	 */
	protected WMView<Dest> allTrackedBeliefs;

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
	protected WMMerger(ManagedComponent component, Class<Src1> srcType1, Class<Src2> srcType2,
			Class<Dest> destType, MatcherFunction<Src1, Src2, Dest> transferFunction1,
			MatcherFunction<Src2, Src1, Dest> transferFunction2, PointerMap<?> srcDestMap,
			PointerMap<?> srcSrcMap, String inSA) {
		super(component);
		this.srcType1 = srcType1;
		this.srcType2 = srcType2;
		this.destType = destType;
		this.matcherFunction1 = transferFunction1;
		this.matcherFunction2 = transferFunction2;
		entryQueue = new WMEventQueue();
		this.srcDestMap = srcDestMap;
		this.srcSrcMap = srcSrcMap;
		this.allTrackedBeliefs = WMView.create(this.component, destType);
		this.createInSA = inSA;
		// this.executor = Executors.newCachedThreadPool();
	}
/*
	protected WorkingMemoryAddress bestMatch1(WorkingMemoryChange wmc, Src1 from) {
		double bestProb = 0.0;
		WorkingMemoryAddress result = null;
		HashSet<Entry<WorkingMemoryAddress, Dest>> copy = new HashSet<Entry<WorkingMemoryAddress, Dest>>(
				allTrackedBeliefs.entrySet());
		for (Entry<WorkingMemoryAddress, Dest> i : copy) {
			double m = matcherFunction1.match(wmc, from, i.getValue());
			if (m > bestProb) {
				result = i.getKey();
				bestProb = m;
			}
		}
		return result;
	}
	
		protected WorkingMemoryAddress bestMatch2(WorkingMemoryChange wmc, Src2 from) {
		double bestProb = 0.0;
		WorkingMemoryAddress result = null;
		HashSet<Entry<WorkingMemoryAddress, Dest>> copy = new HashSet<Entry<WorkingMemoryAddress, Dest>>(
				allTrackedBeliefs.entrySet());
		for (Entry<WorkingMemoryAddress, Dest> i : copy) {
			double m = matcherFunction2.match(wmc, from, i.getValue());
			if (m > bestProb) {
				result = i.getKey();
				bestProb = m;
			}
		}
		return result;
	}
*/
	private void handleNewObservation1(final WorkingMemoryChange ev)
			throws CASTException, IncompatibleAssignmentException {
		log("Got a new observation of type " + ev.type);
		final WorkingMemoryAddress matchingWMA;
		
		final Src1 from = component.getMemoryEntry(ev.address, srcType1);
		matchingWMA = matcherFunction1.bestMatch(ev, from, allTrackedBeliefs);
		if (!matcherFunction1.canHandle(from))
			return;
			
		executor.submit(new Runnable() {

			@Override
			public void run() {
				try {
					if (matchingWMA == null) {
						log("found no match for observation "
								+ CASTUtils.toString(ev.address));
						String id = component.newDataID();

						WorkingMemoryAddress toWMA = new WorkingMemoryAddress(
								id, createInSA);
						log("creating new tracked belief with "
								+ CASTUtils.toString(toWMA));

						Dest to = matcherFunction1.create(toWMA, ev, from);
						if (to != null) {
							log("have created new belief with type=" + to.type
									+ " (" + toWMA.id + ")");
							matcherFunction1.update(ev, from, to);
							manageHistory(ev, from, to);
							log("have filled with values:" + to.type + " ("
									+ toWMA.id + "), ready to write to WM now.");
							component.addToWorkingMemory(toWMA, to);
							log("written to WM, insert into map now:" + to.type
									+ " (" + toWMA.id + ")");
							
							srcDestMap.put(ev.address, toWMA);
							// temporarily put the destination as the key
							srcSrcMap.put(toWMA, ev.address);
							
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
								Dest to = component.getMemoryEntry(matchingWMA,
										destType);
								log("updating belief "
										+ CASTUtils.toString(matchingWMA));
								matcherFunction1.update(ev, from, to);
								manageHistory(ev, from, to);
								component.overwriteWorkingMemory(matchingWMA,
										to);
								srcDestMap.put(ev.address, matchingWMA);
								// temporarily put the destination as the key
								srcSrcMap.put(matchingWMA, ev.address);
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
			}
		});
	}
	
	private void handleNewObservation2(final WorkingMemoryChange ev)
			throws CASTException, IncompatibleAssignmentException {
		log("Got a new observation of type " + ev.type);
		final WorkingMemoryAddress matchingWMA;
		
		final Src2 from = component.getMemoryEntry(ev.address, srcType2);
		matchingWMA = matcherFunction2.bestMatch(ev, from, allTrackedBeliefs);
		if (!matcherFunction2.canHandle(from))
			return;
			
		executor.submit(new Runnable() {

			@Override
			public void run() {
				try {
					if (matchingWMA == null) {
						log("found no match for observation "
								+ CASTUtils.toString(ev.address));
						String id = component.newDataID();

						WorkingMemoryAddress toWMA = new WorkingMemoryAddress(
								id, createInSA);
						log("creating new tracked belief with "
								+ CASTUtils.toString(toWMA));

						Dest to = matcherFunction2.create(toWMA, ev, from);
						if (to != null) {
							log("have created new belief with type=" + to.type
									+ " (" + toWMA.id + ")");
							matcherFunction2.update(ev, from, to);
							manageHistory(ev, from, to);
							log("have filled with values:" + to.type + " ("
									+ toWMA.id + "), ready to write to WM now.");
							component.addToWorkingMemory(toWMA, to);
							log("written to WM, insert into map now:" + to.type
									+ " (" + toWMA.id + ")");
							srcDestMap.put(ev.address, toWMA);
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
								Dest to = component.getMemoryEntry(matchingWMA,
										destType);
								Src1 via = component.getMemoryEntry(srcSrcMap.get(matchingWMA),
										srcType1);
								log("updating belief "
										+ CASTUtils.toString(matchingWMA));
								matcherFunction2.update(ev, from, via, to);
								manageHistory(ev, from, to);
								component.overwriteWorkingMemory(matchingWMA,
										to);
								srcDestMap.put(ev.address, matchingWMA);
								srcSrcMap.put(ev.address, srcSrcMap.get(matchingWMA));
								srcSrcMap.put(srcSrcMap.get(matchingWMA), ev.address);
								srcSrcMap.remove(matchingWMA);
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
			}
		});
	}

	private void handleOverwriteObservation1(final WorkingMemoryChange ev,
			final WorkingMemoryAddress otherWMA, final WorkingMemoryAddress matchingWMA) throws CASTException,
			IncompatibleAssignmentException {
			
		if(otherWMA != null) {
			handleOverwriteObservation2(ev, otherWMA, ev.address, matchingWMA);
			return;
		}
			
		final Src1 from = component.getMemoryEntry(ev.address, srcType1);
		if (!matcherFunction1.canHandle(from))
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
					Dest to = component.getMemoryEntry(matchingWMA, destType);
					matcherFunction1.update(ev, from, to);
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
	
	private void handleOverwriteObservation2(final WorkingMemoryChange ev, final WorkingMemoryAddress fromWMA,
			final WorkingMemoryAddress otherWMA, final WorkingMemoryAddress matchingWMA) throws CASTException,
			IncompatibleAssignmentException {
		final Src2 from = component.getMemoryEntry(fromWMA, srcType2);
		final Src1 via = component.getMemoryEntry(otherWMA, srcType1);
		if (!matcherFunction2.canHandle(from) || !matcherFunction1.canHandle(via))
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
					Dest to = component.getMemoryEntry(matchingWMA, destType);
					matcherFunction2.update(ev, from, via, to);
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

	private void manageHistory(WorkingMemoryChange ev, dBelief from, Dest to) {
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
		log("register listeners for type " + srcType1.getSimpleName());
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType1, WorkingMemoryOperation.ADD), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType1, WorkingMemoryOperation.OVERWRITE), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType1, WorkingMemoryOperation.DELETE), entryQueue);
		
		log("register listeners for type " + srcType2.getSimpleName());			
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType2, WorkingMemoryOperation.ADD), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType2, WorkingMemoryOperation.OVERWRITE), entryQueue);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				srcType2, WorkingMemoryOperation.DELETE), entryQueue);

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
						if(ev.type == srcType1.getSimpleName())
							handleNewObservation1(ev);
						else if(ev.type == srcType2.getSimpleName())
							handleNewObservation2(ev);
							
						break;
					}
					case OVERWRITE: {
						if(ev.type == srcType1.getSimpleName()) {
							WorkingMemoryAddress toWMA = srcDestMap.get(ev.address);
							if (toWMA == null)
								handleNewObservation1(ev);
							else {
								// we assume a static assignment here. if an
								// observation has once been linked to a tracked
								// belief, overwrite will only be propagated to
								// this one!
								WorkingMemoryAddress otherWMA = srcSrcMap.get(ev.address);
								handleOverwriteObservation1(ev, otherWMA, toWMA);
							}
						}
						else if(ev.type == srcType2.getSimpleName()) {
							WorkingMemoryAddress toWMA = srcDestMap.get(ev.address);
							if (toWMA == null)
								handleNewObservation2(ev);
							else {
								// we assume a static assignment here. if an
								// observation has once been linked to a tracked
								// belief, overwrite will only be propagated to
								// this one!
									
								WorkingMemoryAddress otherWMA = srcSrcMap.get(ev.address);
								handleOverwriteObservation2(ev, ev.address, otherWMA, toWMA);
							}
						}
						break;
					}
					case DELETE: {

						if (shouldPropagateDeletion) {
							WorkingMemoryAddress adr = srcDestMap.get(ev.address);
							
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
						srcDestMap.remove(ev.address);
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
