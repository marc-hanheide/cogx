/**
 * 
 */
package castutils.castextensions;

import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import Ice.ObjectImpl;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;

/**
 * This implements a monitor for specific types in WM that are propagated to
 * another (often more high-level) type in WM. It implements Runnable so that
 * such a {@link WMEntrySynchronizer} can be run in a separate Thread.
 * 
 * @see WMTypeAlignment
 * @author marc
 * 
 * @param <From>
 *            the generic type to listen for and generate from
 * @param <To>
 *            the generic that is generated and synchronized to the From type
 */
public class WMEntrySynchronizer<From extends Ice.ObjectImpl, To extends Ice.ObjectImpl>
		extends CASTHelper implements Runnable {

	/**
	 * an interface to be implemented by classes that should serve as a
	 * {@link TransferFunction} for {@link WMEntrySynchronizer}.
	 * 
	 * @author marc
	 * 
	 * @param <From2>
	 *            the generic type of the source of the transfer
	 * @param <To2>
	 *            the generic type of the sink of the transfer
	 */
	public interface TransferFunction<From2 extends Ice.ObjectImpl, To2 extends Ice.ObjectImpl> {

		/**
		 * fill the values of the sink object (the object being transferred
		 * into)
		 * 
		 * @param wmc
		 *            the {@link WorkingMemoryChange} that triggers the update.
		 *            This is always an event affecting an entry of the From2
		 *            type.
		 * @param from
		 *            the object that has changed and needs to be synchronized
		 *            to the "to" objects
		 * @param to
		 *            the object to be updated by this method call.
		 * @return true iff the transform was successful and the To memory entry
		 *         should be updated
		 */
		public boolean transform(WorkingMemoryChange wmc, From2 from, To2 to);

		/**
		 * create a new instance of the To type
		 * 
		 * @param idToCreate
		 *            the id assigned to this new entry (This method must not
		 *            actually modify any working memory).
		 * @param wmc
		 *            the {@link WorkingMemoryChange} that triggers the
		 *            creation. This is always an event affecting an entry of
		 *            the From2 type.
		 * @param from
		 *            the object that has changed and needs to be synchronized
		 *            to the "to" objects
		 * @return
		 */
		public To2 create(WorkingMemoryAddress idToCreate,
				WorkingMemoryChange wmc, From2 from);

	}

	private Map<WorkingMemoryAddress, WorkingMemoryAddress> wm2wmMap;

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
	protected TransferFunction<From, To> transferFunction;

	/**
	 * the set of {@link WorkingMemoryOperation} to check for.
	 * 
	 */
	protected final Set<WorkingMemoryOperation> ops;

	/**
	 * the subarchitecture to write the transformed object to
	 */
	protected final String toSA;

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param transferFunction
	 * @param ops
	 */
	protected WMEntrySynchronizer(ManagedComponent component,
			Class<From> fromType, Class<To> toType,
			TransferFunction<From, To> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		this(component, component.getSubarchitectureID(), fromType, toType,
				transferFunction, ops);
	}

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param transferFunction
	 * @param ops
	 */
	protected WMEntrySynchronizer(ManagedComponent component, String toSA,
			Class<From> fromType, Class<To> toType,
			TransferFunction<From, To> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		super(component);
		this.fromType = fromType;
		this.toType = toType;
		this.transferFunction = transferFunction;
		entryQueue = new WMEventQueue();
		this.ops = ops;
		this.wm2wmMap = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>());
		this.toSA = toSA;
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
	 *            an instance of a {@link TransferFunction}
	 * @param wmOps
	 *            the set of {@link WorkingMemoryOperation} that are
	 *            synchronized
	 * @return a new object
	 */
	public static <FromS extends ObjectImpl, ToS extends ObjectImpl> WMEntrySynchronizer<FromS, ToS> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<ToS> toType, TransferFunction<FromS, ToS> transferFunction,
			Set<WorkingMemoryOperation> wmOps) {

		return new WMEntrySynchronizer<FromS, ToS>(component, fromType, toType,
				transferFunction, wmOps);
	}

	/**
	 * create new synchronizer for all memory operations
	 * 
	 * @param <FromS>
	 * @param <ToS>
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param tf
	 * @return a new object
	 */
	public static <FromS extends ObjectImpl, ToS extends ObjectImpl> WMEntrySynchronizer<FromS, ToS> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<ToS> toType, TransferFunction<FromS, ToS> tf) {

		Set<WorkingMemoryOperation> ops = EnumSet
				.allOf(WorkingMemoryOperation.class);
		return new WMEntrySynchronizer<FromS, ToS>(component, fromType, toType,
				tf, ops);
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
		start();
		try {
			while (component.isRunning()) {
				try {
					final WorkingMemoryChange ev = entryQueue.take();
					switch (ev.operation) {
					case ADD: {
						final From from = component.getMemoryEntry(ev.address,
								fromType);
						final String id = component.newDataID();
						final WorkingMemoryAddress toWMA = new WorkingMemoryAddress(
								id, toSA);
						final To to = transferFunction.create(toWMA, ev, from);
						if (to != null) {
							if (transferFunction.transform(ev, from, to)) {
								component.addToWorkingMemory(toWMA, to);
								log("remember mapping "
										+ CASTUtils.toString(ev.address)
										+ " => " + CASTUtils.toString(toWMA));
								wm2wmMap.put(ev.address, toWMA);
							}

						}
						break;
					}
					case OVERWRITE: {
						if (ops.contains(WorkingMemoryOperation.OVERWRITE)) {
							From from = null;
							try {
								from = component.getMemoryEntry(ev.address,
										fromType);
							} catch (DoesNotExistOnWMException e) {
								component.getLogger().warn(
										"object already removed...");

							}
							WorkingMemoryAddress toWMA = wm2wmMap
									.get(ev.address);
							boolean isNew = false;
							To to;
							try {
								if (toWMA == null && from != null) {
									isNew = true;
									final String id = component.newDataID();
									toWMA = new WorkingMemoryAddress(id, toSA);
									to = transferFunction.create(toWMA, ev,
											from);
								} else {
									isNew = false;
									component.lockEntry(toWMA,
											WorkingMemoryPermissions.LOCKEDOD);
									to = component
											.getMemoryEntry(toWMA, toType);
								}
								if (to != null) {
									if ((from != null)
											&& transferFunction.transform(ev,
													from, to)) {
										if (isNew) {
											component.addToWorkingMemory(toWMA,
													to);
											log("added to WM. remember mapping "
													+ CASTUtils
															.toString(ev.address)
													+ " => "
													+ CASTUtils.toString(toWMA));
											wm2wmMap.put(ev.address, toWMA);
										} else {
											log("overwritten in WM: "
													+ CASTUtils
															.toString(ev.address)
													+ " => "
													+ CASTUtils.toString(toWMA));
											component.overwriteWorkingMemory(
													toWMA, to);
										}
									} else {
										if (!isNew) {
											wm2wmMap.remove(ev.address);
											log("removed from WM: "
													+ CASTUtils
															.toString(ev.address)
													+ " => "
													+ CASTUtils.toString(toWMA));
											component
													.deleteFromWorkingMemory(toWMA);
										}
									}

								}
							} finally {
								if (!isNew) {
									try {
										component.unlockEntry(toWMA);
									} catch (DoesNotExistOnWMException e) {
										component
												.log("tried to unlock a non-existing entry");
									}
								}
							}
						}
						break;
					}
					case DELETE: {
						if (ops.contains(WorkingMemoryOperation.DELETE)) {
							WorkingMemoryAddress toWMA = wm2wmMap
									.get(ev.address);

							if (toWMA != null) {
								wm2wmMap.remove(ev.address);
								log("removed from WM: "
										+ CASTUtils.toString(ev.address)
										+ " => " + CASTUtils.toString(toWMA));
								component.deleteFromWorkingMemory(toWMA);

							}
						}
						break;
					}

					}
				} catch (CASTException e) {
					getLogger().error("in run: ", e);
				} catch (InterruptedException e) {
					getLogger().warn("interrupted in run: ", e);
				}
			}
		} finally {
			try {
				component.removeChangeFilter(entryQueue);
			} catch (SubarchitectureComponentException e) {
				getLogger().error("while removing change filter: ", e);
			}
		}
	}

	/**
	 * to be overwritten by any subclass if required. Is called after the
	 * registration of ChangeReceivers and before actually entering the run loop
	 * (@see {@link Runnable}).
	 */
	protected void start() {
		// to be overwritten

	}

}
