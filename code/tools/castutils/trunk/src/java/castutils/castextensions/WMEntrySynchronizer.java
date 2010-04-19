/**
 * 
 */
package castutils.castextensions;

import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import Ice.ObjectImpl;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * This implements a monitor for specifc types of low-level percepts that are
 * propagated to the Binder. It implements Runnable so that several monitors can
 * be executed concurrently.
 * 
 * @author marc
 * 
 */
public class WMEntrySynchronizer<From extends Ice.ObjectImpl, To extends Ice.ObjectImpl>
		extends CASTHelper implements Runnable {

	public interface TransferFunction<From2 extends Ice.ObjectImpl, To2 extends Ice.ObjectImpl> {

		public boolean transform(WorkingMemoryChange wmc, From2 from, To2 to);

		public To2 create(WorkingMemoryAddress idToCreate,
				WorkingMemoryChange wmc, From2 from);

	}

	private Map<WorkingMemoryAddress, WorkingMemoryAddress> wm2wmMap;

	/** thread pool size for asynchronous refernce resolution */
	private static final int MAX_THREADS = 1;

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

	/** an executor service used for running asynchronous belief propagation */
	protected final ExecutorService executor;

	protected final Set<WorkingMemoryOperation> ops;

	/**
	 * @param c
	 */
	protected WMEntrySynchronizer(ManagedComponent c, Class<From> fromType,
			Class<To> toType, TransferFunction<From, To> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		super(c);
		this.fromType = fromType;
		this.toType = toType;
		this.transferFunction = transferFunction;
		entryQueue = new WMEventQueue();
		executor = Executors.newFixedThreadPool(MAX_THREADS);
		this.ops = ops;
		this.wm2wmMap = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>());
	}

	/**
	 * create new synchronizer only for given memory operations
	 * 
	 * @param <FromS>
	 * @param <ToS>
	 * @param component
	 * @param fromType
	 * @param toType
	 * @param tf
	 * @param wmOps
	 * @return a new object
	 */
	public static <FromS extends ObjectImpl, ToS extends ObjectImpl> WMEntrySynchronizer<FromS, ToS> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<ToS> toType, TransferFunction<FromS, ToS> tf,
			Set<WorkingMemoryOperation> wmOps) {

		return new WMEntrySynchronizer<FromS, ToS>(component, fromType, toType,
				tf, wmOps);
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
			while (true) {
				try {
					final WorkingMemoryChange ev = entryQueue.take();
					switch (ev.operation) {
					case ADD: {
						final From from = component.getMemoryEntry(ev.address,
								fromType);
						final String id = component.newDataID();
						final WorkingMemoryAddress toWMA = new WorkingMemoryAddress(
								id, component.getSubarchitectureID());
						final To to = transferFunction.create(toWMA, ev, from);
						if (to != null) {
							if (transferFunction.transform(ev, from, to)) {
								component.addToWorkingMemory(id, to);
								log("remember mapping "
										+ CASTUtils.toString(ev.address)
										+ " => " + CASTUtils.toString(toWMA));
								wm2wmMap.put(ev.address, toWMA);
							}

						}
						break;
					}
					case OVERWRITE: {
						if (ops.contains(WorkingMemoryOperation.DELETE)) {
							final From from = component.getMemoryEntry(
									ev.address, fromType);
							WorkingMemoryAddress toWMA = wm2wmMap
									.get(ev.address);
							boolean isNew = false;
							To to;
							if (toWMA == null) {
								isNew = true;
								final String id = component.newDataID();
								toWMA = new WorkingMemoryAddress(id, component
										.getSubarchitectureID());
								to = component.getMemoryEntry(toWMA, toType);
							} else {
								isNew = false;
								to = transferFunction.create(toWMA, ev, from);
							}
							if (to != null) {
								if (transferFunction.transform(ev, from, to)) {
									if (isNew) {
										component.addToWorkingMemory(toWMA, to);
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
										component.overwriteWorkingMemory(toWMA,
												to);
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
					logger.error("in run: ", e);
				}
			}
		} catch (InterruptedException e) {
			logger.warn("interrupted in run: ", e);
		} finally {
			try {
				component.removeChangeFilter(entryQueue);
			} catch (SubarchitectureComponentException e) {
				logger.error("while removing change filter: ", e);
			}
		}
	}

	protected void start() {
		// to be overwritten

	}

}
