/**
 * 
 */
package motivation.components.managers;

import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import motivation.components.managers.comparators.AgeComparator;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import motivation.util.castextensions.WMLock;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public class Scheduler extends ManagedComponent {
	WMMotiveSet motives;
	WMLock wmLock;

	private int maxPlannedMotives;
	private Comparator<? super Motive> motiveComparator;
	private WMMotiveEventQueue relevantEventQueue;

	/**
	 * @param specificType
	 * @throws CASTException
	 */
	public Scheduler() throws CASTException {
		super();
		// defaults
		motives = WMMotiveSet.create(this);
		motiveComparator = new AgeComparator();
		relevantEventQueue = new WMMotiveEventQueue();
		maxPlannedMotives = 3;
		wmLock = new WMLock(this, "SchedulerManagerSync");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub
		super.configure(arg0);

		String arg;
		// parsing stuff
		arg = arg0.get("--orderPolicy");
		if (arg != null) {
			String className = this.getClass().getPackage().getName()
					+ ".comparators." + arg.trim() + "Comparator";
			try {
				this.getClass().getPackage().getName();
				println("add type '" + className + "'");
				ClassLoader.getSystemClassLoader().loadClass(className);
				Class<Comparator<? super Motive>> cl = (Class<Comparator<? super Motive>>) Class
						.forName(className);
				motiveComparator = cl.newInstance();
			} catch (ClassNotFoundException e) {
				println("trying to register for a class that doesn't exist.");
				e.printStackTrace();
			} catch (InstantiationException e) {
				println("failed to get an instance.");
				e.printStackTrace();
			} catch (IllegalAccessException e) {
				println("failed to get an instance.");
				e.printStackTrace();
			}
		}

		arg = arg0.get("--maxPlannedMotives");
		if (arg != null) {
			maxPlannedMotives = Integer.parseInt(arg);
		}
		println("maxPlannedMotives is " + maxPlannedMotives);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		motives.start();
		motives.setStateChangeHandler(new MotiveStateTransition(null,
				MotiveStatus.SURFACED), relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(MotiveStatus.SURFACED,
				MotiveStatus.SURFACED), relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.ACTIVE, null), relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.SURFACED, null), relevantEventQueue);
	}

	void scheduleMotives() {
		try {
			log("scheduleMotives: wait to acquire lock");
			wmLock.lock();
			// TODO: big hack to wait for all propagation
			sleepComponent(1000);
			while(!relevantEventQueue.isEmpty())
				try {
					relevantEventQueue.take();
				} catch (InterruptedException e1) {
				}
			Map<WorkingMemoryAddress, Motive> surfacedMotives = motives
					.getMapByStatus(MotiveStatus.SURFACED);
			Map<WorkingMemoryAddress, Motive> activeMotives = motives
					.getMapByStatus(MotiveStatus.ACTIVE);

			if (surfacedMotives.isEmpty())
				return;

			int freeCapacity = Math.max(maxPlannedMotives
					- activeMotives.size(), 0);
			// if we have no capacity left check if we need to reschedule due to
			// priorities
			if (freeCapacity == 0) {
				log("checking if we have to reschedule due to priorities");
				int priorityActives = maxPriority(activeMotives.values());
				int prioritySurfaced = maxPriority(surfacedMotives.values());

				log("max priority in actives: " + priorityActives);
				log("max priority in surfaced: " + prioritySurfaced);
				// reschedule if we have no active motives or one of the
				// surfaced
				// motives is high priority forcing rescheduling
				if (priorityActives < prioritySurfaced) {
					log("there are some surfaced motives that have a highe priority. Forcing rescheduling");
					// deactivate all motives
					motives.setState(MotiveStatus.SURFACED, activeMotives
							.values());
					activeMotives.clear();
					freeCapacity = maxPlannedMotives;
				}
			}
			// if there is no capacity we do nothing
			if (freeCapacity == 0)
				return;

			List<Motive> sortedMotives = new LinkedList<Motive>(surfacedMotives
					.values());
			// rank the motives according to the comparator
			Collections.sort(sortedMotives, motiveComparator);

			int rankCount = 0;
			int numberToSchedule = Math.min(sortedMotives.size(), freeCapacity);
			log("ok, let's schedule " + numberToSchedule + " motives of the "
					+ surfacedMotives.size() + " surfaced ones.");
			for (Motive m : sortedMotives.subList(0, numberToSchedule)) {
				m.status = MotiveStatus.ACTIVE;
				m.tries++;
				m.rank = rankCount++;
				try {
					lockEntry(m.thisEntry, WorkingMemoryPermissions.LOCKEDO);
					getMemoryEntry(m.thisEntry, Motive.class);
					overwriteWorkingMemory(m.thisEntry, m);
				} catch (DoesNotExistOnWMException e) {
					// safely ignore
				} finally {
					unlockEntry(m.thisEntry);
				}
			}
		} catch (CASTException e) {
			e.printStackTrace();
		} finally {
			log("unlocking lock");
			wmLock.unlock();
		}

	}

	private int maxPriority(Collection<Motive> collection) {
		int result = -1;
		for (Motive m : collection) {
			result = Math.max(m.priority.value(), result);
		}

		return result;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		super.runComponent();
		try {
			wmLock.initialize();
			while (isRunning()) {
				log("runComponent loop");
				// check if we have any active motives already... then we don't
				// schedule

				scheduleMotives();
				while (isRunning()) {
					if ((relevantEventQueue.poll(1, TimeUnit.SECONDS)) != null) {
						// relevantEventQueue.clear();
						break;
					}

				}
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
