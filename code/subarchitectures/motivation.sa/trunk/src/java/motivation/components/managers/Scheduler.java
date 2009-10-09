/**
 * 
 */
package motivation.components.managers;

import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import motivation.components.managers.comparators.AgeComparator;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public class Scheduler extends ManagedComponent {
	WMMotiveSet motives;

	private int maxPlannedMotives;
	private Comparator<? super Motive> motiveComparator;
	private WMMotiveEventQueue surfacedEventQueue;

	/**
	 * @param specificType
	 */
	public Scheduler() {
		super();
		// defaults
		motives = WMMotiveSet.create(this);
		motiveComparator = new AgeComparator();
		surfacedEventQueue = new WMMotiveEventQueue();
		maxPlannedMotives = 3;
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
				MotiveStatus.SURFACED), surfacedEventQueue);
	}

	void scheduleMotives() {
		// if we don't have anything to do... just quit.
		log("scheduleMotives");
		Set<Motive> activeMotives = motives
				.getSubsetByStatus(MotiveStatus.ACTIVE);

		if (activeMotives.size() > 0) {
			log("there are still active motives... so we don't reschedule any new");
			// this scheduler does no rescheduling so far
			return;
		}

		Set<Motive> surfacedMotives = motives
				.getSubsetByStatus(MotiveStatus.SURFACED);

		if (surfacedMotives.size() < 0)
			return;

		try {
			log("ok, let's schedule "+ maxPlannedMotives + " of the "+ surfacedMotives.size() + "surfaced ones.");
			int freeCapacity = Math.max(0, maxPlannedMotives
					- activeMotives.size());
			List<Motive> sortedMotives = new LinkedList<Motive>(surfacedMotives);
			// rank the motives according to the comparator
			Collections.sort(sortedMotives, motiveComparator);

			int rankCount = 0;
			for (Motive m : sortedMotives.subList(0, Math.min(sortedMotives
					.size(), freeCapacity))) {
				m.status = MotiveStatus.ACTIVE;
				m.tries++;
				m.rank = rankCount++;
				try {
					lockEntry(m.thisEntry, WorkingMemoryPermissions.LOCKEDO);
					overwriteWorkingMemory(m.thisEntry, m);
				} catch (DoesNotExistOnWMException e) {
					// safely ignore
				}
				finally {
					unlockEntry(m.thisEntry);
				}
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

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
			while (isRunning()) {
				log("runComponent loop");
				// retrieve next event
				surfacedEventQueue.poll(1, TimeUnit.SECONDS);
				scheduleMotives();
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
