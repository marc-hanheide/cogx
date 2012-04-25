/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.managers;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PlanProxy;
import motivation.util.WMMotiveView;
import motivation.util.WMMotiveView.MotiveStateTransition;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.castextensions.WMView.ChangeHandler;
import eu.cogx.planner.facade.PlannerFacade;
import facades.ExecutorFacade;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractScheduler extends ManagedComponent implements
		ChangeHandler<Motive>, Callable<Object> {

	public static final int MAX_GOALS = 5;

	private static final int MINIMUM_PLANNER_TIMEOUT = 30;

	private static final int TIME_TO_WAIT_FOR_CHANGE = 5000;

	private static final int TIME_TO_WAIT_TO_SETTLE = 1000;

	protected static MotivePriority getMaxPriority(
			Map<WorkingMemoryAddress, Motive> possibleGoals) {
		MotivePriority result = MotivePriority.UNSURFACE;
		for (Motive m : possibleGoals.values()) {
			// if the motive has a higher priority then we have so far
			if (result.compareTo(m.priority) < 0) {
				result = m.priority;
			}

		}
		return result;
	}

	private boolean checkAgain = true;
	protected Future<PlanProxy> executionFuture = null;
	final ExecutorFacade executor;

	private WorkingMemoryChange lastChange = null;

	final WMMotiveView motives;

	final PlannerFacade planner;

	/**
	 * 
	 */
	public AbstractScheduler() {
		executor = new ExecutorFacade(this);
		planner = new PlannerFacade(this);
		motives = WMMotiveView.create(this);
	}

	@Override
	public Object call() throws Exception {
		synchronized (this) {
			checkAgain = true;
		}
		return null;
	}

	/**
	 * check if we either have data to process, or we still have so many changes
	 * pending that it's worth to wait a little longer.
	 * 
	 * @return true if there are changes pending, but no new ones have arrived
	 *         in the last milli seconds.
	 */
	protected boolean changesToProcess() {

		try {
			if (executionFuture != null
					&& (executionFuture.isDone() || executionFuture
							.isCancelled()))
				return true;

			synchronized (this) {
				if (checkAgain) {
					checkAgain = false;
					return true;
				}

				if (lastChange == null) {
					this.wait(TIME_TO_WAIT_FOR_CHANGE);
					if (lastChange == null) {
						debug("no relevant event, so we continue waiting");
						return true;
					}
				}
				lastChange = null;
			}
			Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
			synchronized (this) {
				if (lastChange != null) {
					debug("there is another change pending, let's collect them all before actally start planning");
					return false;
				}
			}
		} catch (InterruptedException e) {
			logException(e);
			return false;
		}
		return true;
	}

	protected WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals)
			throws InterruptedException, ExecutionException {
		int timeout = MINIMUM_PLANNER_TIMEOUT;
		for (Motive g : goalsToPlanFor.values()) {
			timeout += g.maxPlanningTime;
		}
		return doPlanning(goalsToPlanFor, possibleGoals, impossibleGoals,
				timeout);
	}

	private WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals, int timeoutMs)
			throws InterruptedException, ExecutionException {
		log("try to plan for " + goalsToPlanFor.size() + " goals; timeout="
				+ timeoutMs + "ms.");
		Map<String, WorkingMemoryAddress> goal2motiveMap = new HashMap<String, WorkingMemoryAddress>();

		List<Goal> goals = new LinkedList<Goal>();
		for (Motive m : goalsToPlanFor.values()) {
			if (!m.goal.goalString.isEmpty())
				// set all those that are of HIGHEST priority to HARD goals
				if (m.priority == MotivePriority.HIGH)
					m.goal.importance = -1;
			goals.add(m.goal);
		}

		for (Entry<WorkingMemoryAddress, Motive> m : goalsToPlanFor.entrySet()) {
			if (!m.getValue().goal.goalString.isEmpty()) {
				if (goal2motiveMap.containsKey(m.getValue().goal.goalString)) {
					getLogger()
							.warn("the goal "
									+ m.getValue().goal.goalString
									+ " is already associated with motive"
									+ goal2motiveMap.get(m.getValue().goal.goalString));
					continue;
				} else {
					goal2motiveMap
							.put(m.getValue().goal.goalString, m.getKey());
				}
			}
		}

		impossibleGoals.clear();
		possibleGoals.clear();
		impossibleGoals.putAll(goalsToPlanFor);
		try {
			WMEntryQueueElement<PlanningTask> result = null;

			if (timeoutMs > 0) {
				result = planner.plan(goals).get(timeoutMs, TimeUnit.SECONDS);
			} else {
				result = planner.plan(goals).get();
			}
			if (result == null) {
				getLogger().warn("failed to create a plan at all");
				return null;
			}
			if (result.getEntry().planningStatus != Completion.SUCCEEDED) {
				getLogger().warn(
						"planner returned with result "
								+ result.getEntry().planningStatus.name());
				return null;
			}

			// mark all motives POSSIBLE that have been planned for
			for (Goal o : result.getEntry().goals) {
				WorkingMemoryAddress correspMotiveWMA = goal2motiveMap
						.get(o.goalString);
				assert (correspMotiveWMA != null);
				log("looking for planning result for goal " + o.goalString
						+ " (#actions=" + result.getEntry().plan.length + "): "
						+ o.isInPlan);

				if (o.isInPlan) {
					log("  goal " + o.goalString + " is possible!");
					possibleGoals.put(correspMotiveWMA,
							impossibleGoals.remove(correspMotiveWMA));
				}
			}

			return result;
		} catch (TimeoutException e) {
			getLogger().warn("planner timeout", e);
			return null;
		}
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
			throws CASTException {
		debug("received change event: " + CASTUtils.toString(wmc));
		synchronized (this) {
			lastChange = wmc;
			this.notifyAll();
		}

	}

	protected Map<WorkingMemoryAddress, Motive> getMaxPrioritySet(
			Map<WorkingMemoryAddress, Motive> surfacedGoals) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		MotivePriority max = getMaxPriority(surfacedGoals);
		int capacity = MAX_GOALS;
		for (Entry<WorkingMemoryAddress, Motive> e : surfacedGoals.entrySet()) {
			if (e.getValue().priority == max) {
				capacity--;
				if (capacity < 0)
					break;
				result.put(e.getKey(), e.getValue());
			}
		}
		return result;
	}

	private void lockSet(Map<WorkingMemoryAddress, Motive> set)
			throws UnknownSubarchitectureException {
		debug("lockSet(): locking component");
		try {
			lockComponent();
			getLogger().debug("lockSet(): component locked");
			for (WorkingMemoryAddress wma : set.keySet()) {
				debug("lockSet(): locking entry " + wma.id);
				try {
					lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
				} catch (DoesNotExistOnWMException e) {
					getLogger()
							.warn("cannot lock entry that already disappeared. will remove it from the set now.",
									e);
					set.remove(wma);
				}
				debug("lockSet(): entry " + wma.id + " is locked");
			}
		} finally {
			unlockComponent();
			debug("lockSet(): component unlocked");
		}
	}

	@Override
	protected abstract void runComponent();

	protected void setStatus(Map<WorkingMemoryAddress, Motive> motiveMap,
			MotiveStatus status) throws UnknownSubarchitectureException,
			DoesNotExistOnWMException, ConsistencyException,
			PermissionException {
		try {
			lockSet(motiveMap);
			for (Entry<WorkingMemoryAddress, Motive> e : motiveMap.entrySet()) {
				Motive goal = getMemoryEntry(e.getKey(), Motive.class);
				goal.status = status;
				if (status == MotiveStatus.ACTIVE)
					goal.tries++;
				overwriteWorkingMemory(e.getKey(), goal);
			}
		} catch (CASTException e) {
			getLogger().warn("exception when setting status: ", e);
		} finally {
			unlockSet(motiveMap);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		try {
			motives.setStateChangeHandler(new MotiveStateTransition(
					MotiveStatus.WILDCARD, MotiveStatus.SURFACED), this);
			motives.setStateChangeHandler(new MotiveStateTransition(
					MotiveStatus.SURFACED, MotiveStatus.WILDCARD), this);
			this.executor.registerCallable(this);

			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	private void unlockSet(Map<WorkingMemoryAddress, Motive> set)
			throws UnknownSubarchitectureException, ConsistencyException {
		try {
			lockComponent();
			for (WorkingMemoryAddress wma : set.keySet()) {
				if (holdsLock(wma.id, wma.subarchitecture)) {
					try {
						unlockEntry(wma);
					} catch (DoesNotExistOnWMException e) {
						logException(
								"when unlocking "
										+ CASTUtils.toString(wma)
										+ " didn't exist anymore, remove it from the set",
								e);
						set.remove(wma);
					}
				}
			}
		} finally {
			unlockComponent();
		}
	}

	protected void flagAchievedGoals() throws CASTException {
		Set<Entry<WorkingMemoryAddress, Motive>> copySet = new HashSet<Entry<WorkingMemoryAddress, Motive>>(
				motives.entrySet());

		for (Entry<WorkingMemoryAddress, Motive> m : copySet) {
			Motive motive = m.getValue();
			if (PlannerFacade.get(this).isGoalAchieved(motive.goal.goalString)) {
				WorkingMemoryAddress wma = m.getKey();

				try {
					log("flag achieved goal " + motive.goal.goalString);
					lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
					motive = getMemoryEntry(wma, Motive.class);
					motive.status = MotiveStatus.COMPLETED;
					overwriteWorkingMemory(m.getKey(), motive);
				} catch (CASTException e) {
					getLogger().warn("CASTException when flagging goal as achieved: "+e.message);
				} finally {
					if (this.holdsLock(wma.id, wma.subarchitecture))
						unlockEntry(wma);
				}

			}
		}
	}

}
