/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.managers;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.apache.batik.svggen.font.table.MaxpTable;

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
public class Scheduler extends ManagedComponent implements
		ChangeHandler<Motive>, Callable<Object> {

	private final static List<MotivePriority> priorityThresholds = Arrays
			.asList(MotivePriority.values());

	private static final int TIME_TO_WAIT_TO_SETTLE = 100;

	private static final int TIME_TO_WAIT_FOR_CHANGE = 5000;

	private static MotivePriority getMaxPriority(
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

	private static Map<WorkingMemoryAddress, Motive> subSetByPriority(
			Map<WorkingMemoryAddress, Motive> surfacedGoals,
			MotivePriority minimumPriority) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, Motive> m : surfacedGoals.entrySet()) {
			if (m.getValue().priority.compareTo(minimumPriority) >= 0) {
				result.put(m.getKey(), m.getValue());
			}
		}
		return result;
	}

	final PlannerFacade planner = new PlannerFacade(this);

	final ExecutorFacade executor = new ExecutorFacade(this);
	final WMMotiveView motives = WMMotiveView.create(this);
	private WorkingMemoryChange lastChange = null;

	private Future<PlanProxy> executionFuture = null;

	private boolean checkAgain = true;

	/**
	 * 
	 */
	public Scheduler() {
		this.executor.registerCallable(this);
	}

	@Override
	public Object call() throws Exception {
		synchronized (this) {
			checkAgain = true;
		}
		return null;
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
			throws CASTException {
		log("received change event: " + CASTUtils.toString(wmc));
		synchronized (this) {
			lastChange = wmc;
			this.notifyAll();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		Map<WorkingMemoryAddress, Motive> surfacedGoals = null;
		Map<WorkingMemoryAddress, Motive> activeGoals = null;
		try {
			while (isRunning()) {

				if (!changesToProcess())
					continue;
				// get all the surfaced motives
				surfacedGoals = motives.getMapByStatus(MotiveStatus.SURFACED);
				activeGoals = motives.getMapByStatus(MotiveStatus.ACTIVE);
				log("we currently have " + surfacedGoals.size()
						+ " surfaced goals.");
				log("we currently have " + activeGoals.size()
						+ " active goals.");

				// lockSet(surfacedGoals);
				// lockSet(activeGoals);
				if (executionFuture != null && executionFuture.isDone()) {
					// we are currently not executing
					// if we are not yet executing, we just check all
					// surfaced goals whether they can be planned for, and
					// activate all those that can be integrated in the plan.
					// Those that cannot be integrated are marked as impossible.
					// When creating the set of goals, we consider the
					// priorities. Only if the set of a higher priority is empty
					// we schedule the lower priority.

					Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
					Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();

					WMEntryQueueElement<PlanningTask> plan = doPlanning(
							surfacedGoals, possibleGoals, impossibleGoals);

					if (plan != null) {
						log("we have a plan to execute: "
								+ plan.getEntry().firstActionID + "costs: "
								+ plan.getEntry().costs);
						if (getMaxPriority(possibleGoals).compareTo(
								getMaxPriority(impossibleGoals)) < 0) {
							// TODO here we should better replan for a subset to
							// make sure that high prioritiy goals are always
							// planned for.
							getLogger()
									.warn(
											"there is a goal impossible which is of higher priority than the possible ones. This should better be treated in the future!");
						}

						setStatus(possibleGoals, MotiveStatus.ACTIVE);
						log("start extecuting for " + possibleGoals.size()
								+ " possible goals");
						executionFuture = executor
								.execute(plan.getEvent().address);
					}

				} else {
					// First of all, we check for any new (surfaced) goal of
					// higher priority than the current plan. If we have a
					// higher priority, we interrupt the current execution,
					// de-activate all goals and start all over.
					// If we are actually already executing, we check if any of
					// the surfaced or impossible goals can be integrated into
					// the existing plan and at which costs. If we decide its
					// worth to change the plan, we activate those goals.

					MotivePriority maxExecutePriority = getMaxPriority(activeGoals);
					MotivePriority maxSurfacePriority = getMaxPriority(surfacedGoals);
					if (maxExecutePriority.compareTo(maxSurfacePriority) <= 0) {
						// if there are surfaced motives that are of same or
						// high priority than the activated ones we have to
						// reschedule.
						log("there are goals which are not in the current plan which should be tried to include into it. Canceling execution.");
						if (!executionFuture.isDone())
							executionFuture.cancel(true);
						setStatus(activeGoals, MotiveStatus.SURFACED);
					}

				}
			}
		} catch (CASTException e) {
			logException(e);
			return;
		} catch (Exception e) {
			logException(e);
			return;
		} finally {
			// log("unlock all goals");
			// for (WorkingMemoryAddress wma : surfacedGoals.keySet()) {
			// if (holdsLock(wma.id, wma.subarchitecture)) {
			// try {
			// unlockEntry(wma);
			// } catch (CASTException e) {
			// logException(e);
			// }
			// }
			// }

		}

	}

	private void setStatus(Map<WorkingMemoryAddress, Motive> activeGoals,
			MotiveStatus status) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {
		try {
			lockSet(activeGoals);
			for (Entry<WorkingMemoryAddress, Motive> e : activeGoals.entrySet()) {
				Motive goal = e.getValue();
				goal.status = status;
				overwriteWorkingMemory(e.getKey(), goal);
			}
		} finally {
			unlockSet(activeGoals);
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
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {

	}

	/**
	 * check if we either have data to process, or we still have so many changes
	 * pending that it's worth to wait a little longer.
	 * 
	 * @return true if there are changes pending, but no new ones have arrived
	 *         in the last milli seconds.
	 */
	private boolean changesToProcess() {

		try {
			synchronized (this) {
				if (checkAgain) {
					checkAgain = false;
					return true;
				}

				if (lastChange == null) {
					this.wait(TIME_TO_WAIT_FOR_CHANGE);
					if (lastChange == null) {
						getLogger().info(
								"no relevant event, so we continue waiting");
						return false;
					}
				}
				lastChange = null;
			}
			Thread.sleep(TIME_TO_WAIT_TO_SETTLE);
			synchronized (this) {
				if (lastChange != null) {
					getLogger()
							.info(
									"there is another change pending, let's collect them all before actally start planning");
					return false;
				}
			}
		} catch (InterruptedException e) {
			logException(e);
			return false;
		}
		return true;
	}

	private WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals)
			throws InterruptedException, ExecutionException {
		return doPlanning(goalsToPlanFor, possibleGoals, impossibleGoals, -1);
	}

	private WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals, int timeoutMs)
			throws InterruptedException, ExecutionException {
		Map<String, WorkingMemoryAddress> goal2motiveMap = new HashMap<String, WorkingMemoryAddress>();

		List<Goal> goals = new LinkedList<Goal>();
		for (Motive m : goalsToPlanFor.values())
			goals.add(m.goal);

		for (Entry<WorkingMemoryAddress, Motive> m : goalsToPlanFor.entrySet()) {
			if (!m.getValue().goal.goalString.isEmpty()) {
				if (goal2motiveMap.containsKey(m.getValue().goal.goalString)) {
					getLogger().warn(
							"the goal "
									+ m.getValue().goal.goalString
									+ " is already associated with motive"
									+ goal2motiveMap
											.get(m.getValue().goal.goalString));
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
			WMEntryQueueElement<PlanningTask> result = planner.plan(goals).get(
					timeoutMs, TimeUnit.MILLISECONDS);

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
					possibleGoals.put(correspMotiveWMA, impossibleGoals
							.remove(correspMotiveWMA));
				}
			}

			return result;
		} catch (TimeoutException e) {
			logException("planner timeout", e);
			return null;
		}
	}

	private void lockSet(Map<WorkingMemoryAddress, Motive> set)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		for (WorkingMemoryAddress wma : set.keySet()) {
			lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
		}

	}

	private void unlockSet(Map<WorkingMemoryAddress, Motive> set) {
		for (WorkingMemoryAddress wma : set.keySet()) {
			if (holdsLock(wma.id, wma.subarchitecture)) {
				try {
					unlockEntry(wma);
				} catch (CASTException e) {
					logException("while unlocking entries: ", e);
				}
			}
		}
	}

}
