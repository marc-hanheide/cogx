/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.managers;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveView;
import motivation.util.WMMotiveView.MotiveStateTransition;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
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
		ChangeHandler<Motive> {

	private static final int TIME_TO_WAIT_TO_SETTLE = 100;
	private static final int TIME_TO_WAIT_FOR_CHANGE = 5000;
	final PlannerFacade planner = new PlannerFacade(this);
	final ExecutorFacade executor = new ExecutorFacade(this);

	final WMMotiveView motives = WMMotiveView.create(this);
	private WorkingMemoryChange lastChange;

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
				log("lock all surfaced and activated entries");
				// get all the surfaced motives
				surfacedGoals = motives.getMapByStatus(MotiveStatus.SURFACED);
				activeGoals = motives.getMapByStatus(MotiveStatus.ACTIVE);
				log("we currently have " + surfacedGoals.size()
						+ " surfaced goals.");
				log("we currently have " + activeGoals.size()
						+ " active goals.");

				lockSet(surfacedGoals);
				lockSet(activeGoals);
				if (activeGoals.isEmpty()) {
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
//					WMEntryQueueElement<PlanningTask> plan = doPriorityAwarePlanning(
//							surfacedGoals, possibleGoals, impossibleGoals);
					if (plan != null) {
						log("we have a plan to execute: "
								+ plan.getEntry().firstActionID + "costs: "
								+ plan.getEntry().costs);
						plan.getEntry().executePlan = true;
						// trigger overwrite to cause plan to be executed
						overwriteWorkingMemory(plan.getEvent().address, plan
								.getEntry());
						executor.setPlan(plan.getEvent().address);
						executor.call();
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

					// ask the planner for which of the surfaced goals it can
					// generate plans

				}
			}
		} catch (CASTException e) {
			logException(e);
			return;
		} catch (Exception e) {
			logException(e);
			return;
		} finally {
			log("unlock all goals");
			for (WorkingMemoryAddress wma : surfacedGoals.keySet()) {
				if (holdsLock(wma.id, wma.subarchitecture)) {
					try {
						unlockEntry(wma);
					} catch (CASTException e) {
						logException(e);
					}
				}
			}

		}

	}

//	private WMEntryQueueElement<PlanningTask> doPriorityAwarePlanning(
//			Map<WorkingMemoryAddress, Motive> surfacedGoals,
//			Map<WorkingMemoryAddress, Motive> possibleGoals,
//			Map<WorkingMemoryAddress, Motive> impossibleGoals) throws Exception {
//		WMEntryQueueElement<PlanningTask> result = null;
//		List<MotivePriority> mpl = Arrays.asList(MotivePriority.values());
//		Collections.sort(mpl);
//		for (MotivePriority mp : mpl) {
//			if (mp == MotivePriority.UNSURFACE)
//				continue;
//			Map<WorkingMemoryAddress, Motive> subSet = subSetByPriority(
//					surfacedGoals, mp);
//			if (subSet.isEmpty())
//				return null;
//			result = doPlanning(subSet, possibleGoals, impossibleGoals);
//			if (result == null) {
//				return null;
//			}
//			log("planning task returned "
//					+ result.getEntry().planningStatus.name() + "plan length: "
//					+ result.getEntry().plan.length);
//			MotivePriority maxPossiblePrio = getMaxPriority(possibleGoals);
//			MotivePriority maxImpossiblePrio = getMaxPriority(impossibleGoals);
//			log("maxPossiblePrio=" + maxPossiblePrio + ", maxImpossiblePrio="
//					+ maxImpossiblePrio);
//			if (maxImpossiblePrio.compareTo(maxPossiblePrio) <= 0)
//				break;
//			log("we still have goal with higher priority that are not in the plan, so we have to reduce to goal with priority >"
//					+ mp.name());
//		}
//		return result;
//	}

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
			Map<WorkingMemoryAddress, Motive> surfacedGoals,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals) throws Exception {
		Map<String, WorkingMemoryAddress> goal2motiveMap = new HashMap<String, WorkingMemoryAddress>();

		Set<Goal> goals=new HashSet<Goal>();
		for (Motive m:surfacedGoals.values())
			goals.add(m.goal);
		planner.setGoals(goals);
		
		for (Entry<WorkingMemoryAddress, Motive> m : surfacedGoals.entrySet()) {
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

		WMEntryQueueElement<PlanningTask> result = planner.call();
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

		// mark all motives POSSIBLE that could be planned for
		impossibleGoals.putAll(surfacedGoals);
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
	}

	private MotivePriority getMaxPriority(
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

	private void lockSet(Map<WorkingMemoryAddress, Motive> set)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		for (WorkingMemoryAddress wma : set.keySet()) {
			lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
		}

	}

	private Map<WorkingMemoryAddress, Motive> subSetByPriority(
			Map<WorkingMemoryAddress, Motive> surfacedGoals, MotivePriority mp) {
		Map<WorkingMemoryAddress, Motive> result = new HashMap<WorkingMemoryAddress, Motive>();
		for (Entry<WorkingMemoryAddress, Motive> m : surfacedGoals.entrySet()) {
			if (m.getValue().priority.compareTo(mp) >= 0) {
				result.put(m.getKey(), m.getValue());
			}
		}
		return result;
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
