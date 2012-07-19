/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.managers;

import java.util.HashMap;
import java.util.Map;

import motivation.components.filters.DelayFilterDisplayClient;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class Scheduler extends AbstractScheduler {

	private final DelayFilterDisplayClient m_display = DelayFilterDisplayClient.getClient();

	public Scheduler() {
		super();
	}

	@Override
	public void start() {
		super.start();
		m_display.connectIceClient(this);
	}

	protected static boolean containsUnsurfaced(
			Map<WorkingMemoryAddress, Motive> possibleGoals) {
		for (Motive m : possibleGoals.values()) {
			// if the motive has a higher priority then we have so far
			if (m.status == MotiveStatus.UNSURFACED) {
				return true;
			}
		}
		return false;
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

				if (!changesToProcess()) {
					continue;
				}
				surfacedGoals = motives.getMapByStatus(MotiveStatus.SURFACED);
				activeGoals = motives.getMapByStatus(MotiveStatus.ACTIVE);
				log("we currently have " + surfacedGoals.size()
						+ " surfaced goals.");
				log("we currently have " + activeGoals.size()
						+ " active goals.");

				
				m_display.updateActiveGoals(activeGoals);
				
				if (executionFuture != null
						&& (executionFuture.isDone() || executionFuture
								.isCancelled())) {

					log("execution has finished... de-activating goals!");

					// now done in flagAchievedGoals
					// setStatus(activeGoals, MotiveStatus.SURFACED);
					// sleepComponent(1000);

					flagAchievedGoals();

					executionFuture = null;
					continue;
				}

				// lockSet(surfacedGoals);
				// lockSet(activeGoals);
				// if (activeGoals.isEmpty()) {
				// if (executionFuture == null)
				// executionFuture.cancel(true);
				// executionFuture = null;
				// }

				if (executionFuture == null) {

					// we are currently not executing
					// if we are not yet executing, we just check all
					// surfaced goals whether they can be planned for, and
					// activate all those that can be integrated in the plan.
					// Those that cannot be integrated are marked as impossible.
					// When creating the set of goals, we consider the
					// priorities. Only if the set of a higher priority is empty
					// we schedule the lower priority.

					// if there are no goals surfaced, we have nothing to do...
					if (surfacedGoals.isEmpty()) {
						log("there are no goals surfaced, so we wait until we get something to do");
						continue;
					}
					Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
					Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();

					Map<WorkingMemoryAddress, Motive> prioritySet = getMaxPrioritySet(surfacedGoals);

					WMEntryQueueElement<PlanningTask> plan = doPlanning(
							prioritySet, possibleGoals, impossibleGoals);

					// George specific. Increment tries for impossible goals
					Map<WorkingMemoryAddress, Motive> updatedImpossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();

					for (WorkingMemoryAddress addr : impossibleGoals.keySet()) {
						Motive updated = recordFailure(addr);
						updatedImpossibleGoals.put(addr, updated);
					}
					impossibleGoals = updatedImpossibleGoals;

					if (plan != null && possibleGoals.size() > 0) {
						log("we have a plan to execute: "
								+ plan.getEntry().firstActionID + "costs: "
								+ plan.getEntry().costs);
						if (getMaxPriority(possibleGoals).compareTo(
								getMaxPriority(impossibleGoals)) < 0) {
							// TODO here we should better replan for a subset to
							// make sure that high prioritiy goals are always
							// planned for.
							getLogger()
									.warn("there is a goal impossible which is of higher priority than the possible ones. This should better be treated in the future!");
						}

						setStatus(possibleGoals, MotiveStatus.ACTIVE);

						log("start executing for " + possibleGoals.size()
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
					log("maxExecutePriority=" + maxExecutePriority.name());
					MotivePriority maxSurfacePriority = getMaxPriority(surfacedGoals);
					log("maxSurfacePriority=" + maxSurfacePriority.name());

					println("active size: " + activeGoals.size());
					println("surfaced size: " + surfacedGoals.size());

					if (containsUnsurfaced(activeGoals)
							|| activeGoals.size() == 0) {
						if (!executionFuture.isDone()) {
							executionFuture.cancel(true);
						}
						log("deactivated all currently executing goals as some were unsurfaced, let's check again");
						continue;
					} else if (maxExecutePriority.compareTo(maxSurfacePriority) < 0) {
						// if there are surfaced motives that are of same or
						// high priority than the activated ones we have to
						// reschedule.
						log("there are goals which are not in the current plan which should be tried to include into it. Canceling execution.");
						if (!executionFuture.isDone()) {
							executionFuture.cancel(true);
						}
						setStatus(activeGoals, MotiveStatus.SURFACED);
						log("deactiveated all currently executed goals, let's check again");
						continue;
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

	private Motive recordFailure(WorkingMemoryAddress wma)
			throws ConsistencyException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		Motive motive = null;
		try {
			lockEntry(wma, WorkingMemoryPermissions.LOCKEDOD);
			motive = getMemoryEntry(wma, Motive.class);
			motive.tries++;
			overwriteWorkingMemory(wma, motive);
			log("recorded failure for:" + motive.goal);
		} catch (CASTException e) {
			getLogger().warn(
					"CASTException when incrementing try count: " + e.message);
		} finally {
			if (this.holdsLock(wma.id, wma.subarchitecture)) {
				unlockEntry(wma);
			}
		}
		return motive;

	}

}
