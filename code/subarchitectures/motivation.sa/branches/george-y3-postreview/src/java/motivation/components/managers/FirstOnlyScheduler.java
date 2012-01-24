package motivation.components.managers;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;

public class FirstOnlyScheduler extends SimpleScheduler {

	@Override
	protected void runComponent() {
		Map<WorkingMemoryAddress, Motive> surfacedGoals = null;

		boolean hasActiveGoal = false;
		try {
			while (isRunning() && !hasActiveGoal) {

				if (!changesToProcess()) {
					continue;
				}

				surfacedGoals = motives.getMapByStatus(MotiveStatus.SURFACED);
				if (surfacedGoals.size() > 0) {
					Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
					Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();

					Map<WorkingMemoryAddress, Motive> prioritySet = getMaxPrioritySet(surfacedGoals);

					WMEntryQueueElement<PlanningTask> plan = doPlanning(
							prioritySet, possibleGoals, impossibleGoals);

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

					hasActiveGoal = true;
				}
			}
			getLogger().info(
					"finished " + FirstOnlyScheduler.class.getSimpleName());
		} catch (CASTException e) {
			logException(e);
			return;
		} catch (Exception e) {
			logException(e);
			return;
		}

	}

}
