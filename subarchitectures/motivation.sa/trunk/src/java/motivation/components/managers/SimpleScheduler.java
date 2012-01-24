/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.managers;

import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class SimpleScheduler extends AbstractScheduler {

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
				if (executionFuture != null
						&& (executionFuture.isDone() || executionFuture
								.isCancelled())) {
					log("execution has finished... de-activate goals!");
					setStatus(activeGoals, MotiveStatus.SURFACED);
					sleepComponent(1000);
					// check if there are any goals that need to be deleted
					// if goal is achieved already we should remove it!
					log("flag achieved goals");
					flagAchievedGoals();

					executionFuture = null;
					continue;
				}

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
									.warn("there is a goal impossible which is of higher priority than the possible ones. This should better be treated in the future!");
						}

						setStatus(possibleGoals, MotiveStatus.ACTIVE);
						log("start executing for " + possibleGoals.size()
								+ " possible goals");
						executionFuture = executor
								.execute(plan.getEvent().address);
					}

				} else {
					debug("check if there are opportunities... NOT IMPLEMENTED YET, so we continue execution");

				}
			}

		} catch (CASTException e) {
			logException(e);
			return;
		} catch (Exception e) {
			logException(e);
			return;
		}

	}

}
