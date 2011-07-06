/**
 * 
 */
package castutils.experimentation;

import java.util.HashMap;
import java.util.Map;

import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class PlannerStopWatch extends ManagedComponent {

	Map<String, StopWatch> taskWatches;
	private WorkingMemoryChangeReceiver monitorTask;

	public PlannerStopWatch() {

		taskWatches = new HashMap<String, StopWatch>();

		// We monitor planning tasks to measure planning time
		monitorTask = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange wmc)
					throws CASTException {
				PlanningTask plan = null;
				if (wmc.operation != WorkingMemoryOperation.DELETE) {
					plan = getMemoryEntry(wmc.address, PlanningTask.class);
				}

				StopWatch watch = taskWatches.get(wmc.address.id);
				if (watch == null) {
					watch = new StopWatch("PlanningTask");
					taskWatches.put(wmc.address.id, watch);
				}

				watch.info(CASTUtils.toString(wmc));
				if (plan != null) {
					watch.info("STATUS = " + plan.planningStatus.toString());
				} else {
					watch.info("PlanningTask deleted");
				}

				switch (wmc.operation) {
				case ADD:
					watch.info(plan.id + "/" + plan.planningStatus.toString()
							+ "/" + plan.executionStatus.toString() + "/"
							+ plan.planningRetries + "/"
							+ plan.goals[0].goalString + "/" + plan.plan.length);
					watch.tic();
					break;
				case OVERWRITE:
					switch (plan.planningStatus) {
					case INPROGRESS:
					case PENDING:
						watch.info(plan.id + "/"
								+ plan.planningStatus.toString() + "/"
								+ plan.executionStatus.toString() + "/"
								+ plan.planningRetries + "/"
								+ plan.goals[0].goalString + "/"
								+ plan.plan.length);
						watch.tic();
						break;
					case ABORTED:
					case FAILED:
					case SUCCEEDED:
						if (watch.isRunning())
							watch.toc(plan.id + "/"
									+ plan.planningStatus.toString() + "/"
									+ plan.executionStatus.toString() + "/"
									+ plan.planningRetries + "/"
									+ plan.goals[0].goalString + "/"
									+ plan.plan.length);
						else
							log("watch is not running");
						break;
					default:
						if (!watch.isRunning())
							watch.tic();
					}
					break;
				case DELETE:
					if (watch.isRunning())
						watch.toc("DELETED");
					break;
				}

			}
		};

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		// TODO Auto-generated method stub
		super.start();
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(PlanningTask.class),
				monitorTask);
	}
}
