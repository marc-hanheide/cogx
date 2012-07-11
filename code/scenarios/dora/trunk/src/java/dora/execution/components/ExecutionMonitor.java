package dora.execution.components;

import java.util.HashMap;
import java.util.Map;

import si.unilj.fri.cogx.v11n.core.DisplayClient;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import castutils.experimentation.StopWatch;
import execution.slice.Action;

public class ExecutionMonitor extends ManagedComponent {
	DisplayClient displayClient = new DisplayClient();
	private long startMillis;
	StopWatch swPlanningAction = new StopWatch("PlannerAction");
	StopWatch swExecAction = new StopWatch("ExecutionAction");
	Map<String, Long> exeTimeElapsed = new HashMap<String, Long>();
	Map<String, Long> plannedActTimeElapsed = new HashMap<String, Long>();
	Map<String, Long> exeCount = new HashMap<String, Long>();
	Map<String, Long> plannedActCount = new HashMap<String, Long>();

	@Override
	protected void start() {
		startMillis = System.currentTimeMillis();
		displayClient.connectIceClient(this);

		addChangeFilter(
				ChangeFilterFactory
						.createGlobalTypeFilter(autogen.Planner.Action.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange arg0)
							throws CASTException {
						try {
							processPlanningAction(arg0);
						} catch (CASTException e) {
							logException(e);
						}

					}
				});
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(Action.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange arg0)
							throws CASTException {
						try {
							processExecutionAction(arg0);
						} catch (CASTException e) {
							logException(e);
						}

					}
				});
	}

	protected void processExecutionAction(WorkingMemoryChange arg0)
			throws CASTException {
		switch (arg0.operation) {
		case ADD:
		case OVERWRITE:
			Action act = getMemoryEntry(arg0.address, Action.class);
			switch (act.status) {
			case ACCEPTED:
				swExecAction.tic();
				break;
			case COMPLETE:
				long timeElapsed = swExecAction.toc();
				String actName = act.getClass().getCanonicalName();
				updateTimings(timeElapsed, actName, exeTimeElapsed, exeCount);
				long avg = exeTimeElapsed.get(actName)
				/ exeCount.get(actName);
				displayClient.setHtml("Timing Execution Action", actName, "<p>"
						+ actName + ": " + avg + " ms</p>");
				println("AvgExecutionAction," + actName + "," + avg);

			}
			String text = "<p>[" + (System.currentTimeMillis() - startMillis)
					+ "] <b>" + act.getClass().getSimpleName() + ":</b>"
					+ act.status.toString() + " / " + act.success.toString()
					+ "</p>";
			displayClient.setHtml("Action", arg0.address.id, text);
			break;
		case DELETE:
			displayClient.setHtml("Action", arg0.address.id, "");
			break;
		}

	}

	protected static void updateTimings(long timeElapsed, String actName,
			Map<String, Long> timings, Map<String, Long> counts) {
		if (timings.containsKey(actName)) {
			timings.put(actName, timings.get(actName) + timeElapsed);
			counts.put(actName, counts.get(actName) + 1);
		} else {
			timings.put(actName, timeElapsed);
			counts.put(actName, 1l);
		}
	}

	protected void processPlanningAction(WorkingMemoryChange arg0)
			throws CASTException {
		switch (arg0.operation) {
		case ADD:
		case OVERWRITE:
			autogen.Planner.Action act = getMemoryEntry(arg0.address,
					autogen.Planner.Action.class);
			switch (act.status) {
			case PENDING:
				swPlanningAction.tic();
				break;
			case SUCCEEDED:
				long timeElapsed = swPlanningAction.toc();
				String actName = act.name;
				updateTimings(timeElapsed, actName, plannedActTimeElapsed,
						plannedActCount);
				long avg = plannedActTimeElapsed.get(actName)
						/ plannedActCount.get(actName);
				displayClient.setHtml("Timing Planned Action", actName, "<p>"
						+ actName + ": " + avg + " ms</p>");
				println("AvgDurationPlannedAction," + actName + "," + avg);
				break;
			}

			displayClient.setHtml("Planned-Action", arg0.address.id, "<p><b>"
					+ act.name + ":</b>" + act.status.toString() + " cost="
					+ act.cost + "</p>");
			break;
		case DELETE:
			break;

		}
	}

	@Override
	protected void configure(Map<String, String> config) {
		displayClient.configureDisplayClient(config);
	}

}
