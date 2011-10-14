package dora.execution.components;

import java.util.Map;

import si.unilj.fri.cogx.v11n.core.DisplayClient;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import execution.slice.Action;

public class ExecutionMonitor extends ManagedComponent {
	DisplayClient displayClient = new DisplayClient();
	private long startMillis; 

	@Override
	protected void start() {
		startMillis = System.currentTimeMillis();
		displayClient.connectIceClient(this);

		addChangeFilter(ChangeFilterFactory
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
				}, ChangeReceiverPriority.HIGH);
		addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(Action.class),
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
				}, ChangeReceiverPriority.HIGH);
	}

	protected void processExecutionAction(WorkingMemoryChange arg0)
			throws CASTException {
		switch (arg0.operation) {
		case ADD:
		case OVERWRITE:
			Action act = getMemoryEntry(arg0.address, Action.class);
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

	protected void processPlanningAction(WorkingMemoryChange arg0)
			throws CASTException {
		switch (arg0.operation) {
		case ADD:
		case OVERWRITE:
			autogen.Planner.Action act = getMemoryEntry(arg0.address,
					autogen.Planner.Action.class);
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
