package castutils.experimentation;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;

import motivation.slice.PlanProxy;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import eu.cogx.planner.facade.PlannerFacade;

public class TourGiver extends ManagedComponent {

	private static final String CONFIG_DELAY_MS = "--delay-ms";
	private static final int DEFAULT_DELAY = 5000;
	private static final String CONFIG_PLACES = "--places";
	private static final String CONFIG_GOAL = "--goal";
	private String goalString = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey(CONFIG_PLACES)) {
			StringTokenizer st = new StringTokenizer(config.get(CONFIG_PLACES));
			while (st.hasMoreTokens()) {
				placeIds.add(Integer.parseInt(st.nextToken()));
			}
		}
		for (Integer i : placeIds) {
			println("  tour place: " + i);
		}
		if (config.containsKey(CONFIG_DELAY_MS)) {
			delayTimeInMs = Integer.parseInt(config.get(CONFIG_DELAY_MS));
		}
		println("start after " + delayTimeInMs + " ms");
		if (config.containsKey(CONFIG_GOAL)) {
			goalString = config.get(CONFIG_GOAL);
		}
		println("start after " + delayTimeInMs + " ms");

	}

	private List<Integer> placeIds = new ArrayList<Integer>();
	private long delayTimeInMs = DEFAULT_DELAY;
	private PlannerFacade planner;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		planner = PlannerFacade.get(this);

		sleepComponent(delayTimeInMs);
		for (Integer io : placeIds) {
			try {
				println("goto place " + io);
				NavCommand nc = createNavCommand(io);
				Completion completion = executeNavCommand(nc);
				if (completion != Completion.COMMANDSUCCEEDED) {
					getLogger().warn(
							"navigation command did not succeed: "
									+ completion.toString());
				}
			} catch (CASTException e) {
				logException(e);
			} catch (InterruptedException e) {
				logException(e);
			}
		}
		println("tour finished");
		if (goalString != null) {
			println("planning for goal " + goalString);
			submitPlan(goalString);
		}
	}

	private Completion executeNavCommand(NavCommand navCommand)
			throws CASTException, InterruptedException {
		String id = newDataID();
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id), queue);
		addToWorkingMemory(id, "spatial.sa", navCommand);
		Completion completion = Completion.COMMANDFAILED;
		while (isRunning()) {
			WorkingMemoryChange ev = queue.take();
			if (ev.operation == WorkingMemoryOperation.OVERWRITE) {
				NavCommand nc = getMemoryEntry(ev.address, NavCommand.class);
				completion = nc.comp;
				if (completion == Completion.COMMANDPENDING
						|| completion == Completion.COMMANDINPROGRESS)
					continue;
				else
					break;
			}
		}
		removeChangeFilter(queue);
		return completion;
	}

	/**
	 * @param io
	 * @return
	 */
	private NavCommand createNavCommand(Integer io) {
		return new NavCommand(CommandType.GOTOPLACE, Priority.NORMAL,
				new long[] { io.longValue() }, new double[0], new double[0],
				new double[0], new double[0], StatusError.UNKNOWN,
				Completion.COMMANDPENDING);
	}

	public void submitPlan(String goalStr) {
		LinkedList<Goal> goals = new LinkedList<Goal>();
		Goal goal = new Goal(-1, -1, (String) goalStr, false);
		goals.add(goal);
		try {
			WMEntryQueueElement<PlanningTask> res = planner.plan(goals, true)
					.get();
			if (res == null)
				getLogger().warn("PLANNING FAILED");
			else {
				addToWorkingMemory(newDataID(), new PlanProxy(
						res.getEvent().address));
			}
		} catch (Exception e) {
			logException(e);
		}

	}

}
