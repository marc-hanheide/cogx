package castutils.experimentation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Priority;
import SpatialData.StatusError;

import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;

public class TourGiver extends ManagedComponent {

	private static final String CONFIG_DELAY_MS = "--delay-ms";
	private static final int DEFAULT_DELAY = 5000;
	private static final String CONFIG_PLACES = "--places";

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

	}

	private List<Integer> placeIds = new ArrayList<Integer>();
	private long delayTimeInMs = DEFAULT_DELAY;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
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
	}

	private Completion executeNavCommand(NavCommand navCommand)
			throws CASTException, InterruptedException {
		String id = newDataID();
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id), queue);
		addToWorkingMemory(id, navCommand);
		Completion completion = Completion.COMMANDFAILED;
		while (isRunning()) {
			WorkingMemoryChange ev = queue.take();
			if (ev.operation == WorkingMemoryOperation.OVERWRITE) {
				NavCommand nc = getMemoryEntry(id, NavCommand.class);
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
}
