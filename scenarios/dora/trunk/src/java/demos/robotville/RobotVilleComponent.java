package demos.robotville;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

import spatial.demo.PlacePatroller;
import spatial.demo.ShakeHeadBehaviour;

import motivation.slice.PlanProxy;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;
import eu.cogx.planner.facade.PlannerFacade;

public class RobotVilleComponent extends ManagedComponent {
	public static final String ROBOT_TAG = "ROBOT";

	Set<WorkingMemoryAddress> beliefWmas = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());
	private String robotID = null;
	WorkingMemoryAddress wmaPlanProxy = null;
	final RobotVilleJFrame frame = new RobotVilleJFrame(this);

	private PlannerFacade planner;

	private WorkingMemoryAddress wmaPlanningTask;

	final PlacePatroller m_patroller = new PlacePatroller(this);
	final Thread m_patrollerThread = new Thread(m_patroller);
	final ShakeHeadBehaviour m_shaker = new ShakeHeadBehaviour(this);
	final Thread m_shakerThread = new Thread(m_shaker);
	
	boolean isPatrolling = false;
	boolean isShaking = false;

	public void stopRunningTasks() {
		try {
			if (isPatrolling) {
				m_patroller.stopPatrolling();
				isPatrolling = false;
			}
			if (isShaking) {
				m_shaker.stopShaking();
				isShaking = false;
			}

			if (wmaPlanningTask != null) {
				deleteFromWorkingMemory(wmaPlanningTask);
				wmaPlanningTask = null;
			}
			if (wmaPlanProxy != null) {
				deleteFromWorkingMemory(wmaPlanProxy);
				wmaPlanProxy = null;
			}
		} catch (CASTException e) {
			logException(e);
		}
	}

	@Override
	protected void start() {
		planner = new PlannerFacade(this);
		m_patrollerThread.start();
		m_shakerThread.start();
		frame.setVisible(true);
		WorkingMemoryChangeReceiver wmcr = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				println("adding place for patrolling (if it's a trueplace)");
				m_patroller.addPlace(getMemoryEntry(_wmc.address, Place.class));
			}
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), wmcr);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.OVERWRITE), wmcr);

	}

	public void startPlannedAction(String string) {
		LinkedList<Goal> goals = new LinkedList<Goal>();
		try {
			Goal g = new Goal(-1, -1, substituteRobotBelief(string), false);
			goals.add(g);

			WMEntryQueueElement<PlanningTask> res = planner.plan(goals, true)
					.get();

			if (res == null) {
				frame.setStatus("planning failed");
				wmaPlanProxy = null;
				wmaPlanningTask = null;
			} else {
				wmaPlanProxy = new WorkingMemoryAddress(newDataID(),
						"planner.sa");
				wmaPlanningTask = res.getEvent().address;
				addToWorkingMemory(wmaPlanProxy, new PlanProxy(
						res.getEvent().address));
			}
		} catch (Exception e) {
			frame.setStatus(e.getMessage());
			logException(e);
		}

	}

	private String substituteRobotBelief(String goalString)
			throws CASTException {
		if (goalString.contains(ROBOT_TAG)) {
			String id = getRobotBeliefID();
			return goalString.replace(ROBOT_TAG, id);
		} else {
			return goalString;
		}
	}

	private NavCommand createHomeNavCommand() {
		NavCommand nc = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, null, null, null, null, null,
				StatusError.NONE, Completion.COMMANDPENDING);
		nc.pose = new double[3];
		nc.pose[0] = 0.0;
		nc.pose[1] = 0.0;
		nc.pose[2] = 0.0;

		nc.tolerance = new double[3];
		nc.tolerance[0] = 0.1;
		nc.tolerance[1] = 0.1;
		nc.tolerance[2] = Math.PI * 10.0 / 180.0;

		nc.destId = new long[1];
		nc.destId[0] = 0;
		nc.distance = new double[0];
		nc.angle = new double[0];
		return nc;
	}

	private String getRobotBeliefID() throws CASTException {
		if (robotID == null) {
			println("the ID of the robot's belief is not yet known, so we have to search for it...");
			for (WorkingMemoryAddress wma : beliefWmas) {
				GroundedBelief bg = getMemoryEntry(wma, GroundedBelief.class);
				if (bg.type.equals(LocalizedAgentTransferFunction.ROBOT)) {
					robotID = bg.id;
					println("the ID of the robot's belief has been found to be "
							+ robotID);
					break;
				}
			}
			assert (robotID != null);
		}
		return robotID;
	}

	public void goHome() {
		try {
			addToWorkingMemory(new WorkingMemoryAddress(newDataID(),
					"spatial.sa"), createHomeNavCommand());
		} catch (CASTException e) {
			logException(e);
		}

	}

	public void startIdleBehaviour() {
		m_shaker.startShaking();
		isShaking = true;
	}

	public void startPatrolBehaviour() {
		m_patroller.startPatrolling();
		isPatrolling = true;
	}

}
