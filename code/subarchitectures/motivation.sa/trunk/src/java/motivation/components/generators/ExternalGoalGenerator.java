/**
 * 
 */
package motivation.components.generators;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import motivation.factories.MotiveFactory;
import motivation.slice.ExternalGoalServer;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.MotiveStatus;
import motivation.slice._ExternalGoalServerOperations;
import motivation.slice._ExternalGoalServerTie;
import Ice.Current;
import autogen.Planner.Goal;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;

/**
 * @author marc
 * 
 */
public class ExternalGoalGenerator extends ManagedComponent implements
		_ExternalGoalServerOperations {
	public static final String ROBOT_TAG = "ROBOT";
	public static final double DEFAULT_CONSTANT_GAIN = 1.0;
	public static final float DEFAULT_CONSTANT_COSTS = (float) 1.0;

	// give planning at maximum 60 seconds
	public static final int DEFAULT_MAX_PLANNING_TIME = 120;
	// give the max at max 60 minutes to reach goal
	public static final int DEFAULT_MAX_EXECUTION_TIME = 60 * 60;

	public double constantGain = DEFAULT_CONSTANT_GAIN;
	public float constantCosts = DEFAULT_CONSTANT_COSTS;
	public int maxPlanningTime = DEFAULT_MAX_PLANNING_TIME;
	public int maxExecutionTime = DEFAULT_MAX_EXECUTION_TIME;

	Map<String, WorkingMemoryAddress> goalAddresses = new HashMap<String, WorkingMemoryAddress>();
	Set<WorkingMemoryAddress> beliefWmas = Collections
			.synchronizedSet(new HashSet<WorkingMemoryAddress>());
	private String robotID = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.generators.AbstractMotiveGenerator#start()
	 */
	@Override
	protected void start() {
		super.start();
		registerIceServer(ExternalGoalServer.class, new _ExternalGoalServerTie(
				this));

		addChangeFilter(
				ChangeFilterFactory
						.createGlobalTypeFilter(GroundedBelief.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange wmc)
							throws CASTException {
						switch (wmc.operation) {
						case ADD:
							beliefWmas.add(wmc.address);
							break;
						case DELETE:
							beliefWmas.remove(wmc.address);
							break;
						default:
							break;
						}

					}
				});

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		for (Entry<String, String> c : config.entrySet()) {
			if (c.getKey().equals("--maxPlanningTime"))
				maxPlanningTime = Integer.valueOf(c.getValue());
			else if (c.getKey().equals("--maxExecutionTime"))
				maxExecutionTime = Integer.valueOf(c.getValue());
			else if (c.getKey().equals("--costs"))
				constantCosts = Float.valueOf(c.getValue());
			else if (c.getKey().equals("--gain"))
				constantGain = Double.valueOf(c.getValue());

		}
	}

	@Override
	public MotiveStatus submitGoal(String goalString, float importance,
			Current __current) {
		try {

			log("received command to submit new goal " + goalString
					+ " with importance " + importance);
			WMEventQueue queue = submitGoalToWM(goalString, importance);
			return waitForCompletion(queue);

		} catch (CASTException e) {
			getLogger().error(e);
		} catch (InterruptedException e) {
			getLogger().error(e);
		}
		return MotiveStatus.IMPOSSIBLE;
	}

	/**
	 * @param queue
	 * @return
	 * @throws InterruptedException
	 * @throws SubarchitectureComponentException
	 */
	protected MotiveStatus waitForCompletion(WMEventQueue queue)
			throws InterruptedException, SubarchitectureComponentException {
		GeneralGoalMotive updatedGgm = null;
		WorkingMemoryOperation op = WorkingMemoryOperation.WILDCARD;
		MotiveStatus status = MotiveStatus.WILDCARD;
		// wait until that motive has been removed or flagged completed and
		// update in other cases
		while (op != WorkingMemoryOperation.DELETE
				&& status != MotiveStatus.COMPLETED) {
			try {
				WorkingMemoryChange event = queue.take();
				op = event.operation;
				if (op != WorkingMemoryOperation.DELETE) {
					updatedGgm = getMemoryEntry(event.address,
							GeneralGoalMotive.class);
					status = updatedGgm.status;
					println("current status of motive "
							+ updatedGgm.goal.goalString + " after op " + op
							+ " is " + status);
				}
			} catch (CASTException e) {
				getLogger().warn(e);
			}
		}
		removeChangeFilter(queue);
		if (updatedGgm != null) {
			println("final status of motive " + updatedGgm.goal.goalString
					+ ": " + updatedGgm.status);

			return updatedGgm.status;
		} else {
			return MotiveStatus.IMPOSSIBLE;
		}

	}

	/**
	 * @param goalString
	 * @param importance
	 * @param queue
	 * @return
	 * @throws CASTException
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected WMEventQueue submitGoalToWM(String goalString, float importance)
			throws CASTException, AlreadyExistsOnWMException,
			DoesNotExistOnWMException, UnknownSubarchitectureException {
		WMEventQueue queue = new WMEventQueue();
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
				getSubarchitectureID());
		GeneralGoalMotive ggm = MotiveFactory.createGeneralGoalMotive(wma);
		ggm.thisEntry = wma;
		goalAddresses.put(goalString, wma);

		ggm.goal = new Goal(importance, -1, substituteRobotBelief(goalString),
				false);
		ggm.informationGain = constantGain;
		ggm.costs = constantCosts;
		addChangeFilter(ChangeFilterFactory.createAddressFilter(wma), queue);
		addToWorkingMemory(wma, ggm);
		println("motive has been created and submitted to WM with address "
				+ CASTUtils.toString(wma));
		return queue;
	}

	protected String getRobotBeliefID() throws CASTException {
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

	protected String substituteRobotBelief(String goalString)
			throws CASTException {
		if (goalString.contains(ROBOT_TAG)) {
			String id = getRobotBeliefID();
			return goalString.replace(ROBOT_TAG, id);
		} else {
			return goalString;
		}
	}

	@Override
	public void config(Map<String, String> cfg, Current __current) {
		this.configure(cfg);
	}

}
