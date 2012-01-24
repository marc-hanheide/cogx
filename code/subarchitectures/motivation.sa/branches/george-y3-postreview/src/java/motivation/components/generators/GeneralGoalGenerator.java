/**
 * 
 */
package motivation.components.generators;

import java.util.Map;
import java.util.Map.Entry;

import autogen.Planner.Goal;

import motivation.factories.MotiveFactory;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.Motive;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class GeneralGoalGenerator extends AbstractMotiveGenerator {
	/** add a really low gain to homing */
	private static final double DEFAULT_CONSTANT_GAIN = 1.0;
	private static final double DEFAULT_CONSTANT_COSTS = 1.0;
	private static final String DEFAULT_GOAL = "(forall (?p - place) (= (placestatus ?p) trueplace))";
	// private static final String DEFAULT_GOAL =
	// "(and (forall (?p - place) (= (explored ?p) true)) (forall (?r - room) (kval '@R' (areaclass ?r))))";
	// give planning at maximum 15 seconds
	private static final int DEFAULT_MAX_PLANNING_TIME = 15;
	// give the max at max 60 minutes to reach goal
	private static final int DEFAULT_MAX_EXECUTION_TIME = 60 * 60;
	private boolean initialized;

	private String goalString = DEFAULT_GOAL;
	private double constantGain = DEFAULT_CONSTANT_GAIN;
	private double constantCosts = DEFAULT_CONSTANT_COSTS;
	protected int maxPlanningTime = DEFAULT_MAX_PLANNING_TIME;
	protected int maxExecutionTime = DEFAULT_MAX_EXECUTION_TIME;

	/**
	 * 
	 */
	public GeneralGoalGenerator() {
		initialized = false;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.generators.AbstractMotiveGenerator#checkMotive(
	 * motivation.slice.Motive)
	 */
	@Override
	protected boolean checkMotive(Motive motive) throws CASTException {
		if (!initialized) {
			initialized = true;
			return true;

		}
		return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.generators.AbstractMotiveGenerator#start()
	 */
	@Override
	protected void start() {
		super.start();
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				Ice.Object.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						debug(CASTUtils.toString(_wmc));
						// create a new motive from this node...
						if (!initialized) { // if home base is not set yet
							GeneralGoalMotive newMotive = MotiveFactory
									.createGeneralGoalMotive(_wmc.address);
							newMotive.goal = new Goal(-1, goalString, false);
							newMotive.informationGain = constantGain;
							newMotive.costs = (float) constantCosts;

							newMotive.maxPlanningTime = maxPlanningTime;
							// wait at most 5 minutes
							newMotive.maxExecutionTime = maxExecutionTime;

							scheduleCheckMotive(newMotive);
						} else { // if we have this one already
							try {
								removeChangeFilter(this);
							} catch (SubarchitectureComponentException e) {
								logException(e);
							}
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
			if (c.getKey().equals("--goal"))
				goalString = c.getValue();
			else if (c.getKey().equals("--maxPlanningTime"))
				maxPlanningTime = Integer.valueOf(c.getValue());
			else if (c.getKey().equals("--maxExecutionTime"))
				maxExecutionTime = Integer.valueOf(c.getValue());
			else if (c.getKey().equals("--costs"))
				constantCosts = Double.valueOf(c.getValue());
			else if (c.getKey().equals("--gain"))
				constantGain = Double.valueOf(c.getValue());

		}
	}

}
