/**
 * 
 */
package motivation.components.generators;

import motivation.factories.MotiveFactory;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
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
public class ExploreAndCategorizeGenerator extends AbstractMotiveGenerator {
	/** add a really low gain to homing */
	private static final double CONSTANT_GAIN = 1.0;
	private static final String GOAL = "(and (forall (?p - place) (= (explored ?p) true)) (forall (?r - room) (kval '@R' (areaclass ?r))))";
	protected static final float CONSTANT_COST = (float) 10.0;
	private boolean initialized;

	/**
	 * 
	 */
	public ExploreAndCategorizeGenerator() {
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
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);
			if (source.status == PlaceStatus.TRUEPLACE) {
				log("  we have a first true place and take it as the home base");
				initialized = true;
				write(motive);
				return true;
			}
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
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				if (!initialized) { // if home base is not set yet
					GeneralGoalMotive newMotive = MotiveFactory
							.createGeneralGoalMotive(_wmc.address);
					newMotive.internalGoal = GOAL;
					newMotive.informationGain = CONSTANT_GAIN;
					newMotive.costs = CONSTANT_COST;

					newMotive.maxPlanningTime = 5;
					// wait at most 5 minutes
					newMotive.maxExecutionTime = 5 * 60;

					scheduleCheckMotive(newMotive);
				} else { // if we have this one already
					try {
						removeChangeFilter(this);
					} catch (SubarchitectureComponentException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		});
	}

}
