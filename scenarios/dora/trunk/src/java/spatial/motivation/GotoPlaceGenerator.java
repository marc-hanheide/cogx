package spatial.motivation;

import java.util.Map;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class GotoPlaceGenerator extends
		AbstractBeliefMotiveGenerator<PatrolMotive, GroundedBelief>  {
	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Place.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 30;

	/**
	 * the maximum costs to drop we assign if information gain is really high
	 * (~1) in seconds
	 */
	private static final double MAX_COSTS_TO_DROP = 50;
	private int targetPlaceID = 0;
	private int deadLine = -1;

	public GotoPlaceGenerator() {
		super(PLACETYPE, PatrolMotive.class, GroundedBelief.class);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		reactivateCompleteMotives(true);
		monitorMotivesForDeletion(true);
		String intStr = config.get("--placeID");
		if (intStr != null)
			targetPlaceID = Integer.parseInt(intStr);
		intStr = config.get("--deadline");
		if (intStr != null)
			deadLine = Integer.parseInt(intStr);
	}

	@Override
	protected PatrolMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(PLACETYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		FormulaDistribution pIdFD = belief.getContent().get(
				PlaceTransferFunction.PLACE_ID_ID);
		int placeID = pIdFD.getDistribution().getMostLikely().getInteger();
		if (placeID != targetPlaceID)
			return null;
		// get the most likely status
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		log("checkForAddition(): placestatus="
				+ belief.getContent().get("placestatus").getDistribution()
						.getMostLikely().getProposition());

		// register on any changes to this motive, so we can reactivate it
		
		if (isExplored) {
			log("place is explored, so it is a goal to goto");
			PatrolMotive result = new PatrolMotive();
			result.created = getCASTTime();
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = adr;
			result.status = MotiveStatus.UNSURFACED;
			result.lastVisisted = result.created;
			fillValues(belief, result);
			return result;
		}
		return null;
	}


	@Override
	protected PatrolMotive checkForUpdate(GroundedBelief newEntry,
			PatrolMotive motive) {
		assert (newEntry.type.equals(PLACETYPE));
		log("check goal " + CASTUtils.toString(motive.thisEntry)
				+ " for update");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		// if that is a place holder
		if (isExplored) {
			log("place is explored, so it is a goal to patrol");
			motive.lastVisisted = getCASTTime();
			fillValues(belief, motive);
			return motive;
		} else {
			return null;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.generators.AbstractBeliefMotiveGenerator#start()
	 */
	@Override
	protected void start() {
		super.start();
		// try {
		// SpatialFacade.get(this).registerPlaceChangedCallback(
		// new SpatialFacade.PlaceChangedHandler() {
		// @Override
		// public synchronized void update(Place p) {
		// log("explicitly scheduling all motives to be checked due to place change. new place is "
		// + p.id);
		// recheckAllMotives();
		// }
		// });
		// } catch (CASTException e1) {
		// println("exception when registering placeChangedCallbacks");
		// e1.printStackTrace();
		// }

	}

	private void fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			PatrolMotive motive) {
		motive.updated = getCASTTime();
		motive.informationGain = 1.0;
		motive.goal = new Goal(computeImportance(motive), deadLine,
				"(= (is-in '" + this.getRobotBeliefAddr().id + "') '"
						+ belief.getId() + "')", false);
		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

	}

	float computeImportance(PatrolMotive m) {
		if (m.informationGain < 0)
			return -1.0f;
		else
			return (float) (m.informationGain * MAX_COSTS_TO_DROP);
	}



}
