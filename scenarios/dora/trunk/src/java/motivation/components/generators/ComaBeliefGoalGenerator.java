package motivation.components.generators;

import java.util.Map;

import motivation.slice.CategorizeRoomMotive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import SpatialData.PlaceStatus;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class ComaBeliefGoalGenerator extends
		AbstractBeliefMotiveGenerator<CategorizeRoomMotive, GroundedBelief> {

	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ComaRoom.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 10;

	/**
	 * the maximum costs to drop we assign if information gain is really high
	 * (~1) in seconds
	 */
	private static final double MAX_COSTS_TO_DROP = 5 * 60;

	public ComaBeliefGoalGenerator() {
		super(PLACETYPE, CategorizeRoomMotive.class, GroundedBelief.class);
	}

	@Override
	protected CategorizeRoomMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(PLACETYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		// get the most likely status
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		log("checkForAddition(): placestatus="
				+ belief.getContent().get("placestatus").getDistribution()
						.getMostLikely().getProposition());
		// if that is a place holder
		if (!isExplored) {
			log("place is not yet explored, so it is a goal");
			CategorizeRoomMotive result = new CategorizeRoomMotive();
			result.created = getCASTTime();
			result.correspondingUnion = "";
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = adr;
			result.status = MotiveStatus.UNSURFACED;
			fillValues(belief, result);
			return result;
		}
		return null;
	}

	@Override
	protected CategorizeRoomMotive checkForUpdate(GroundedBelief newEntry,
			CategorizeRoomMotive motive) {
		assert (newEntry.type.equals(PLACETYPE));
		log("check goal " + CASTUtils.toString(motive.thisEntry)
				+ " for update");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		// TODO: check for existing category is still missing:
		fillValues(belief, motive);
		return motive;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {

	}

	private void fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			CategorizeRoomMotive motive) {
		motive.updated = getCASTTime();
		// initially this costs are taken as -1, corresponding to an ultimate
		// goal.
		motive.costs = -1;
		motive.informationGain = 1.0;
		assert (getRobotBeliefAddr() != null);
		motive.goal = new Goal(computeImportance(motive), "(kval '"
				+ getRobotBeliefAddr() + "' (areaclass '" + belief.getId()
				+ "'))", false);
		log("updated goal to " + motive.goal.goalString);
	}

	float computeImportance(CategorizeRoomMotive m) {
		return -1.0f;
	}

}
