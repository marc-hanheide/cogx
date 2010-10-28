package motivation.components.generators;

import java.util.Map;

import motivation.slice.CategorizeRoomMotive;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class ComaBeliefGoalGenerator extends
		AbstractBeliefMotiveGenerator<CategorizeRoomMotive, GroundedBelief> {

	private static final String COMATYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ComaRoom.class));

	/**
	 * the maximum costs to drop we assign if information gain is really high
	 * (~1) in seconds
	 */
	private static final double MAX_COSTS_TO_DROP = 5 * 60;

	public ComaBeliefGoalGenerator() {
		super(COMATYPE, CategorizeRoomMotive.class, GroundedBelief.class);
	}

	@Override
	protected CategorizeRoomMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(COMATYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		if (!isCategorized(belief)) {
			log("room not yet categorized, so it is a goal");
			CategorizeRoomMotive result = new CategorizeRoomMotive();
			fillDefault(result);
			fillValues(belief, result);
			return result;
		}
		return null;
	}

	private boolean isCategorized(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected CategorizeRoomMotive checkForUpdate(GroundedBelief newEntry,
			CategorizeRoomMotive motive) {
		assert (newEntry.type.equals(COMATYPE));
		log("check goal " + CASTUtils.toString(motive.thisEntry)
				+ " for update");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		if (!isCategorized(belief)) {
			fillValues(belief, motive);
			return motive;
		}else {
			return null;
		}

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
