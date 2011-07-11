package eu.cogx.goals.george;

import java.util.Map;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.RobotInitiativeMotive;
import VisionData.VisualObject;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public abstract class LearnConceptGenerator extends
		AbstractBeliefMotiveGenerator<RobotInitiativeMotive, GroundedBelief> {

	private static final String OBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 10;

	/**
	 * the maximum costs to drop we assign if information gain is really high
	 * (~1) in seconds
	 */
	private static final double MAX_COSTS_TO_DROP = 5 * 60;

	public LearnConceptGenerator() {
		super(OBJECTTYPE, RobotInitiativeMotive.class, GroundedBelief.class);
	}

	@Override
	protected RobotInitiativeMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(OBJECTTYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		RobotInitiativeMotive result = new RobotInitiativeMotive();
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

	@Override
	protected RobotInitiativeMotive checkForUpdate(GroundedBelief newEntry,
			RobotInitiativeMotive motive) {
		assert (newEntry.type.equals(OBJECTTYPE));
		log("check goal " + CASTUtils.toString(motive.thisEntry)
				+ " for update");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
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
			Motive motive) {
		motive.updated = getCASTTime();
		// initially this costs are taken as -1, corresponding to an ultimate
		// goal.
		motive.costs = -1;
		motive.informationGain = belief.getContent().get(
				this.getGainPropertyName()).getDistribution().getMostLikely()
				.getDouble();

		motive.goal = new Goal(computeImportance(motive),
				"("+getConceptName().toLowerCase()+"-learned '" + belief.getId() + "')", false);
		log("updated goal to " + motive.goal.goalString);
	}

	protected abstract String getGainPropertyName();

	protected abstract String getConceptName();

	float computeImportance(Motive m) {
//		if (m.informationGain < 0)
//			return -1.0f;
//		else
//			return (float) (m.informationGain * MAX_COSTS_TO_DROP);
		return -1.0f;
	}

}
