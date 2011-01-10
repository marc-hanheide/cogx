package eu.cogx.goals.george;

import java.util.Map;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.RobotNonSituatedMotive;
import VisionData.VisualConceptModelStatus;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.ModelStatusTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class ModelStatusGoalGenerator extends
		AbstractBeliefMotiveGenerator<RobotNonSituatedMotive, GroundedBelief> {

	private static final String OBJECTTYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils
					.typeName(VisualConceptModelStatus.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 10;

	public ModelStatusGoalGenerator() {
		super(OBJECTTYPE, RobotNonSituatedMotive.class, GroundedBelief.class);
	}

	@Override
	protected RobotNonSituatedMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(OBJECTTYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		RobotNonSituatedMotive result = new RobotNonSituatedMotive();
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
	protected RobotNonSituatedMotive checkForUpdate(GroundedBelief newEntry,
			RobotNonSituatedMotive motive) {
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
				ModelStatusTransferFunction.GAIN).getDistribution()
				.getMostLikely().getDouble();
		String concept=belief.getContent().get(
				ModelStatusTransferFunction.CONCEPT_ID).getDistribution()
				.getMostLikely().getProposition();
		String value=belief.getContent().get(
				ModelStatusTransferFunction.MOST_PROMISING).getDistribution()
				.getMostLikely().getProposition();
		//		((RobotNonSituatedMotive) motive).text = "show me something "
//				+ belief.getContent().get(
//						ModelStatusTransferFunction.MOST_PROMISING)
//						.getDistribution().getMostLikely().getProposition();

		motive.goal = new Goal(-1, "(object-of-desired-"+concept+"-available "+concept+"-name_"+value+")", false);
		//motive.goal = new Goal(-1, "(object-of-desired-"+concept+"-available)", false);
		log("updated goal to " + motive.goal.goalString);
	}

}
