package motivation.components.generators;

import autogen.Planner.Goal;
import motivation.slice.ExploreMotive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class PlaceBeliefGoalGenerator extends
		AbstractBeliefMotiveGenerator<ExploreMotive, GroundedBelief> {

	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Place.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 10;

	public PlaceBeliefGoalGenerator() {
		super(PLACETYPE, ExploreMotive.class, GroundedBelief.class);
	}

	@Override
	protected ExploreMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(PLACETYPE));
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		// get the most likely status
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		// if that is a place holder
		if (!isExplored) {
			ExploreMotive result = new ExploreMotive();
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
	protected ExploreMotive checkForUpdate(GroundedBelief newEntry,
			ExploreMotive motive) {
		assert (newEntry.type.equals(PLACETYPE));
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		// if that is a place holder
		if (!isExplored) {
			fillValues(belief, motive);
			return motive;
		} else {
			return null;
		}
	}

	private void fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			ExploreMotive motive) {
		motive.updated = getCASTTime();
		// initially this costs are taken as -1, corresponding to an ultimate
		// goal.
		motive.costs = -1;
		motive.goal = new Goal(-1.0f, "(= (placestatus '" + belief.getId()
				+ "') trueplace)", false);
	}

}
