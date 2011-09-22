package eu.cogx.goals.george;

import java.util.List;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import VisionData.VisualObject;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectMotiveGenerator extends
		AbstractBeliefMotiveGenerator<LearnObjectFeatureMotive, GroundedBelief> {

	private static final String VO_TYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final int MAX_EXECUTION_TIME = 30;

	private static final int MAX_PLANNING_TIME = 30;

	// TODO share this correctly with other components
	public static final String COLOUR_KEY = "color";
	public static final String COLOUR_LEARNT_KEY = COLOUR_KEY + "-learned";

	private boolean m_colourEnabled = true;
	private boolean m_shapeEnabled = true;
	private boolean m_identityEnabled = true;

	public VisualObjectMotiveGenerator() {
		super(VO_TYPE, LearnObjectFeatureMotive.class, GroundedBelief.class);
	}

	@Override
	protected LearnObjectFeatureMotive checkForUpdate(GroundedBelief _newEntry,
			LearnObjectFeatureMotive _motive) {
		if (_motive.feature == null) {
			getLogger().warn("LearnObjectFeatureMotive.feature is null",
					getLogAdditions());
		} else {
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, _newEntry);

			// if this is a colour motive and colour has been learnt for the
			// object then stop
			if (_motive.feature.equals(COLOUR_KEY) && colourLearnt(belief)) {
				return null;
			} else {
				return _motive;
			}
		}
		return _motive;

	}

	@Override
	protected LearnObjectFeatureMotive checkForAddition(
			WorkingMemoryAddress addr, GroundedBelief newEntry) {
		throw new RuntimeException(
				"The single motive version should not be called directly for this generator");
	}

	@Override
	protected void checkForAdditions(WorkingMemoryAddress addr,
			GroundedBelief newEntry, List<LearnObjectFeatureMotive> newAdditions) {
		if (m_colourEnabled) {
			LearnObjectFeatureMotive motive = generateLearnColourMotive(addr,
					newEntry);
			if (motive != null) {
				newAdditions.add(motive);
			}
		}

	}

	private LearnObjectFeatureMotive generateLearnColourMotive(
			WorkingMemoryAddress _wma, GroundedBelief _newEntry) {
		assert (_newEntry.type.equals(VO_TYPE));

		log("checkForAddition(): check belief " + _newEntry.id
				+ " for addition");

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, _newEntry);

		LearnObjectFeatureMotive result = null;

		if (!colourLearnt(belief)) {
			log("ProtoObject belief is not linked to VisualObject, so generating motive.");
			result = newLearnObjectFeatureMotive(_wma);
			result.goal = new Goal(100f, beliefPredicateGoal(COLOUR_LEARNT_KEY,
					belief), false);
			result.feature = COLOUR_KEY;
			log("goal is " + result.goal.goalString + " with inf-gain "
					+ result.informationGain);
		}
		return result;
	}

	private LearnObjectFeatureMotive newLearnObjectFeatureMotive(
			WorkingMemoryAddress _refEntry) {
		LearnObjectFeatureMotive result = new LearnObjectFeatureMotive();
		result.created = getCASTTime();
		result.updated = result.created;
		result.correspondingUnion = "";
		result.maxExecutionTime = MAX_EXECUTION_TIME;
		result.maxPlanningTime = MAX_PLANNING_TIME;
		result.priority = MotivePriority.UNSURFACE;
		result.referenceEntry = _refEntry;
		result.status = MotiveStatus.UNSURFACED;
		return result;
	}

	private boolean colourLearnt(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief) {
		return belief.getContent().containsKey(COLOUR_LEARNT_KEY);
	}

	private static String beliefPredicateGoal(String _predicate,
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> _belief) {
		StringBuilder sb = new StringBuilder("(");
		sb.append(_predicate);
		sb.append(" '");
		sb.append(_belief.getId());
		sb.append("')");
		return sb.toString();
	}
}
