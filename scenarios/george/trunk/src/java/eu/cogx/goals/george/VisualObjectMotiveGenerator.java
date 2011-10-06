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
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.george.transferfunctions.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectMotiveGenerator extends
		AbstractBeliefMotiveGenerator<LearnObjectFeatureMotive, GroundedBelief> {

	private static final String VO_TYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(VisualObject.class));

	private static final int MAX_EXECUTION_TIME = 30;

	private static final int MAX_PLANNING_TIME = 30;

	// TODO share these correctly with other components
	public static final String LEARNT_POSTFIX_KEY = "-learned";

	public static final String COLOUR_KEY = "color";
	public static final String COLOUR_LEARNT_KEY = COLOUR_KEY
			+ LEARNT_POSTFIX_KEY;
	public static final String SHAPE_KEY = "shape";
	public static final String SHAPE_LEARNT_KEY = SHAPE_KEY
			+ LEARNT_POSTFIX_KEY;
	public static final String IDENTITY_KEY = "identity";
	public static final String IDENTITY_LEARNT_KEY = IDENTITY_KEY
			+ LEARNT_POSTFIX_KEY;

	// TODO Add config options to set these
	private boolean m_colourEnabled = true;
	private boolean m_shapeEnabled = false;
	private boolean m_identityEnabled = false;

	public VisualObjectMotiveGenerator() {
		super(VO_TYPE, LearnObjectFeatureMotive.class, GroundedBelief.class);
	}

	private boolean motiveFeatureLearnt(String _featureKey,
			String _featureLearntPredicate, LearnObjectFeatureMotive _motive,
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> _belief) {
		return _motive.feature.equals(_featureKey)
				&& _belief.getContent().containsKey(_featureLearntPredicate);
	}

	@Override
	protected LearnObjectFeatureMotive checkForUpdate(GroundedBelief _newEntry,
			LearnObjectFeatureMotive _motive) {
		if (_motive.feature == null) {
			getLogger().warn("LearnObjectFeatureMotive.feature is null",
					getLogAdditions());
			return null;
		} else {

			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, _newEntry);

			// only generate things if the VO is actually visible

			if (!visualObjectIsVisible(belief)) {
				return null;
			} else if ((m_colourEnabled && motiveFeatureLearnt(COLOUR_KEY,
					COLOUR_LEARNT_KEY, _motive, belief))
					|| (m_shapeEnabled && motiveFeatureLearnt(SHAPE_KEY,
							SHAPE_LEARNT_KEY, _motive, belief))
					|| (m_identityEnabled && motiveFeatureLearnt(IDENTITY_KEY,
							IDENTITY_LEARNT_KEY, _motive, belief))) {
				return null;
			} else {
				return _motive;
			}
		}

	}

	@Override
	protected LearnObjectFeatureMotive checkForAddition(
			WorkingMemoryAddress addr, GroundedBelief newEntry) {
		throw new RuntimeException(
				"The single motive version should not be called directly for this generator");
	}

	protected boolean visualObjectIsVisible(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> _belief) {
		FormulaDistribution fd = _belief.getContent().get(
				VisualObjectTransferFunction.PRESENCE_KEY);
		String presenceValue = fd.getDistribution().firstValue()
				.getProposition();

		log("visual object presence: " + presenceValue);
		return presenceValue
				.equals(VisualObjectTransferFunction.PRESENCE_VISIBLE);
	}

	@Override
	protected void checkForAdditions(WorkingMemoryAddress addr,
			GroundedBelief _newEntry,
			List<LearnObjectFeatureMotive> newAdditions) {

		assert (_newEntry.type.equals(VO_TYPE));

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, _newEntry);

		// only generate things if the VO is actually visible

		if (visualObjectIsVisible(belief)) {

			if (m_colourEnabled) {
				LearnObjectFeatureMotive motive = generateLearnFeatureMotive(
						COLOUR_KEY, COLOUR_LEARNT_KEY, addr, belief);
				if (motive != null) {
					newAdditions.add(motive);
				}
			}

			if (m_shapeEnabled) {
				LearnObjectFeatureMotive motive = generateLearnFeatureMotive(
						SHAPE_KEY, SHAPE_LEARNT_KEY, addr, belief);
				if (motive != null) {
					newAdditions.add(motive);
				}
			}

			if (m_identityEnabled) {
				LearnObjectFeatureMotive motive = generateLearnFeatureMotive(
						IDENTITY_KEY, IDENTITY_LEARNT_KEY, addr, belief);
				if (motive != null) {
					newAdditions.add(motive);
				}
			}

		}
	}

	private LearnObjectFeatureMotive generateLearnFeatureMotive(
			String _featureKey, String _featureLearntPredicate,
			WorkingMemoryAddress _wma,
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> _belief) {

		LearnObjectFeatureMotive result = null;

		if (!_belief.getContent().containsKey(_featureLearntPredicate)) {
			log("ProtoObject belief is not linked to VisualObject, so generating motive.");
			result = newLearnObjectFeatureMotive(_wma);
			result.goal = new Goal(100f, beliefPredicateGoal(
					_featureLearntPredicate, _belief), false);
			result.feature = _featureKey;
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
