package eu.cogx.goals.george;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import VisionData.ProtoObject;
import autogen.Planner.Goal;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.perceptmediator.transferfunctions.ProtoObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class ProtoObjectMotiveGenerator extends
		AbstractBeliefMotiveGenerator<AnalyzeProtoObjectMotive, MergedBelief> {

	private static final String PO_TYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ProtoObject.class));

	private static final int MAX_EXECUTION_TIME = 30;

	private static final int MAX_PLANNING_TIME = 30;

	public ProtoObjectMotiveGenerator() {
		super(PO_TYPE, AnalyzeProtoObjectMotive.class, MergedBelief.class);
	}

	@Override
	protected AnalyzeProtoObjectMotive checkForAddition(
			WorkingMemoryAddress _wma, MergedBelief _newEntry) {

		assert (_newEntry.type.equals(PO_TYPE));

		log("checkForAddition(): check belief " + _newEntry.id
				+ " for addition");

		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

		AnalyzeProtoObjectMotive result = null;

		if (!belief.getContent().containsKey(
				ProtoObjectTransferFunction.VISUAL_OBJECT_LINK)) {

			log("ProtoObject belief is not linked to VisualObject, so generating motive.");

			result = new AnalyzeProtoObjectMotive();

			result.created = getCASTTime();
			result.updated = result.created;
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = _wma;
			result.status = MotiveStatus.UNSURFACED;

			// (exists (?v - VisualObject) (= (po-is-associated-with ?p) ?v))

			result.goal = new Goal(-1, -1,
					"(exists (?v - VisualObject) (= (po-is-associated-with '"
							+ belief.getId() + "') ?v))", false);

			log("goal is " + result.goal.goalString + " with inf-gain "
					+ result.informationGain);

		}
		return result;

	}

	@Override
	protected AnalyzeProtoObjectMotive checkForUpdate(MergedBelief _newEntry,
			AnalyzeProtoObjectMotive _motive) {

		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

		if (!belief.getContent().containsKey(
				ProtoObjectTransferFunction.VISUAL_OBJECT_LINK)) {
			log("ProtoObject belief is still not linked to VisualObject, so leaving motive unchanged.");
			return _motive;
		} else {
			log("ProtoObject belief is now linked to VisualObject, so marking as complete.");
			_motive.status = MotiveStatus.COMPLETED;
			return _motive;
		}
	}
}
