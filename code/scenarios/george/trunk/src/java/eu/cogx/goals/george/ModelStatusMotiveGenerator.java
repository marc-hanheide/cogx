package eu.cogx.goals.george;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.RobotNonSituatedMotive;
import VisionData.VisualConceptModelStatus;
import autogen.Planner.Goal;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTData;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.perceptmediator.george.transferfunctions.ModelStatusTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class ModelStatusMotiveGenerator extends
		AbstractBeliefMotiveGenerator<RobotNonSituatedMotive, MergedBelief> {

	private static final String MS_TYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils
					.typeName(VisualConceptModelStatus.class));

	private static final int MAX_EXECUTION_TIME = 30;

	private static final int MAX_PLANNING_TIME = 30;

	private static final double GAIN_THRESHOLD = 0.1;

	// keeps track of the current most promising
	private final HashMap<String, String> m_conceptValues;

	public ModelStatusMotiveGenerator() {
		super(MS_TYPE, RobotNonSituatedMotive.class, MergedBelief.class);
		m_conceptValues = new HashMap<String, String>(2);
	}

	@Override
	protected RobotNonSituatedMotive checkForAddition(
			WorkingMemoryAddress _wma, MergedBelief _newEntry) {

		assert (_newEntry.type.equals(MS_TYPE));

		log("checkForAddition(): check belief " + _newEntry.id
				+ " for addition");

		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

		double gain = getModelGain(belief);

		RobotNonSituatedMotive result = null;

		log("gain: " + gain);

		if (gain > GAIN_THRESHOLD) {

			String concept = getConcept(belief);
			String value = getMostPromising(belief);

			// HACK
//			if (concept.equals("shape")) {
//				getLogger().warn("ignoring shape model status",
//						getLogAdditions());
//				return null;
//			} else {
//				log("generating for: " + concept);
//			}
			// END HACK

			m_conceptValues.put(concept, value);

			result = new RobotNonSituatedMotive();
			result.created = getCASTTime();
			result.updated = result.created;
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = _wma;
			result.status = MotiveStatus.UNSURFACED;

			// (object-of-desired-color-available COLORNAME)

			String goal = createGoalString(concept, value);

			result.goal = new Goal(100f, -1, goal, false);

			log("goal is " + result.goal.goalString);
		}
		return result;

	}

	// TODO HACK sanitise between Dora and George
	private WorkingMemoryAddress m_robotBeliefAddr;

	// TODO HACK sanitise between Dora and George
	@Override
	protected WorkingMemoryAddress getRobotBeliefAddr() {
		while (m_robotBeliefAddr == null) {
			List<CASTData<MergedBelief>> groundedBeliefs = new ArrayList<CASTData<MergedBelief>>();
			try {
				getMemoryEntriesWithData(MergedBelief.class, groundedBeliefs,
						"binder", 0);
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				return null;
			}

			for (CASTData<MergedBelief> beliefEntry : groundedBeliefs) {
				if (beliefEntry.getData().type.equalsIgnoreCase("Robot")) {
					m_robotBeliefAddr = new WorkingMemoryAddress(
							beliefEntry.getID(), "binder");
					break;
				}
			}
			if (m_robotBeliefAddr == null) {
				getLogger().warn("unable to find belief '" + "Robot" + "'");
				sleepComponent(200);
			}
		}
		return m_robotBeliefAddr;

	}

	private String createGoalString(String concept, String value) {
		String predicate = "object-of-desired-" + concept + "-available";

		return VisualObjectMotiveGenerator.beliefFunctionGoal(predicate,
				getRobotBeliefAddr().id, value);

	}

	private String getMostPromising(
			CASTIndependentFormulaDistributionsBelief<MergedBelief> belief) {
		FormulaDistribution fd;
		fd = belief.getContent()
				.get(ModelStatusTransferFunction.MOST_PROMISING);
		String value = fd.getDistribution().getMostLikely().getProposition();
		return value;
	}

	private String getConcept(
			CASTIndependentFormulaDistributionsBelief<MergedBelief> belief) {
		FormulaDistribution fd;
		fd = belief.getContent().get(ModelStatusTransferFunction.CONCEPT_ID);
		String concept = fd.getDistribution().getMostLikely().getProposition();
		return concept;
	}

	private double getModelGain(
			CASTIndependentFormulaDistributionsBelief<MergedBelief> belief) {
		FormulaDistribution fd = belief.getContent().get(
				ModelStatusTransferFunction.GAIN);
		double gain = fd.getDistribution().getMostLikely().getDouble();
		return gain;
	}

	@Override
	protected RobotNonSituatedMotive checkForUpdate(MergedBelief _newEntry,
			RobotNonSituatedMotive _motive) {

		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

		String concept = getConcept(belief);
		String value = getMostPromising(belief);
		double gain = getModelGain(belief);

		RobotNonSituatedMotive result = null;

		if (gain > GAIN_THRESHOLD) {

			result = _motive;

			String previousValue = m_conceptValues.get(concept);

			if (previousValue == null) {
				getLogger().warn("I'm not sure this should be happening",
						getLogAdditions());
				return null;
			}

			// gain has increased enough so we can do something
			if (!previousValue.equals(value)) {
				log("changed model status goal from " + previousValue + " to "
						+ value);
				m_conceptValues.put(concept, value);
				result.goal.goalString = createGoalString(concept, value);
				result.status = MotiveStatus.UNSURFACED;
			}

		}

		return result;
	}
}
