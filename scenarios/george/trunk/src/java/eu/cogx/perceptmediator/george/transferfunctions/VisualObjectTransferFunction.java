package eu.cogx.perceptmediator.george.transferfunctions;

/**
 * 
 */

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PrivateBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc, nah
 * 
 */
public class VisualObjectTransferFunction extends
		SimpleDiscreteTransferFunction<VisualObject, PrivateBelief> {

	public static final String PRESENCE_VISIBLE = "visible";
	public static final String PRESENCE_UNKNOWN = "unknown";
	public static final String PRESENCE_REMOVED = "removed";
	public static final String PRESENCE_WAS_VISIBLE = "was-visible";

	public static final String PRESENCE_KEY = "presence";

	static Logger logger = Logger.getLogger(VisualObjectTransferFunction.class);

	public VisualObjectTransferFunction(ManagedComponent component) {
		super(component, logger, PrivateBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from)
			throws InterruptedException, BeliefException {
		Map<String, Formula> result = new HashMap<String, Formula>();

		result.put("salience", DoubleFormula.create(from.salience)
				.getAsFormula());

		// The status of the VO: unknow, visible, was_visible, removed
		switch (from.presence) {
		case VopVISIBLE:
			result.put(PRESENCE_KEY, PropositionFormula
					.create(PRESENCE_VISIBLE).getAsFormula());
			break;
		case VopWasVISIBLE:
			result.put(PRESENCE_KEY,
					PropositionFormula.create(PRESENCE_WAS_VISIBLE)
							.getAsFormula());
			break;
		case VopREMOVED:
			result.put(PRESENCE_KEY, PropositionFormula
					.create(PRESENCE_REMOVED).getAsFormula());
			break;
		default:
			result.put(PRESENCE_KEY, PropositionFormula
					.create(PRESENCE_UNKNOWN).getAsFormula());
			break;
		}

		// logger.info("added salience");

		fillConcept("color", result, from.colorLabels, from.colorDistrib);

		// logger.info("added color");

		fillConcept("shape", result, from.shapeLabels, from.shapeDistrib);

		return result;
	}

	private void fillConcept(String concept, Map<String, Formula> result,
			String[] labels, double[] distrib) {

		if (labels.length > 0) {

			double maxLabelProb = Double.MIN_VALUE;
			int maxIndex = 0;

			for (int i = 0; i < labels.length; i++) {

				// HACK for planner cleanliness
				if (distrib[i] > maxLabelProb) {
					maxLabelProb = distrib[i];
					maxIndex = i;
				}
				// HACK - END
			}

			result.put(concept + "-prob", DoubleFormula.create(maxLabelProb)
					.getAsFormula());

			result.put(concept, PropositionFormula.create(labels[maxIndex])
					.getAsFormula());
		}
	}
}
