package eu.cogx.perceptmediator.george.transferfunctions;

/**
 * 
 */

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import VisionData.VisualObjectPresence;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleTransferFunction;

/**
 * @author marc, nah
 * 
 */
public class VisualObjectTransferFunction extends
		SimpleTransferFunction<VisualObject, GroundedBelief> {

	public static final String PRESENCE_VISIBLE = "visible";
	public static final String PRESENCE_UNKNOWN = "unknown_presence";
	public static final String PRESENCE_REMOVED = "removed";
	public static final String PRESENCE_WAS_VISIBLE = "was-visible";

	public static final String PRESENCE_KEY = "presence";

	static Logger logger = Logger.getLogger(VisualObjectTransferFunction.class);

	public VisualObjectTransferFunction(ManagedComponent component) {
		super(component, logger, GroundedBelief.class);
	}

	@Override
	protected Map<String, FormulaProbPair> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from)
			throws InterruptedException, BeliefException {
		Map<String, FormulaProbPair> result = new HashMap<String, FormulaProbPair>();

		FormulaProbPair fpair = new FormulaProbPair();
		FloatFormula ff = new FloatFormula();
		ff.val = (float) from.salience;
		fpair.val = ff;
		fpair.prob = 1;

		result.put("salience", fpair);

		// The status of the VO: unknow, visible, was_visible, removed
		fpair = new FormulaProbPair();
		ElementaryFormula ef = new ElementaryFormula();
		switch (from.presence) {
		case VopVISIBLE:
			ef.prop = PRESENCE_VISIBLE;
			break;
		case VopWasVISIBLE:
			ef.prop = PRESENCE_WAS_VISIBLE;
			break;
		case VopREMOVED:
			ef.prop = PRESENCE_REMOVED;
			break;
		default:
			ef.prop = PRESENCE_UNKNOWN;
			break;
		}
		fpair.val = ef;
		fpair.prob = 1;

		result.put(PRESENCE_KEY, fpair);

		// non-visible objects can no longer be talked about
		if (from.presence != VisualObjectPresence.VopVISIBLE) {
			result.put("is-potential-object-in-question", new FormulaProbPair(
					BoolFormula.create(false).get(), 1f));
		}

		// logger.info("added salience");

		fillConcept("color", result, from.colorLabels, from.colorDistrib);

		// logger.info("added color");

		fillConcept("shape", result, from.shapeLabels, from.shapeDistrib);

		if (from.identLabels.length == 0) {
			// HACK to allow questions to be answered with no recognition result
			fillConcept("objecttype", result, new String[] { "box" },
					new double[] { 0.01 });
		} else {
			fillConcept("objecttype", result, from.identLabels,
					from.identDistrib);
		}
		return result;
	}

	private void fillConcept(String concept,
			Map<String, FormulaProbPair> result, String[] labels,
			double[] distrib) {

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

			FormulaProbPair fpair = new FormulaProbPair();
			FloatFormula ff = new FloatFormula();
			ff.val = (float) maxLabelProb;
			fpair.val = ff;
			fpair.prob = 1;

			result.put(concept + "-prob", fpair);

			fpair = new FormulaProbPair();
			ElementaryFormula ef = new ElementaryFormula();
			ef.prop = labels[maxIndex];
			fpair.val = ef;
			fpair.prob = (float) maxLabelProb;

			result.put(concept, fpair);
		}
	}
}
