/**
 * 
 */
package eu.cogx.perceptmediator;

import java.util.HashMap;
import java.util.Map;

import VisionData.VisualConceptModelStatus;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class ModelStatusTransferFunction
		extends
		SimpleDiscreteTransferFunction<VisualConceptModelStatus, GroundedBelief> {

	public static final String GAIN = "gain";
	public static final String MOST_PROMISING = "most-promising";
	public static final String CONCEPT_ID = "concept";
	private static final String ASKED_FOR = "asked-for";

	public ModelStatusTransferFunction(ManagedComponent component) {
		super(component, component.getLogger(), GroundedBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualConceptModelStatus from)
			throws InterruptedException, BeliefException {

		Map<String, Formula> result = new HashMap<String, Formula>();
		result.put(CONCEPT_ID, PropositionFormula.create(from.concept)
				.getAsFormula());
		double maxGain = 0.0;
		String maxLabel = null;
		boolean maxAsked4 = false;
		for (int i = 0; i < from.gains.length; i++) {
			// if we have asked for it already we can skip it.
			
//				if (from.askedFor[i])
//					continue;
			if (from.gains[i] >= maxGain) {
				maxGain = from.gains[i];
				maxLabel = from.labels[i];
				if (from.askedFor.length>=i+1)
					maxAsked4 = from.askedFor[i];
			}
		}
		if (maxLabel == null || maxAsked4 == true)
			return null;
		result.put(MOST_PROMISING, PropositionFormula.create(maxLabel)
				.getAsFormula());
		result.put(GAIN, DoubleFormula.create(maxGain).getAsFormula());
		result.put(ASKED_FOR, BoolFormula.create(maxAsked4).getAsFormula());
		return result;
	}

}
