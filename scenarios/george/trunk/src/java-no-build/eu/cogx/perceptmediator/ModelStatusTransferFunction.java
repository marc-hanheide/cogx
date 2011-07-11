/**
 * 
 */
package eu.cogx.perceptmediator;

import java.util.HashMap;
import java.util.Map;

import VisionData.VisualConceptModelStatus;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.IntFormula;
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
	private static final String CONCEPT_ID = "concept";

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
		for (int i = 0; i < from.gains.length; i++) {
			if (from.gains[i] > maxGain) {
				maxGain = from.gains[i];
				maxLabel = from.labels[i];
			}
		}
		if (maxLabel == null)
			return null;
		result.put(MOST_PROMISING, PropositionFormula.create(maxLabel)
				.getAsFormula());
		result.put(GAIN, DoubleFormula.create(maxGain).getAsFormula());

		return result;
	}

}
