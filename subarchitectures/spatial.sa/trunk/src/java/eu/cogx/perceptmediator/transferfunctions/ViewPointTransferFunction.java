/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.ViewPoint;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
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
public class ViewPointTransferFunction extends SimpleDiscreteTransferFunction<ViewPoint, GroundedBelief> {

	public static final String OBJECT_PROBABILITY_ID = "probability";
	public static final String OBJECT_LABEL_ID = "label";

	public ViewPointTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(ViewPointTransferFunction.class), GroundedBelief.class);
	}

	@Override
	protected
	Map<String, Formula> getFeatureValueMapping(WorkingMemoryChange wmc, ViewPoint from) throws BeliefException {
		assert(from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

		//the object which can be found from this cone
		result.put(OBJECT_LABEL_ID, PropositionFormula.create(from.label).getAsFormula());

		//the 
		result.put(OBJECT_PROBABILITY_ID, DoubleFormula.create(from.probability).getAsFormula());
		return result;
	}


}
