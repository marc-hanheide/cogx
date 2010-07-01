/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.AgentMatchingFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author marc
 * 
 */
public class PlaceContainmentAgentTransferFunction extends
		DependentDiscreteTransferFunction<PlaceContainmentAgentProperty> {

	public PlaceContainmentAgentTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(PlaceContainmentAgentTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	public PerceptBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from) {
		PerceptBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from)
			throws BeliefException, InterruptedException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

		WorkingMemoryAddress wmaAgent = getReferredBelief(new AgentMatchingFunction(
				from.agentID));
		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				((IntegerValue) from.mapValue).value));

		result.put("val0", WMPointer.create(wmaAgent).getAsFormula());
		result.put("val1", WMPointer.create(wmaPlace).getAsFormula());
		result.put("is-in", BoolFormula.create(true).getAsFormula());

		return result;
	}

}
