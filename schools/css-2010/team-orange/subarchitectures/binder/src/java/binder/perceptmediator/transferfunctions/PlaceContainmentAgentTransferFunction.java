/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement0;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement1;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.AgentMatchingFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

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
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from)
			throws BeliefException, InterruptedException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaAgent = getReferredBelief(new AgentMatchingFunction(from.agentID));
		result.put("is-in", new BooleanValue(true));
		result.put(RelationElement0.value, FeatureValueBuilder
				.createNewStringValue(wmaAgent.id));

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				((IntegerValue) from.mapValue).value));
		result.put(RelationElement1.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));
		return result;
	}

	@Override
	public PerceptBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, PlaceContainmentAgentProperty from) {
		PerceptBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

}
