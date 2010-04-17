/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;
import castutils.facades.SpatialFacade;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author marc
 *
 */
public class PlaceContainmentAgentTransferFunction extends DependentDiscreteTransferFunction<PlaceContainmentAgentProperty, PerceptBelief> {

	public PlaceContainmentAgentTransferFunction(ManagedComponent component, WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger.getLogger(PlaceContainmentAgentTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(PlaceContainmentAgentProperty from) throws BeliefException, InterruptedException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put("AgentId", FeatureValueBuilder.createNewIntegerValue((int) from.agentID));
		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				((IntegerValue) from.mapValue).value));
		result.put(FeatPlaceId.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));
		return result;
	}


}
