package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.GatewayPlaceProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

public class GatewayTransferFunction
		extends
		DependentDiscreteTransferFunction<GatewayPlaceProperty, PerceptBelief> {

	
	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(
			final GatewayPlaceProperty from) throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				from.placeId));

		result.put(FeatPlaceId.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public GatewayTransferFunction(ManagedComponent component, WMView<PerceptBelief> perceptBeliefs) {
		super(component, perceptBeliefs, Logger.getLogger(GatewayTransferFunction.class));

	}

}
