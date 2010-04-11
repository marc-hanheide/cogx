package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.GatewayPlaceProperty;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

public class GatewayTransferFunction
		extends
		DependentDiscreteTransferFunction<GatewayPlaceProperty, PerceptBelief> {

	
	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(
			final GatewayPlaceProperty from) throws InterruptedException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				from.placeId));

		result.put(PlaceMatchingFunction.PLACE_ID, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public GatewayTransferFunction(WMView<PerceptBelief> perceptBeliefs) {
		super(perceptBeliefs, Logger.getLogger(GatewayTransferFunction.class));

	}

}
