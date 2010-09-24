package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.GatewayPlaceProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

public class GatewayTransferFunction
		extends
		DependentDiscreteTransferFunction<GatewayPlaceProperty> {

	
	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc, 
			final GatewayPlaceProperty from) throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				from.placeId));

		result.put(FeatPlaceId.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));
		result.put("isGateway", new BooleanValue(true));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public GatewayTransferFunction(ManagedComponent component, WMView<PerceptBelief> perceptBeliefs) {
		super(component, perceptBeliefs, Logger.getLogger(GatewayTransferFunction.class));

	}

}
