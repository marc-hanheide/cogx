package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.ConnectivityPathProperty;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

public class ConnectivityTransferFunction extends
		DependentDiscreteTransferFunction<ConnectivityPathProperty, PerceptBelief> {

	static final String ATTR_CONNECTED1="ConnectedTo1";
	static final String ATTR_CONNECTED2="ConnectedTo2";
	
	
	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(
			final ConnectivityPathProperty from) throws InterruptedException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

	
		WorkingMemoryAddress wmaPlace1 = getReferredBelief(new PlaceMatchingFunction(from.place1Id));
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(new PlaceMatchingFunction(from.place2Id));

		result.put(ATTR_CONNECTED1, FeatureValueBuilder
				.createNewStringValue(wmaPlace1.id));
		result.put(ATTR_CONNECTED2, FeatureValueBuilder
				.createNewStringValue(wmaPlace2.id));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public ConnectivityTransferFunction(WMView<PerceptBelief> perceptBeliefs) {
		super(perceptBeliefs, Logger.getLogger(ConnectivityTransferFunction.class));
		
	}


}
