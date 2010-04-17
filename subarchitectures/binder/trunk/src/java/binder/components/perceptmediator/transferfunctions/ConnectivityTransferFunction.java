package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.ConnectivityPathProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

public class ConnectivityTransferFunction
		extends
		DependentDiscreteTransferFunction<ConnectivityPathProperty, PerceptBelief> {

	static final String ATTR_CONNECTED1 = "element0";
	static final String ATTR_CONNECTED2 = "element1";
	static final String ATTR_CONNECTED = "connected";

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			final ConnectivityPathProperty from) throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaPlace1 = getReferredBelief(new PlaceMatchingFunction(
				from.place1Id));
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(new PlaceMatchingFunction(
				from.place2Id));

		result.put(ATTR_CONNECTED1, FeatureValueBuilder
				.createNewStringValue(wmaPlace1.id));
		result.put(ATTR_CONNECTED2, FeatureValueBuilder
				.createNewStringValue(wmaPlace2.id));
		result.put(ATTR_CONNECTED, FeatureValueBuilder
				.createNewBooleanValue(true));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public ConnectivityTransferFunction(ManagedComponent component, WMView<PerceptBelief> perceptBeliefs) {
		super(component, perceptBeliefs, Logger
				.getLogger(ConnectivityTransferFunction.class));

	}

	/* (non-Javadoc)
	 * @see binder.components.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction#createBelief(java.lang.String, cast.cdl.WorkingMemoryAddress, java.lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief createBelief(String id, WorkingMemoryAddress srcAddr,
			String type, CASTTime curTime) throws BeliefException {
		PerceptBelief bel= super.createBelief(id, srcAddr, type, curTime);
		bel.type = "relation";
		return bel;
	}
	
	

}
