package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.ConnectivityPathProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatConnected;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement0;
import beliefmodels.autogen.featurecontent.featurenames.RelationElement1;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

public class ConnectivityTransferFunction extends
		DependentDiscreteTransferFunction<ConnectivityPathProperty> {

	static final String ATTR_CONNECTED = "connected";

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, final ConnectivityPathProperty from)
			throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		WorkingMemoryAddress wmaPlace1 = getReferredBelief(new PlaceMatchingFunction(
				from.place1Id));
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(new PlaceMatchingFunction(
				from.place2Id));

		result.put(RelationElement0.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace1.id));
		result.put(RelationElement1.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace2.id));
		result.put(FeatConnected.value, FeatureValueBuilder
				.createNewBooleanValue(true));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public ConnectivityTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> perceptBeliefs) {
		super(component, perceptBeliefs, Logger
				.getLogger(ConnectivityTransferFunction.class));

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seebinder.components.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, ConnectivityPathProperty from) {
		PerceptBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

}
