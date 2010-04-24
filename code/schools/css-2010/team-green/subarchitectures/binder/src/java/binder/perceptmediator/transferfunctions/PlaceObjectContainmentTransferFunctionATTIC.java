/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentObjectProperty;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

/**
 * @author marc
 *
 */
public class PlaceObjectContainmentTransferFunctionATTIC extends DependentDiscreteTransferFunction<PlaceContainmentObjectProperty> {

	public PlaceObjectContainmentTransferFunctionATTIC(ManagedComponent component, WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger.getLogger(PlaceObjectContainmentTransferFunctionATTIC.class));
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc, PlaceContainmentObjectProperty from) throws BeliefException, InterruptedException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put("Label", FeatureValueBuilder.createNewStringValue(from.label));

		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				((IntegerValue) from.mapValue).value));
		result.put(FeatPlaceId.value, FeatureValueBuilder
				.createNewStringValue(wmaPlace.id));

		
		return result;
	}


}
