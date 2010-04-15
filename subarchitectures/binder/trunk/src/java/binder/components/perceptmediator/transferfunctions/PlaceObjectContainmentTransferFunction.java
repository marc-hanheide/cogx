/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.IntegerValue;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 *
 */
public class PlaceObjectContainmentTransferFunction extends SimpleDiscreteTransferFunction<PlaceContainmentObjectProperty, PerceptBelief> {

	public PlaceObjectContainmentTransferFunction() {
		super(Logger.getLogger(PlaceObjectContainmentTransferFunction.class));
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(PlaceContainmentObjectProperty from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put(FeatPlaceId.value, FeatureValueBuilder.createNewIntegerValue((int)((IntegerValue) from.mapValue).value));
//		result.put("Label", FeatureValueBuilder.createNewStringValue(((StringValue) from.mapValue).value));
		
		return result;
	}


}
