/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.StringValue;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceStatus;
import beliefmodels.builders.FeatureValueBuilder;
import binder.components.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 *
 */
public class PlaceObjectContainmentTransferFunction extends SimpleDiscreteTransferFunction<PlaceContainmentObjectProperty, PerceptBelief> {

	public PlaceObjectContainmentTransferFunction() {
		super(Logger.getLogger(PlaceObjectContainmentTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(PlaceContainmentObjectProperty from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put(FeatPlaceId.value, FeatureValueBuilder.createNewIntegerValue((int) from.placeId));
		result.put("Label", FeatureValueBuilder.createNewStringValue(((StringValue) from.mapValue).value));
		
		return result;
	}


}
