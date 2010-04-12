/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
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
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place, PerceptBelief> {

	public PlaceTransferFunction() {
		super(Logger.getLogger(PlaceTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(Place from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put(FeatPlaceId.value, FeatureValueBuilder.createNewIntegerValue((int) from.id));
		result.put(FeatPlaceStatus.value, FeatureValueBuilder.createNewStringValue(from.status.name()));
		return result;
	}


}
