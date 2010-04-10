/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place, PerceptBelief> {

	@Override
	Map<String, FeatureValue> getFeatureValueMapping(Place from) {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

		result.put("PlaceId", FeatureValueBuilder.createNewIntegerValue((int) from.id));
		result.put("PlaceStatus", FeatureValueBuilder.createNewStringValue(from.status.name()));
		return result;
	}


}
