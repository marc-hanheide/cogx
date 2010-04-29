/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceId;
import beliefmodels.autogen.featurecontent.featurenames.FeatPlaceStatus;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place> {

	public PlaceTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(PlaceTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc, Place from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		result.put(FeatPlaceId.value, FeatureValueBuilder.createNewIntegerValue((int) from.id));
		result.put(FeatPlaceStatus.value, FeatureValueBuilder.createNewStringValue(from.status.name()));

		if (from.connectionCounter==1)
		    result.put("endnode", FeatureValueBuilder.createNewBooleanValue(true));
		else
		    result.put("endnode", FeatureValueBuilder.createNewBooleanValue(false));


		return result;
	}


}
