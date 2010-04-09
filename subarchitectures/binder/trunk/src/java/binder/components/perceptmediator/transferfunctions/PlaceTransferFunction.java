/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.Feature;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.arch.BinderException;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place, PerceptBelief> {

	@Override
	Map<Feature, FeatureValue> getFeatureValueMapping(Place from) {
		assert(from != null);
		Map<Feature, FeatureValue> result = new HashMap<Feature, FeatureValue>();
		// TODO: the features are stupid here!
		result.put(Feature.PlaceId, FeatureValueBuilder.createNewIntegerValue((int) from.id));
		result.put(Feature.PlaceStatus, FeatureValueBuilder.createNewStringValue(from.status.name()));
		return result;
	}

	@Override
	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException, BeliefException {
		PerceptBelief basePb = super.createBelief(id, curTime);
		PerceptBelief newPb = new PerceptBelief();
		newPb.estatus = basePb.estatus;
		newPb.frame = basePb.frame;
		newPb.hist = basePb.hist;
		newPb.id = id;
		newPb.content = null;

		return newPb;
	}

}
