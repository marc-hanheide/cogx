/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

/**
 * @author marc
 *
 */
public class PlaceTransferFunction extends SimpleDiscreteTransferFunction<Place, PerceptBelief> {

	@Override
	Map<String, FeatureValue> getFeatureValueMapping(Place from) {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: the features are stupid here!
		result.put("PlaceId", FeatureValueBuilder.createNewIntegerValue((int) from.id));
		result.put("PlaceStatus", FeatureValueBuilder.createNewStringValue(from.status.name()));
		return result;
	}

	@Override
	public PerceptBelief createBelief(String id, WorkingMemoryAddress srcAddr, CASTTime curTime) throws BeliefException {

		PerceptBelief basePb = super.createBelief(id, srcAddr, curTime);
		PerceptBelief newPb = new PerceptBelief();
		newPb.estatus = basePb.estatus;
		newPb.frame = basePb.frame;
		newPb.hist = basePb.hist;
		newPb.id = id;
		newPb.content = null;

		return newPb;
	}

}
