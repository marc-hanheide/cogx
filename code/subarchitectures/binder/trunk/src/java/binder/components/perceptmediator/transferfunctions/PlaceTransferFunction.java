/**
 * 
 */
package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.epstatus.EpistemicStatus;
import binder.autogen.featurecontent.Feature;
import binder.autogen.featurecontent.FeatureValue;
import binder.autogen.framing.SpatioTemporalFrame;
import binder.autogen.history.PerceptHistory;
import binder.builders.EpistemicStatusBuilder;
import binder.builders.FeatureValueBuilder;
import binder.builders.PerceptBuilder;
import binder.builders.SpatioTemporalFrameBuilder;
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
	public PerceptBelief createBelief(String id, CASTTime curTime) throws BinderException {
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
