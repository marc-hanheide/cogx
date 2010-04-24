/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import comadata.ComaRoom;

import VisionData.VisualObject;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 *
 */
public class ComaRoomTransferFunction extends SimpleDiscreteTransferFunction<ComaRoom> {

	public ComaRoomTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(ComaRoomTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc, ComaRoom from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		result.put("RoomId", FeatureValueBuilder.createNewStringValue(wmc.address.id));
		result.put("concepts", FeatureValueBuilder.createNewStringValue(from.concepts[0]));
		return result;
	}


}
