/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

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
public class VisualObjectTransferFunction extends SimpleDiscreteTransferFunction<VisualObject> {

	public VisualObjectTransferFunction(ManagedComponent component) {
		super(component, Logger.getLogger(VisualObjectTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected
	Map<String, FeatureValue> getFeatureValueMapping(WorkingMemoryChange wmc,  VisualObject from) throws BeliefException {
		assert(from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		result.put("VisualObjectId", FeatureValueBuilder.createNewStringValue(wmc.address.id));
		result.put("label", FeatureValueBuilder.createNewStringValue(from.label));

		if (from.detectionConfidence>0.5)
		    result.put("detected", FeatureValueBuilder.createNewBooleanValue(true));
		else
		    result.put("detected", FeatureValueBuilder.createNewBooleanValue(false));

		return result;
	}


}
