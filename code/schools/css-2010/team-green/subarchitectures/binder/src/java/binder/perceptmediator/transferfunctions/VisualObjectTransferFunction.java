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
		result.put("vo_id", FeatureValueBuilder.createNewStringValue(wmc.address.id));//VisualObjectId
		result.put("name", FeatureValueBuilder.createNewStringValue(from.label));//VisualObjectName
		result.put("conf", FeatureValueBuilder.createNewFloatValue(from.detectionConfidence));//VisuaObjectIdConfidence
float siz ;
		if( from.views.length > 0 ) {		
			siz = (float)from.views[0].boundingBox.height * (float)from.views[0].boundingBox.width ; 
		}else{
			siz = 0 ; 
		}
result.put("siz", FeatureValueBuilder.createNewFloatValue(siz));

 
		//result.put("distance", FeatureValueBuilder.createNewFloatValue(from.distance));
		return result;
	}


}
