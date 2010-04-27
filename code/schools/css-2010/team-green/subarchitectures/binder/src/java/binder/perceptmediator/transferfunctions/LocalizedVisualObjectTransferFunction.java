/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import VisionData.VisualObject;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import castutils.facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public class LocalizedVisualObjectTransferFunction extends
		DependentDiscreteTransferFunction<VisualObject> {

	public LocalizedVisualObjectTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedVisualObjectTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from) throws BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		try {
			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
			result.put("VisualObjectId", FeatureValueBuilder//result.put("VisualObjectId", FeatureValueBuilder
					.createNewStringValue(wmc.address.id));
			result.put("VisualObjectName", FeatureValueBuilder//result.put("VisualObjectId", FeatureValueBuilder
					.createNewStringValue(from.label));
			result.put("VisuaObjectIdConfidence", FeatureValueBuilder.createNewFloatValue(from.detectionConfidence));			
		//	result.put("distance", FeatureValueBuilder.createNewFloatValue(from.distance));
		} catch (BeliefException e) {
			component.logException(e);
		} catch (CASTException e) {
			component.logException(e);
			//System.out.print("An error occured in LocalizedVisualObjectTransferFunction!!"); 
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return result;
	}

}
