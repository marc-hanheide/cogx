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
import binder.perceptmediator.transferfunctions.helpers.ObjectLabelMatchingFunction;
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
public class LocalizedObjectTransferFunction extends
		DependentDiscreteTransferFunction<VisualObject> {

	public LocalizedObjectTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedObjectTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from) throws BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
                if (from.detectionConfidence == 0.0)
                {
                    //No confidence. Just return null.
                    return null;
                }
		try {
			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
                        /*PerceptBelief existingObjectBelief = tryGetReferredBelief(new ObjectLabelMatchingFunction(
                                from.label));
                        if (existingObjectBelief != null) {
                            //There's already a belief about an object with this
                            //label. Skip it.
                            return null;
                        }*/
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
			result.put("ObjectId", FeatureValueBuilder
					.createNewStringValue(wmc.address.id));
                        result.put("label", FeatureValueBuilder
                                .createNewStringValue(from.label));
		} catch (BeliefException e) {
			component.logException(e);
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return result;
	}

}
