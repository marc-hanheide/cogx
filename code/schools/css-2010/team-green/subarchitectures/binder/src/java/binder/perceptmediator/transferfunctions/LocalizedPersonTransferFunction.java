/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import VisionData.Person;
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
public class LocalizedPersonTransferFunction extends
		DependentDiscreteTransferFunction<Person> {

	public LocalizedPersonTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedPersonTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, Person from) throws BeliefException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		// TODO: we should use a DoubleValue here!
		try {
			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
			result.put("PersonId", FeatureValueBuilder
					.createNewStringValue(wmc.address.id));
                        result.put("record", FeatureValueBuilder.createNewStringValue("not_asked"));
			result.put("distance", FeatureValueBuilder.createNewFloatValue(from.distance));
			result.put("name", FeatureValueBuilder.createNewStringValue("not_asked"));

 		

			boolean detected ; 
			if ( from.distance < 3.0 ) {
				detected = true ;
			} else {
				detected = false ;
			} 
			result.put("detected", FeatureValueBuilder.createNewBooleanValue(detected)); 

			if (detected){
				try { 
					Runtime rt = Runtime.getRuntime(); 
				    	Process p = rt.exec("espeak -s90 'I_have_detected_a_pathetic_human_Answer_my_questions_or_I_will_destroy_you.'");

				   // 	p.waitFor();
				}
				catch(Exception e) { 
				    System.out.println(e.getMessage()); 
				}
			}



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
