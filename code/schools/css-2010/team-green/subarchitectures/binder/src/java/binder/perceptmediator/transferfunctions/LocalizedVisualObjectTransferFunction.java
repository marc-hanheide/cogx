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

import java.util.List ;
import java.util.ArrayList;

import beliefmodels.autogen.beliefs.StableBelief ; 
import java.util.Iterator;
import beliefmodels.autogen.distribs.*;
import beliefmodels.autogen.distribs.FeatureContentUtils;

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
	 		// scan thorugh all working memory stable entries...
			List<StableBelief> ls = new ArrayList<StableBelief>(); //Ice.Object
			String str_tmp ;
			//StableBelief tmp_stbel ;
			//StableBelief [] tmp_stbel2 ;

			List<FeatureValueProbPair>  ls_pair = new ArrayList<FeatureValueProbPair>(); 
			FeatureContentUtils tmp_util ;

			component.getMemoryEntries(StableBelief.class, ls) ;
			//component.getMemoryEntriesWithData(StableBelief.class, ls) ;
 
//wmc.address.id
// from.label

Iterator it=ls.iterator();

			StableBelief stb ; 
			stb = 	(StableBelief)it.next() ;

		String featlabel = "label" ;
		ls_pair = tmp_util.FeatureContentUtils.getValuesInBelief(stb, featlabel) ; 


 
 
		//	str_tmp =  stb.id ; //FeatureValueBuilder.createNewStringValue( ls[0].label ) ;
		//	System.out.print("--------------------------------------------------->" + str_tmp ); 
			


		/*	for ( int i = 0 ; i < ls.size() ; i++) {
				str_tmp =  FeatureValueBuilder.createNewStringValue( ls[i].label )) ;
				//System.out.print("An error occured in LocalizedVisualObjectTransferFunction!!"); 

			}*/
			

                      // component.getWorkingMemoryEntries(tmp_stbel) ;


		//	component.getMemoryEntries(ls) ;
 


			//
			


			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
			result.put("vo_id", FeatureValueBuilder//  VisualObjectId
					.createNewStringValue(wmc.address.id));
			result.put("name", FeatureValueBuilder// VisualObjectName
					.createNewStringValue(from.label));
			result.put("conf", FeatureValueBuilder.createNewFloatValue(from.detectionConfidence)); 

			boolean detected ; 
			if ( from.detectionConfidence > 0.5 ) {
				detected = true ;
			} else {
				detected = false ;
			}
			result.put("detected", FeatureValueBuilder.createNewBooleanValue(detected));			
			

			float siz ;
		if( from.views.length > 0 ) {		
			siz = (float)from.views[0].boundingBox.height * (float)from.views[0].boundingBox.width ; 
		}else{
			siz = 0 ; 
		}
		result.put("siz", FeatureValueBuilder.createNewFloatValue(siz)); 

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
