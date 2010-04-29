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
import beliefmodels.autogen.beliefs.Belief ;
import java.util.Iterator;
import beliefmodels.autogen.distribs.*;
import beliefmodels.utils.FeatureContentUtils;

import java.lang.String ; 
import beliefmodels.autogen.featurecontent.StringValue ;
import beliefmodels.autogen.featurecontent.FeatureValue ;
import beliefmodels.autogen.featurecontent.PointerValue ;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.beliefs.PerceptBelief;
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
		/*	List<StableBelief> ls = new ArrayList<StableBelief>(); //Ice.Object
			String str_tmp ; 

			List<FeatureValueProbPair>  ls_pair = new ArrayList<FeatureValueProbPair>(); 
			FeatureContentUtils tmp_util ;

			component.getMemoryEntries(StableBelief.class, ls) ;

			Iterator it=ls.iterator();
			Belief stb ; 
				
			while( it.hasNext() ) {
				stb = (Belief)it.next() ;
				ls_pair = FeatureContentUtils.getValuesInBelief(stb, "name" ) ;
				if ( ls_pair.size() > 0 ) {
					Iterator it2=ls_pair.iterator();
					if( it2.hasNext()  ) {
						FeatureValueProbPair fvp_tmp ;
						fvp_tmp = (FeatureValueProbPair)it2.next() ;
						StringValue val  = (StringValue)fvp_tmp.val ;
						String vv  = val.val ;
						if ( vv.equals("record_1") || vv.equals("record_2") || vv.equals("record_3") || vv.equals("record_4")) {
							List<FeatureValueProbPair>  ls_pair2 = new ArrayList<FeatureValueProbPair>();
							ls_pair2 = FeatureContentUtils.getValuesInBelief(stb, "is-in" ) ;
							Iterator it3 =ls_pair2.iterator();
							fvp_tmp = (FeatureValueProbPair)it3.next() ;
							 
							PointerValue  valxs  = (PointerValue)fvp_tmp.val   ;
 							WorkingMemoryAddress wma = valxs.beliefId ;

							StableBelief beliefData = component.getMemoryEntry(wma,StableBelief.class );

							List<FeatureValueProbPair>  ls_pair_plc = new ArrayList<FeatureValueProbPair>(); 
							ls_pair_plc = FeatureContentUtils.getValuesInBelief(beliefData, "PlaceId" ) ;
							Iterator it4 =ls_pair_plc.iterator();							
							FeatureValueProbPair fvp_tmp2 = (FeatureValueProbPair)it4.next() ;

							IntegerValue val_plc  = (IntegerValue)fvp_tmp2.val ;
							int valofplc =  val_plc.val ;

							System.out.print("----------------------->" + vv + " is at place " + valofplc  );
						}				
					}
				}
			}
			 

 */
			


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
