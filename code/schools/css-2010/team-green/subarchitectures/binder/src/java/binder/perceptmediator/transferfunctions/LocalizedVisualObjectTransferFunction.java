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
import beliefmodels.autogen.featurecontent.BooleanValue ;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
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
		/* * /	
			String[] record_names = {"record_1", "record_2", "record_3", "record_4"} ;
 			//String[] person_names = { "person_1", "person_2", "person_3", "person_4"} ;	

			List<StableBelief> ls = new ArrayList<StableBelief>(); //Ice.Object
			String str_tmp ; 

			List<FeatureValueProbPair>  ls_pair = new ArrayList<FeatureValueProbPair>(); 
			FeatureContentUtils tmp_util ;

			component.getMemoryEntries(StableBelief.class, ls) ;

			Iterator it=ls.iterator();
			Belief stb ; 
				
			while( it.hasNext() ) {
				String what_I_have_to_say = "";
				stb = (Belief)it.next() ;
				ls_pair = FeatureContentUtils.getValuesInBelief(stb, "name" ) ;
				if ( ls_pair.size() > 0 ) {
					Iterator it2=ls_pair.iterator();
					if( it2.hasNext()  ) {
						FeatureValueProbPair fvp_tmp ;
						fvp_tmp = (FeatureValueProbPair)it2.next() ;
						StringValue val  = (StringValue)fvp_tmp.val ;
						String vv  = val.val ;
						
						int detected_entity = 0 ; // 1 ... record, 2 ... person
						for ( int i_r = 0 ; i_r < record_names.length; i_r++ ) { 
							if ( vv.equals(record_names[i_r])) {
								detected_entity = 1 ;
								break ;			
							}
						}
/*
						for ( int i_r = 0 ; i_r < person_names.length; i_r++ ) { 
							if ( vv.equals(record_names[i_r])) {
								detected_entity = 2 ;
								break ;			
							}
						}
* /

						if ( detected_entity == 1 ) {//vv.equals("record_1") || vv.equals("record_2") || vv.equals("record_3") || vv.equals("record_4")) 
							// we have detected a record							
							List<FeatureValueProbPair>  ls_pair_det = new ArrayList<FeatureValueProbPair>();
							ls_pair_det = FeatureContentUtils.getValuesInBelief(stb, "detected" ) ;
							Iterator it_det = ls_pair_det.iterator();
							fvp_tmp = (FeatureValueProbPair)it_det.next() ;
							BooleanValue val_det  = (BooleanValue)fvp_tmp.val   ;
							boolean have_detection = val_det.val ;
							if ( have_detection == true ){
								// check for the place
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
								what_I_have_to_say = "Record "+ vv + " is at place " + valofplc ;
								System.out.print( what_I_have_to_say ) ;
							}
						} else if ( detected_entity == 2 ) {
/ *
							// we have detected a person: check if its detected, where it is, which record it owns 							
							List<FeatureValueProbPair>  ls_pair_det = new ArrayList<FeatureValueProbPair>();
							ls_pair_det = FeatureContentUtils.getValuesInBelief(stb, "detected" ) ;
							Iterator it_det = ls_pair_det.iterator();
							fvp_tmp = (FeatureValueProbPair)it_det.next() ;
							BooleanValue val_det  = (BooleanValue)fvp_tmp.val   ;
							boolean have_detection = val_det.val ;
							if ( have_detection == true ){
								// check for the place
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
 
								what_I_have_to_say = "Person "+ vv + " is at place " + valofplc + "and owns a record " + name_of_my_record ;
								System.out.print( what_I_have_to_say ) ;
							}
* /
						}				
					}
				}
			}
			 

 /**/
			


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
