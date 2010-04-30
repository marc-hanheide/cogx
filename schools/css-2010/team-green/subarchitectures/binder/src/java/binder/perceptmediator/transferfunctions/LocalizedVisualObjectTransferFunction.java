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
import cast.core.CASTUtils;
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
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import cast.core.CASTData ;


import java.io.*;
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
/** /	 		System.out.println(" Starting verbalization.....................................................");
 			String[] record_names = {"record1", "record2", "record3", "record4"} ;
 			// String[] person_names = { "person_1", "person_2", "person_3", "person_4"} ;

 			List<StableBelief> ls = new ArrayList<StableBelief>(); 
 			String str_tmp ; 

 			List<FeatureValueProbPair>  ls_pair = new ArrayList<FeatureValueProbPair>(); 
 			FeatureContentUtils tmp_util ;

 			component.getMemoryEntries(StableBelief.class, ls) ;

 			Iterator it=ls.iterator();
 			Belief stb ; 
 			//System.out.println("ls size: " + ls.size());
			String what_I_have_to_say = "espeak -s110 '" ;
 			while( it.hasNext() ) {
 				
 				stb = (Belief)it.next() ;
 				ls_pair = FeatureContentUtils.getValuesInBelief(stb, "name" ) ;
				//System.out.println("ls_pair size: " + ls_pair.size());
 				if ( ls_pair.size() > 0 ) {
 					Iterator it2=ls_pair.iterator();
 					if( it2.hasNext()  ) {
						System.out.println("--> I've entered it2.hasNext()");
 						FeatureValueProbPair fvp_tmp ;
 						fvp_tmp = (FeatureValueProbPair)it2.next() ;
 						StringValue val  = (StringValue)fvp_tmp.val ;
 						String vv  = val.val ;
						//System.out.println("Value of String vv: " + vv );						

						//System.out.println("stb type: " + stb.type );
						int detected_entity = 0 ;
						//int detectedVB = 0 ;
						String obstype = (String)stb.type ;
						String typeVB = "VisualObject" ; 
						String typePS = "Person" ;
						String name_to_use  ;

						if ( obstype.equals(typeVB)  ) {
							detected_entity = 1 ;
						} else if ( obstype.equals(typePS) ) {
							detected_entity = 2 ;
						}

						System.out.println("detected_entity: " + detected_entity ) ;

						if ( detected_entity != 0 ) {
							// we have detected a record							
 							List<FeatureValueProbPair>  ls_pair_det = new ArrayList<FeatureValueProbPair>();
 							ls_pair_det = FeatureContentUtils.getValuesInBelief(stb, "detected" ) ;
							//System.out.println("Value of ls_pair_det: " + ls_pair_det );

 							Iterator it_det = ls_pair_det.iterator();
 							fvp_tmp = (FeatureValueProbPair)it_det.next() ;
 							BooleanValue val_det  = (BooleanValue)fvp_tmp.val   ;
 							boolean have_detection = val_det.val ;

							System.out.println("Value of have_detection: " + have_detection );
 							if ( have_detection == true ){
 								// check for the place
 								List<FeatureValueProbPair>  ls_pair2 = new ArrayList<FeatureValueProbPair>();
 								ls_pair2 = FeatureContentUtils.getValuesInBelief(stb, "is-in" ) ;
 								Iterator it3 =ls_pair2.iterator();
 								fvp_tmp = (FeatureValueProbPair)it3.next() ; 							 	

 								PointerValue valxs  = (PointerValue)fvp_tmp.val   ;					
								

  								WorkingMemoryAddress wma = valxs.beliefId ;
							//	System.out.println("Value of wma: " + CASTUtils.toString(wma) );
								//System.out.print( "I've come to the point here..." ) ;
								
 								//CASTData beliefData = component.getWorkingMemoryEntry(wma ); //,StableBelief.class TemporalUnionBelief
								StableBelief beliefData = component.getMemoryEntry(wma, StableBelief.class ); 
								//System.out.print( "Have I come to here?...:" +beliefData ) ;

 								List<FeatureValueProbPair>  ls_pair_plc = new ArrayList<FeatureValueProbPair>(); 
 								ls_pair_plc = FeatureContentUtils.getValuesInBelief(beliefData, "PlaceId" ) ;
								//System.out.print( "Size of ls_pair_plc :" + ls_pair_plc.size() ) ;

 								Iterator it4 =ls_pair_plc.iterator();							
 								FeatureValueProbPair fvp_tmp2 = (FeatureValueProbPair)it4.next() ;

 								IntegerValue val_plc  = (IntegerValue)fvp_tmp2.val ;
 								int valofplc =  val_plc.val ;

								// extract room number from  beliefData
									
								List<FeatureValueProbPair>  ls_pair4 = new ArrayList<FeatureValueProbPair>();
 								ls_pair4 = FeatureContentUtils.getValuesInBelief(beliefData, "in-room" ) ;
 								it4 =ls_pair4.iterator();
 								fvp_tmp = (FeatureValueProbPair)it4.next() ; 
								PointerValue valxss  = (PointerValue)fvp_tmp.val   ;					
  								wma = valxss.beliefId ;
								
								StableBelief beliefDataX = component.getMemoryEntry(wma, StableBelief.class ); 
								List<FeatureValueProbPair>  ls_pair_plcc = new ArrayList<FeatureValueProbPair>();
								ls_pair_plcc = FeatureContentUtils.getValuesInBelief(beliefDataX, "RoomId" ) ;
								it4 =ls_pair_plcc.iterator();
								fvp_tmp2 = (FeatureValueProbPair)it4.next() ;
								val_plc  = (IntegerValue)fvp_tmp2.val ;
 								int valofRoom =  val_plc.val ;
							
								if ( detected_entity == 1 ) {
									what_I_have_to_say = what_I_have_to_say + "The record " + vv + " is at place " + valofplc + " which is in room number" + valofRoom + ". " ;
								} else if ( detected_entity == 2) {
									what_I_have_to_say = what_I_have_to_say + "The person " + vv + " is at place " + valofplc + " which is in room number" + valofRoom +". " ;
								}
 							}
							
						}

 					}
 				}
 			}
			what_I_have_to_say = what_I_have_to_say + "'";	
			System.out.println(  what_I_have_to_say);
/ * */

			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
			result.put("is-in", FeatureValueBuilder
					.createNewPointerValue(placeWMA));
			result.put("vo_id", FeatureValueBuilder//  VisualObjectId
					.createNewStringValue(wmc.address.id));
			result.put("name", FeatureValueBuilder// VisualObjectName
					.createNewStringValue(from.label));
			result.put("conf", FeatureValueBuilder.createNewFloatValue( from.detectionConfidence ));  
			
			boolean detected ; 
			if ( from.detectionConfidence > 0.5 ) {
				detected = true ;
			} else {
				detected = false ;
			}
			result.put("detected", FeatureValueBuilder.createNewBooleanValue(detected));
			if (detected){
				String detectedRecordName = from.label;
				try { 
					Runtime rt = Runtime.getRuntime(); 
				    	Process p = rt.exec("espeak -s110" + "'I_just_detected_a_record_called_" + detectedRecordName + "._who_bought_this_boring_old_crap?'");

				    	p.waitFor();
				}
				catch(Exception e) { 
				    System.out.println(e.getMessage()); 
				}
			}			

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
