package execution.components;

import java.util.List;

import autogen.Planner.Action;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import cast.CASTException;
import castutils.facades.BinderFacade;
import execution.slice.ActionExecutionException;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.ComsysTestFeatureValue;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.PrintMessage;
import execution.slice.actions.Start;
import execution.slice.actions.Report;
import execution.util.ActionConverter;

import execution.slice.TriBool;
import execution.slice.ActionStatus;


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
import cast.cdl.WorkingMemoryAddress;
import java.io.*;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author nah
 * 
 */
public class DoraExecutionMediator extends PlanExecutionMediator implements
		ActionConverter {
	private final BinderFacade m_binderFacade;
	private static final String[] DEFAULT_LABELS = {
	  "James", "Jesus", "Heartbreakers", "ChakaKhan"
	  /*"record1", "record2", "record3", "record4"*/
	};
	private String[] m_objectLabels;

	public DoraExecutionMediator() {
		m_binderFacade = new BinderFacade(this);
		m_objectLabels = DEFAULT_LABELS;
	}
/*
	@Override
	protected void configure(Map<String, String> _config) {
		String labels = _config.get("--labels");
		if (labels != null) {
			m_objectLabels = labels.split(",");
		}
		log("using object labels: " + m_objectLabels);
	}
*/
	@Override
	protected void start() {
		super.start();
		m_binderFacade.start();
	}

	/**
	 * Does the system specific work of converting a planning action into real
	 * system stuff.
	 * 
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	public execution.slice.Action toSystemAction(Action _plannedAction)
			throws CASTException {
	    if (_plannedAction.name.equals("move") || _plannedAction.name.equals("move_to_report_pos")) {
			assert _plannedAction.arguments.length == 2 : "move action arity is expected to be 2";

			// create a new instance of the action, look in the
			// execution.slice.actions package to see all available actions:
			// http://www.cs.bham.ac.uk/~hanheidm/spring-school-javadoc/execution/slice/actions/package-summary.html
			GoToPlace act = newActionInstance(GoToPlace.class);	       	

			// read the stable belief of the place that has been passed as an
			// argument of this action
			String stableBeliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;

			// read the belief from working memory using the binder facade
			// helper class
			Belief stableBelief = m_binderFacade.getBelief(stableBeliefID);

			if (stableBelief == null) {
				throw new ActionExecutionException(
						"No union for place union id: " + stableBeliefID);
			}

			// read all features PlaceId from the StableBelief by using the
			// binder facade again. should be exactly one
			List<FeatureValue> placeIDFeatures = m_binderFacade
					.getFeatureValue(stableBelief, "PlaceId");
			if (placeIDFeatures.isEmpty()) {
				throw new ActionExecutionException(
						"No PlaceId features for belief id: " + stableBeliefID);

			}

			// read the one placeID from the StableBelief
			IntegerValue placeID = (IntegerValue) placeIDFeatures.get(0);
			act.placeID = placeID.val;
			// we have now created the action object, so we create it and let
			// the execution framework to the rest of the work
			return act;
		} else if (_plannedAction.name.equals("explore_place")) {
			// this is an empty action that actually doesn't do anything
			return new ExplorePlace();
		} else if (_plannedAction.name.equals("detect-objects")) {
			// this is the action that just triggers the object detector once
			assert _plannedAction.arguments.length == 1 : "detect-objects action arity is expected to be 1";
			// create the action instance and initialize it with the default
			// labels. This will trigger the object detector for objects with
			// this label. check
			// instantiations/includes/vision.sa/vision-objects.cast or
			// instantiations/includes/vision.sa/vision-blobs.cast to setup the
			// corresponding components to accept these labels.
			DetectObjects act = newActionInstance(DetectObjects.class);
			act.labels = m_objectLabels;
			return act;
		} else if (_plannedAction.name.equals("detect-people")) {
			// this is the action that just triggers the people detector once
			assert _plannedAction.arguments.length == 1 : "detect-people action arity is expected to be 1";
			// create the action instance and return it
			DetectPeople act = newActionInstance(DetectPeople.class);
			return act;
		} else if (_plannedAction.name.equals("look-for-people")) {
			// this is the action that make the robot turn and triggers the
			// people detector several times
			assert _plannedAction.arguments.length == 1 : "look-for-people action arity is expected to be 1";
			// create the action instance and return it
			LookForPeople act = newActionInstance(LookForPeople.class);
			return act;
		} else if (_plannedAction.name.equals("look-for-objects")) {
			// this is the action that make the robot turn and triggers the
			// object detector several times
			assert _plannedAction.arguments.length == 1 : "look-for-objects action arity is expected to be 1";
			// create the action instance and return it
			LookForObjects act = newActionInstance(LookForObjects.class);
			act.labels = m_objectLabels;
			return act;
		} else if (_plannedAction.name.equals("ask-for-your-name")) {
			// this is the action to ask for a person's name
			assert _plannedAction.arguments.length == 2 : "ask-for-your-name action arity is expected to be 2";
			// create the action instance: This action will trigger a GUI
			// dialog. The feature that is being asked for (in this case "name")
			// will be written to the belief that is given as argument if the
			// user confirms that he knows it.
			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
			// the action requires the id of the StableBelief in which the
			// feature should be put
			act.beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			// this is the feature that shall be written
			act.featureID = "person-name";
			// optionally we can define an explicit question here to make life
			// simpler
			act.question = "What is your name?";
			return act;
		} /*else if (_plannedAction.name.equals("ask-for-your-record")) {
			// this is the action to ask for a person's name
			assert _plannedAction.arguments.length == 2 : "ask-for-your-name action arity is expected to be 2";
			// create the action instance: This action will trigger a GUI
			// dialog. The feature that is being asked for (in this case "name")
			// will be written to the belief that is given as argument if the
			// user confirms that he knows it.
			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
			// the action requires the id of the StableBelief in which the
			// feature should be put
			act.beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			// this is the feature that shall be written
			act.featureID = "name";
			// optionally we can define an explicit question here to make life
			// simpler
			act.question = "What is your name?";
			return act;
                        } */else if (_plannedAction.name.equals("ask-for-placename")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-feature action arity is expected to be 2";
			String beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			String featureID = "name";
			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
			act.question = "";
			act.beliefID = beliefID;
			act.featureID = featureID;
			return act;
		} else if (_plannedAction.name.equals("verify-placename")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-feature action arity is expected to be 2";
			String beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			String featureID = "name";
			ComsysTestFeatureValue act = newActionInstance(ComsysTestFeatureValue.class);
			act.question = "";
			act.beliefID = beliefID;
			act.featureType = featureID;
			act.featureValue = _plannedAction.arguments[2];
			return act;
		} else if (_plannedAction.name.equals("commit-name")) {
			PrintMessage act = newActionInstance(PrintMessage.class);
			act.status=ActionStatus.COMPLETE;
			act.success=TriBool.TRITRUE;
			act.message=_plannedAction.fullName;
			return act;
		} else if(_plannedAction.name.equals("start")) {
		    assert _plannedAction.arguments.length == 1 : "start action arity is expected to be 1";
		    Start act = newActionInstance(Start.class);
		    act.status = ActionStatus.COMPLETE;
		    act.success = TriBool.TRITRUE;
		    return act;
		} else if(_plannedAction.name.equals("report")) {
		    assert _plannedAction.arguments.length == 1 : "report action arity is expected to be 1";
		    Report act = newActionInstance(Report.class);
		    act.status = ActionStatus.COMPLETE;
		    act.success = TriBool.TRITRUE;
		
		String what_I_have_to_say = "espeak -s110 'From what I gather, " ;
		try{
			 

	 		//System.out.println(" Starting verbalization.....................................................");
 			//String[] record_names = {"record1", "record2", "record3", "record4"} ;
 			// String[] person_names = { "person_1", "person_2", "person_3", "person_4"} ;

 			List<StableBelief> ls = new ArrayList<StableBelief>(); 
 			String str_tmp ; 

 			List<FeatureValueProbPair>  ls_pair = new ArrayList<FeatureValueProbPair>(); 
 			FeatureContentUtils tmp_util ;

 			getMemoryEntries(StableBelief.class, ls) ;

 			Iterator it=ls.iterator();
 			Belief stb ; 
 			//System.out.println("ls size: " + ls.size());
			
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
								StableBelief beliefData = getMemoryEntry(wma, StableBelief.class ); 
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
								
								StableBelief beliefDataX = getMemoryEntry(wma, StableBelief.class ); 
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
			
			try { BufferedWriter out = new BufferedWriter(new FileWriter("/what_I_had_to_say.txt")); 
				out.write(what_I_have_to_say); out.close(); 
			} catch (IOException e) { } 

			
		}catch(Exception e){
			System.out.println(e.getMessage()); 
		}	




 

		    act.message = what_I_have_to_say ; //"espeak 'I_have_finished_my_task,Marc_come_and_see.'";
		    return act;
		}
		// in case we do not find the action we have an exception
		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.name);
	}


// 	public execution.slice.Action toSystemAction(Action _plannedAction)
// 			throws CASTException {
// 		if (_plannedAction.name.equals("move") || _plannedAction.name.equals("move_to_report_pos")) {
// 			assert _plannedAction.arguments.length == 2 : "move action arity is expected to be 1";

// 			GoToPlace act = newActionInstance(GoToPlace.class);
// 			String placeUnionID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
// 			Belief placeUnion = m_binderFacade.getBelief(placeUnionID);

// 			if (placeUnion == null) {
// 				throw new ActionExecutionException(
// 						"No union for place union id: " + placeUnionID);
// 			}

// 			List<FeatureValue> placeIDFeatures = m_binderFacade
// 					.getFeatureValue(placeUnion, "PlaceId");
// 			if (placeIDFeatures.isEmpty()) {
// 				throw new ActionExecutionException(
// 						"No PlaceId features for union id: " + placeUnionID);

// 			}
// 			IntegerValue placeID = (IntegerValue) placeIDFeatures.get(0);
// 			act.placeID = placeID.val;
// 			return act;
// 		} else if (_plannedAction.name.equals("look-for-object")) {
// 			assert _plannedAction.arguments.length == 1 : "look-for-object action arity is expected to be 1";

// 			DetectObjects act = newActionInstance(DetectObjects.class);
// 			act.labels = DEFAULT_LABELS;
// 			return act;
// 		} else if (_plannedAction.name.equals("look-for-people")) {
// 			assert _plannedAction.arguments.length == 1 : "look-for-people action arity is expected to be 1";

// 			DetectPeople act = newActionInstance(DetectPeople.class);
// 			return act;
// 		} else if (_plannedAction.name.equals("ask-for-placename")) {
// 			assert _plannedAction.arguments.length == 2 : "ask-for-feature action arity is expected to be 2";
// 			String beliefID =  ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
// 			String featureID = "name";
// 			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
//             act.beliefID = beliefID;
//             act.featureID = featureID;
// 			return act;
// 		} else if (_plannedAction.name.equals("verify-placename")) {
// 			assert _plannedAction.arguments.length == 3 : "ask-for-feature action arity is expected to be 2";
// 			String beliefID =  ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
// 			String featureID = "name";
// 			ComsysTestFeatureValue act = newActionInstance(ComsysTestFeatureValue.class);
//             act.beliefID = beliefID;
//             act.featureType = featureID;
//             act.featureValue = _plannedAction.arguments[2];
// 			return act;
// 		} else if(_plannedAction.name.equals("start")) {
// 		    assert _plannedAction.arguments.length == 1 : "start action arity is expected to be 1";
// 		    //execution.slice.Action act = newActionInstance(execution.slice.Action.class);
// 		    Start act = newActionInstance(Start.class);
// 		    act.status = ActionStatus.COMPLETE;
// 		    act.success = TriBool.TRITRUE;
// 		    System.out.println("Creating Start Action!!!");
// 		    return act;
// 		} else if(_plannedAction.name.equals("report")) {
// 		    assert _plannedAction.arguments.length == 1 : "report action arity is expected to be 1";
// 		    execution.slice.Action act = newActionInstance(execution.slice.Action.class);
// 		    act.status = ActionStatus.COMPLETE;
// 		    act.success = TriBool.TRITRUE;
// 		    return act;
// 		}

// 		throw new ActionExecutionException("No conversion available for: "
// 				+ _plannedAction.fullName);
// 	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
