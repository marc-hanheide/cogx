package execution.components;

import java.util.List;

import java.util.Map;

import java.lang.Runtime;
import java.io.*;

import autogen.Planner.Action;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import cast.CASTException;
import castutils.facades.BinderFacade;
import execution.slice.ActionExecutionException;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.ComsysTestFeatureValue;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.GoToPlaceRough;
import execution.slice.actions.LookForObjects;
import execution.slice.actions.SpinAround;
import execution.slice.actions.PTULookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.PrintMessage;
import execution.util.ActionConverter;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author marc
 * 
 */
public class SpringSchoolExecutionMediator extends PlanExecutionMediator
		implements ActionConverter {
	private final BinderFacade m_binderFacade;
	private static final String[] DEFAULT_LABELS = { "record1", "record2",
			"record3", "record4" };

	private String[] m_objectLabels;
	private double m_tolerance;

	public SpringSchoolExecutionMediator() {
	    m_objectLabels = DEFAULT_LABELS;
		m_binderFacade = new BinderFacade(this);
	}

    private void say_something(String __command){

		try {

		    Runtime.getRuntime().exec(__command);
		    
		} catch (Exception e) {
		    //System.out.println("Exception " + e.getMessage());
		}
    }
    
	@Override
	protected void start() {
		super.start();
		m_binderFacade.start();


		
		try {
		    FileWriter fstream = new FileWriter("talk.sh");
		    BufferedWriter out = new BufferedWriter(fstream);
		    out.write("echo \"$1\" | festival --tts");
		    out.close();
		    
		    String command = "chmod a+x talk.sh";
		    Runtime.getRuntime().exec(command);
		} catch (Exception e) {
		    //System.out.println("Exception " + e.getMessage());
		}

// // 		// Make an executable bash script for generating noise.
// // 		FileWriter fstream = new FileWriter("talk.sh");
// // 		BufferedWriter out = new BufferedWriter(fstream);
// // 		out.write("echo \"" + args[0] + "\" | festival --tts");
// // 		out.close();
// // 		String command = "chmod a+x talk.sh";
// // 		Runtime.getRuntime().exec(command);
	}

	@Override
	protected void configure(Map<String, String> _config) {
		//System.exit(0);
		String labels = _config.get("--labels");
		if (labels != null) {
			m_objectLabels = labels.split(",");
		}
		log("using object labels: " + m_objectLabels);
		String tolstring = _config.get("--tolerance");
		if (tolstring != null) {
			m_tolerance = Double.parseDouble(tolstring);
		} else {
			m_tolerance = 0.5;
		}
		log("Using tolerance " + m_tolerance + "m");
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
// 	assert 1==2 : "Got action with name :: " + _plannedAction.name;

	
	

// 	System.exit(0);

	if (_plannedAction.name.equals("expensive-move")) {
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
	}else if (_plannedAction.name.equals("move")) {
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

	    
	    say_something("./talk.sh Hmmmmm-this-looks-like-a-nice-place-to-stop?-Yeah!");

	    // we have now created the action object, so we create it and let
	    // the execution framework to the rest of the work
	    return act;
	}else if (_plannedAction.name.equals("move-rough")) {
	    assert _plannedAction.arguments.length == 2 : "move action arity is expected to be 2";

	    // create a new instance of the action, look in the
	    // execution.slice.actions package to see all available actions:
	    // http://www.cs.bham.ac.uk/~hanheidm/spring-school-javadoc/execution/slice/actions/package-summary.html
	    GoToPlaceRough act = newActionInstance(GoToPlaceRough.class);

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
	    act.tol = new double[1];
	    act.tol[0] = m_tolerance;
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
	    act.labels = DEFAULT_LABELS;
	    return act;
	} else if (_plannedAction.name.equals("detect-people")) {
	    // this is the action that just triggers the people detector once
	    assert _plannedAction.arguments.length == 1 : "detect-people action arity is expected to be 1";
	    // create the action instance and return it
	    DetectPeople act = newActionInstance(DetectPeople.class);
	    return act;
	} else if (_plannedAction.name.equals("look-for-people-first")) {

	    // this is the action that make the robot turn and triggers the
	    // people detector several times
	    assert _plannedAction.arguments.length == 2 : "look-for-people-first action arity is expected to be 2 but got" + _plannedAction.arguments.length;

	    //System.exit(0);

	    // create the action instance and return it
	    LookForPeople act = newActionInstance(LookForPeople.class);
	    
	    
	    say_something("./talk.sh Anyone-about?");

	    return act;
	} else if (_plannedAction.name.equals("look-for-people-second")) {
	    // this is the action that make the robot turn and triggers the
	    // people detector several times
	    assert _plannedAction.arguments.length == 2 : "look-for-people-second action arity is expected to be 2 but got " + _plannedAction.arguments.length;
	    // create the action instance and return it
	    LookForPeople act = newActionInstance(LookForPeople.class);

	    say_something("./talk.sh Anyone-there?-I-want-a-friend...Please!");
	    
	    return act;
	} else if (_plannedAction.name.equals("look-for-people")) {
	    // this is the action that make the robot turn and triggers the
	    // people detector several times
	    assert _plannedAction.arguments.length == 1 : "look-for-people action arity is expected to be 1";
	    // create the action instance and return it
	    LookForPeople act = newActionInstance(LookForPeople.class);
	    return act;
	} else if (_plannedAction.name.equals("spin-around")) {
	    // this is the action that make the robot turn but does
	    // not trigger any detector since we have no vision
	    assert _plannedAction.arguments.length == 1 : "spin-around action arity is expected to be 1";
	    // create the action instance and return it
	    SpinAround act = newActionInstance(SpinAround.class);
	    return act;
	}   else if (_plannedAction.name.equals("ptu-look-for-objects")) {
		
	    assert _plannedAction.arguments.length == 2 : "ptu-look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    PTULookForObjects act = newActionInstance(PTULookForObjects.class);
	    act.labels = m_objectLabels;
	    return act;
	}   else if (_plannedAction.name.equals("cheap-look-for-objects")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    // 			DetectObjects act = newActionInstance(LookForObjects.class);
	    LookForObjects act = newActionInstance(LookForObjects.class);
	    // 			DetectObjects act = newActionInstance(DetectObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
	    return act;
	} else if (_plannedAction.name.equals("expensive-look-for-objects")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    // 			DetectObjects act = newActionInstance(LookForObjects.class);
	    LookForObjects act = newActionInstance(LookForObjects.class);
	    // 			DetectObjects act = newActionInstance(DetectObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
	    return act;
	}else if (_plannedAction.name.equals("look-for-objects")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    // 			DetectObjects act = newActionInstance(LookForObjects.class);
	    LookForObjects act = newActionInstance(LookForObjects.class);
	    // 			DetectObjects act = newActionInstance(DetectObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
	    return act;


	    // 			// this is the action that make the robot turn and triggers the
	    // 			// object detector several times
	    // 			assert _plannedAction.arguments.length == 1 : "look-for-objects action arity is expected to be 1";
	    // 			// create the action instance and return it
	    // 			LookForObjects act = newActionInstance(LookForObjects.class);
	    // 			act.labels = DEFAULT_LABELS;
	    // 			return act;
	}else if (_plannedAction.name.equals("look-for-objects-and-people-wave2")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    say_something("./talk.sh More information, more information, aaaahhh!");

	    LookForObjects act = newActionInstance(LookForObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
	    return act;

	}else if (_plannedAction.name.equals("look-for-objects-and-people-wave1")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    say_something("./talk.sh More-information,-more-information,-aaaahhh!");

	    LookForObjects act = newActionInstance(LookForObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
	    return act;

	}else if (_plannedAction.name.equals("look-for-objects-and-people")) {
		
	    assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2 but we got " + _plannedAction.arguments.length;
	    
	    say_something("./talk.sh More-information,-more-information,-aaaahhh!");
// 	    say_something("./talk.sh More information, more information, aaaahhh!");

	    LookForObjects act = newActionInstance(LookForObjects.class);
	    act.labels = m_objectLabels;// DEFAULT_LABELS;
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
	    act.featureID = "name";
	    // optionally we can define an explicit question here to make life
	    // simpler
	    act.question = "What is your name?";
	    say_something("./talk.sh What-is-your-name?");

// 	    String __command = "./talk.sh What-is-your-name?";
	    // Runtime.getRuntime().exec(__command);

	    return act;
	} else if (_plannedAction.name.equals("ask-for-placename")) {
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
	}
	// in case we do not find the action we have an exception
	throw new ActionExecutionException("No conversion available for: "
					   + _plannedAction.name);
    }

    @Override
	public ActionConverter getActionConverter() {

	return this;
    }

}
