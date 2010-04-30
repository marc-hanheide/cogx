package execution.components;

import java.util.List;

import autogen.Planner.Action;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
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
import execution.slice.actions.LookForObjects;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.PrintMessage;
import execution.util.ActionConverter;
import violetsound.AePlayWave;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author marc
 * 
 */
public class SpringSchoolExecutionMediator extends PlanExecutionMediator
		implements ActionConverter {
	private final BinderFacade m_binderFacade;
	private static final String[] DEFAULT_LABELS = { "chakakhan", "heartbreakers",
			"james", "jesusjones" };

	public SpringSchoolExecutionMediator() {
		m_binderFacade = new BinderFacade(this);
	}

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
		if (_plannedAction.name.equals("move")) {
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
			act.labels = DEFAULT_LABELS;
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
			act.labels = DEFAULT_LABELS;
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
			return act;
		} else if (_plannedAction.name.equals("ask-for-your-record")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-feature action arity is expected to be 2";
			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
			act.question = "What's your record?";
			act.beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			act.featureID = "person-record";
			return act;
		} else if (_plannedAction.name.equals("ask-for-record-room")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-feature action arity is expected to be 3";
			String recordName = ((StringValue) _plannedAction.arguments[2]).val;
			ComsysQueryFeature act = newActionInstance(ComsysQueryFeature.class);
			act.question = "In which room is the record "+ recordName +"?";
			act.beliefID = ((PointerValue) _plannedAction.arguments[1]).beliefId.id;
			act.featureID = "is-in-room";
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
		} else if (_plannedAction.name.equals("report-finished")) {
			new AePlayWave("wavs/GlaDOS-okay_the_test_is_over_now.wav").start();
			PrintMessage act = newActionInstance(PrintMessage.class);
			act.status=ActionStatus.COMPLETE;
			act.success=TriBool.TRITRUE;
			act.message=_plannedAction.fullName;
			return act;
		} else if (_plannedAction.name.equals("commit-name")) {
			PrintMessage act = newActionInstance(PrintMessage.class);
			act.status=ActionStatus.COMPLETE;
			act.success=TriBool.TRITRUE;
			act.message=_plannedAction.fullName;
			return act;
		} else if (_plannedAction.name.equals("acknowledge-object") ||
		    _plannedAction.name.equals("acknowledge-asked-one") ||
		    _plannedAction.name.equals("acknowledge-asked-two") ||
		    _plannedAction.name.equals("acknowledge-asked-four") ||
		    _plannedAction.name.equals("acknowledge-asked-three") ) {
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
