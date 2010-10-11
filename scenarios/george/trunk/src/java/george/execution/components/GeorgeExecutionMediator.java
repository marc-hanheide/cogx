package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForObjectWithFeatureValue;
import execution.slice.actions.AskForShape;
import execution.slice.actions.AskPolarColour;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskPolarShape;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.util.ActionConverter;

/**
 * Execution mediator for George year 2.
 * 
 * @author nah
 * 
 */
public class GeorgeExecutionMediator extends BeliefBasedPlanExecutionMediator
		implements ActionConverter {

	public GeorgeExecutionMediator() {
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

		if (_plannedAction.name.equals("ask-for-an-objects-color-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-color-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForColour.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-shape-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-shape-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForShape.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-ident-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-ident-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForIdentity.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-color-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-color-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-shape-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-shape-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-ident-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-ident-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("learn-color")) {

			assert _plannedAction.arguments.length == 3 : "learn-color is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-shape")) {

			assert _plannedAction.arguments.length == 3 : "learn-shape is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-ident")) {

			assert _plannedAction.arguments.length == 3 : "learn-ident is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("unlearn-color")) {

			assert _plannedAction.arguments.length == 3 : "unlearn-color is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("unlearn-shape")) {
			assert _plannedAction.arguments.length == 3 : "unlearn-shape is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("unlearn-ident")) {
			assert _plannedAction.arguments.length == 3 : "unlearn-ident is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		}
		else if (_plannedAction.name.equals("ask-for-and-object-with-color")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-color is expected to be of arity 2";
			return createAskForAction("color",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-shape")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-shape is expected to be of arity 2";
			return createAskForAction("shape",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-ident")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-ident is expected to be of arity 2";
			return createAskForAction("ident",
					(ElementaryFormula) _plannedAction.arguments[1]);
		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	/**
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	private AskForObjectWithFeatureValue createAskForAction(String _feature,
			ElementaryFormula _value) throws CASTException {
		AskForObjectWithFeatureValue action = newActionInstance(AskForObjectWithFeatureValue.class);
		action.feature = _feature;
		action.value = stringFromElementaryFormula(_value);
		return action;
	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
