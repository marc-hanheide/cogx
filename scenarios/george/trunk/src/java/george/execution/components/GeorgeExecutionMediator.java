package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForShape;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.util.ActionConverter;

/**
 * Execution mediator for George year 2.
 * 
 * @author nah
 * 
 */
public class GeorgeExecutionMediator extends BeliefBasedPlanExecutionMediator implements
		ActionConverter {

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

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-color-general is expected to be of artity 2";
			return createSingleBeliefAction(AskForColour.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-shape-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-shape-general is expected to be of artity 2";
			return createSingleBeliefAction(AskForShape.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-ident-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-ident-general is expected to be of artity 2";
			return createSingleBeliefAction(AskForIdentity.class,
					_plannedAction.arguments[1]);
		}
		else if (_plannedAction.name.equals("learn-color")) {

			assert _plannedAction.arguments.length == 3 : "learn-color is expected to be of artity 3";
			return createBeliefPlusStringAction(LearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		}
		else if (_plannedAction.name.equals("learn-shape")) {

			assert _plannedAction.arguments.length == 3 : "learn-shape is expected to be of artity 3";
			return createBeliefPlusStringAction(LearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		}		else if (_plannedAction.name.equals("learn-ident")) {

			assert _plannedAction.arguments.length == 3 : "learn-ident is expected to be of artity 3";
			return createBeliefPlusStringAction(LearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
