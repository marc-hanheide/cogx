package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import execution.components.PlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForShape;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.SingleBeliefAction;
import execution.util.ActionConverter;

/**
 * Execution mediator for George year 2.
 * 
 * @author nah
 * 
 */
public class GeorgeExecutionMediator extends PlanExecutionMediator implements
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

	/**
	 * @param _arguments
	 * @param action
	 * @return
	 */
	private <T extends SingleBeliefAction> SingleBeliefAction beliefPointerToBeliefAction(
			dFormula _argument, T action) {
		WMPointer beliefPtr = WMPointer.create(_argument);
		action.beliefAddress = beliefPtr.getVal();
		return action;
	}

	private <T extends SingleBeliefAction> T createSingleBeliefAction(
			Class<T> _cls, dFormula _belief) throws CASTException {
		T action = newActionInstance(_cls);
		return (T) beliefPointerToBeliefAction(_belief, action);
	}

	private <T extends BeliefPlusStringAction> T createBeliefPlusStringAction(
			Class<T> _cls, dFormula _belief, dFormula _value)
			throws CASTException {
		T action = newActionInstance(_cls);
		ElementaryFormula formula = (ElementaryFormula) _value;
		GenericFormula<ElementaryFormula> elementaryFormula = GenericFormula
				.create(ElementaryFormula.class, formula);
		action.value = elementaryFormula.getProposition();
		return (T) beliefPointerToBeliefAction(_belief, action);
	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
