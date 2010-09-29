package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import execution.components.PlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForShape;
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
			AskForColour action = newActionInstance(AskForColour.class);
			return beliefPointerToBeliefAction(_plannedAction.arguments[1],
					action);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-shape-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-shape-general is expected to be of artity 2";
			AskForShape action = newActionInstance(AskForShape.class);
			return beliefPointerToBeliefAction(_plannedAction.arguments[1],
					action);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-ident-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-ident-general is expected to be of artity 2";
			AskForIdentity action = newActionInstance(AskForIdentity.class);
			return beliefPointerToBeliefAction(_plannedAction.arguments[1],
					action);
		}

		//
		//
		//
		// if (_plannedAction.name.equals("move")) {
		// assert _plannedAction.arguments.length == 2 :
		// "move action arity is expected to be 2";
		//
		// GoToPlace act = newActionInstance(GoToPlace.class);
		// WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);
		//
		// // read the belief from WM
		// IndependentFormulaDistributionsBelief<dBelief> placeUnion =
		// IndependentFormulaDistributionsBelief
		// .create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
		// dBelief.class));
		//
		// Formula placeProperty = placeUnion.getContent().get("PlaceId")
		// .getDistribution().firstValue();
		// act.placeID = placeProperty.getInteger();
		//
		// return act;
		// } else
		// // if (_plannedAction.name.equals("look-for-object")) {
		// // assert _plannedAction.arguments.length == 1 :
		// // "look-for-object action arity is expected to be 1";
		// //
		// // DetectObjects act = newActionInstance(DetectObjects.class);
		// // act.labels = DEFAULT_LABELS;
		// // return act;
		// // } else if (_plannedAction.name.equals("look-for-people")) {
		// // assert _plannedAction.arguments.length == 1 :
		// // "look-for-people action arity is expected to be 1";
		// //
		// // DetectPeople act = newActionInstance(DetectPeople.class);
		// // return act;
		// // // } else if (_plannedAction.name.equals("ask-for-placename")) {
		// // // assert _plannedAction.arguments.length == 2 :
		// // // "ask-for-feature action arity is expected to be 2";
		// // // String beliefID = ((PointerValue)
		// // // _plannedAction.arguments[1]).beliefId.id;
		// // // String featureID = "name";
		// // // ComsysQueryFeature act =
		// // // newActionInstance(ComsysQueryFeature.class);
		// // // act.beliefID = beliefID;
		// // // act.featureID = featureID;
		// // // return act;
		// // // } else if (_plannedAction.name.equals("verify-placename")) {
		// // // assert _plannedAction.arguments.length == 3 :
		// // // "ask-for-feature action arity is expected to be 2";
		// // // String beliefID = ((PointerValue)
		// // // _plannedAction.arguments[1]).beliefId.id;
		// // // String featureID = "name";
		// // // ComsysTestFeatureValue act =
		// // // newActionInstance(ComsysTestFeatureValue.class);
		// // // act.beliefID = beliefID;
		// // // act.featureType = featureID;
		// // // act.featureValue = _plannedAction.arguments[2];
		// // // return act;
		// // }
		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	/**
	 * @param _arguments
	 * @param action
	 * @return
	 */
	private SingleBeliefAction beliefPointerToBeliefAction(dFormula _argument,
			SingleBeliefAction action) {
		WMPointer beliefPtr = WMPointer.create(_argument);
		action.beliefAddress = beliefPtr.getVal();
		return action;
	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
