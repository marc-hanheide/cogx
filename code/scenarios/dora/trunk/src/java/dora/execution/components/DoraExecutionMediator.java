package dora.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import execution.components.PlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.GoToPlace;
import execution.util.ActionConverter;
import facades.BinderFacade;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author nah
 * 
 */
public class DoraExecutionMediator extends PlanExecutionMediator implements
		ActionConverter {
	private final BinderFacade m_binderFacade;
	private static final String[] DEFAULT_LABELS = { "record1", "record2",
			"record3", "record4" };

	public DoraExecutionMediator() {
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

			GoToPlace act = newActionInstance(GoToPlace.class);
			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> placeUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
							dBelief.class));

			Formula placeProperty = placeUnion.getContent().get("PlaceId")
					.getDistribution().firstValue();
			act.placeID = placeProperty.getInteger();

			return act;
		} else if (_plannedAction.name.equals("look-for-object")) {
			assert _plannedAction.arguments.length == 2 : "look-for-object action arity is expected to be 2";
			String label = ((ElementaryFormula) _plannedAction.arguments[1]).prop;

			DetectObjects act = newActionInstance(DetectObjects.class);
			String[] labels = { label };
			act.labels = labels;
			return act;
		} else if (_plannedAction.name.equals("look-for-people")) {
			assert _plannedAction.arguments.length == 1 : "look-for-people action arity is expected to be 1";

			DetectPeople act = newActionInstance(DetectPeople.class);
			return act;
			// } else if (_plannedAction.name.equals("ask-for-placename")) {
			// assert _plannedAction.arguments.length == 2 :
			// "ask-for-feature action arity is expected to be 2";
			// String beliefID = ((PointerValue)
			// _plannedAction.arguments[1]).beliefId.id;
			// String featureID = "name";
			// ComsysQueryFeature act =
			// newActionInstance(ComsysQueryFeature.class);
			// act.beliefID = beliefID;
			// act.featureID = featureID;
			// return act;
			// } else if (_plannedAction.name.equals("verify-placename")) {
			// assert _plannedAction.arguments.length == 3 :
			// "ask-for-feature action arity is expected to be 2";
			// String beliefID = ((PointerValue)
			// _plannedAction.arguments[1]).beliefId.id;
			// String featureID = "name";
			// ComsysTestFeatureValue act =
			// newActionInstance(ComsysTestFeatureValue.class);
			// act.beliefID = beliefID;
			// act.featureType = featureID;
			// act.featureValue = _plannedAction.arguments[2];
			// return act;
		}
		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
