package dora.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.CreateConesForModel;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.DetectPeople;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.ProcessConesAtPlace;
import execution.util.ActionConverter;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author nah
 * 
 */
public class DoraExecutionMediator extends BeliefBasedPlanExecutionMediator
		implements ActionConverter {

	@Override
	protected void start() {
		super.start();
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
					.create(dBelief.class,
							getMemoryEntry(beliefPtr.getVal(), dBelief.class));

			Formula placeProperty = placeUnion.getContent().get(PlaceTransferFunction.PLACE_ID_ID)
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

		} else if (_plannedAction.name.equals("create_cones")) {
			assert _plannedAction.arguments.length == 3 : "create_cones action arity is expected to be 3";

			WorkingMemoryAddress roomBeliefAddress = addressFromFormula(_plannedAction.arguments[2]);

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class,
							getMemoryEntry(roomBeliefAddress,
									GroundedBelief.class));


			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(this, gb,
					ComaRoom.class);

			CreateConesForModel act = newActionInstance(CreateConesForModel.class);
			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[1]);
			act.placeIDs = room.containedPlaceIds;

			return act;

		} else if (_plannedAction.name.equals("process_all_cones_at_place")) {
			assert _plannedAction.arguments.length == 3 : "process_all_cones_at_place action arity is expected to be 3";

			ProcessConesAtPlace act = newActionInstance(ProcessConesAtPlace.class);
		
			
			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> placeUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class,
							getMemoryEntry(beliefPtr.getVal(), dBelief.class));

			Formula placeProperty = placeUnion.getContent().get(PlaceTransferFunction.PLACE_ID_ID)
					.getDistribution().firstValue();
			act.placeID = placeProperty.getInteger();

			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[2]);

			return act;
		}
		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);

	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

}
