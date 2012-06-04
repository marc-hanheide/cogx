package dora.execution.components;

import java.util.Map;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import autogen.Planner.Action;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskForLabelExistence;
import execution.slice.actions.CreateConesForModel;
import execution.slice.actions.CreateRelationalConesForModel;
import execution.slice.actions.DetectObjects;
import execution.slice.actions.ExplorePlace;
import execution.slice.actions.GoToPlace;
import execution.slice.actions.LookForPeople;
import execution.slice.actions.ProcessConeGroupAction;
import execution.slice.actions.ProcessConesAtPlace;
import execution.slice.actions.ReportPosition;
import execution.slice.actions.TurnToHuman;
import execution.util.ActionConverter;

/**
 * Execution mediator specifically for Dora/Spring School.
 * 
 * @author nah
 * 
 */
public class DoraExecutionMediator extends BeliefBasedPlanExecutionMediator
		implements ActionConverter {

	protected boolean m_paused = false;
	private boolean m_gui = false;

	@Override
	protected void start() {
		super.start();

		if (m_gui) {
			JFrame jFrame = new JFrame("execution pauser");
			final JCheckBox jCheckbox = new JCheckBox("pause execution");
			jCheckbox.addChangeListener(new ChangeListener() {

				@Override
				public void stateChanged(ChangeEvent arg0) {
					m_paused = jCheckbox.isSelected();

				}
			});
			jFrame.add(jCheckbox);
			jFrame.pack();
			jFrame.setVisible(true);
		}

	}

	@Override
	protected void configure(Map<String, String> config) {

		super.configure(config);
		if (config.containsKey("--gui")) {
			m_gui = true;
		}
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
		if (_plannedAction.name.equals("move")
				|| _plannedAction.name.equals("move_direct")) {
			assert _plannedAction.arguments.length == 2 : "move action arity is expected to be 2";

			GoToPlace act = newActionInstance(GoToPlace.class);
			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> placeUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
							dBelief.class));

			Formula placeProperty = placeUnion.getContent().get(
					PlaceTransferFunction.PLACE_ID_ID).getDistribution()
					.firstValue();
			act.placeID = placeProperty.getInteger();

			return act;
		} else if (_plannedAction.name.equals("explore_place")) {
			assert _plannedAction.arguments.length == 2 : "explore_place action arity is expected to be 2";

			ExplorePlace act = newActionInstance(ExplorePlace.class);
			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> placeUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
							dBelief.class));

			Formula placeProperty = placeUnion.getContent().get(
					PlaceTransferFunction.PLACE_ID_ID).getDistribution()
					.firstValue();
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

			LookForPeople act = newActionInstance(LookForPeople.class);
			return act;

		} else if (_plannedAction.name.equals("create_cones")) {
			assert _plannedAction.arguments.length == 3 : "create_cones action arity is expected to be 3";

			WorkingMemoryAddress roomBeliefAddress = addressFromFormula(_plannedAction.arguments[2]);

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(
							roomBeliefAddress, GroundedBelief.class));

			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(this, gb,
					ComaRoom.class);

			CreateConesForModel act = newActionInstance(CreateConesForModel.class);
			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[1]);
			act.placeIDs = room.containedPlaceIds;

			return act;

		} else if (_plannedAction.name.equals("create_cones_in_room")) {
			assert _plannedAction.arguments.length == 3 : "create_cones action arity is expected to be 3";

			WorkingMemoryAddress roomBeliefAddress = addressFromFormula(_plannedAction.arguments[2]);

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(
							roomBeliefAddress, GroundedBelief.class));

			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(this, gb,
					ComaRoom.class);

			CreateRelationalConesForModel act = newActionInstance(CreateRelationalConesForModel.class);
			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[1]);
			act.relation = "inroom";
			act.roomID = room.roomId;

			return act;

		} else if (_plannedAction.name.equals("create_cones_at_object")) {
			// args: agent, label, support label, relation, object, room
			assert _plannedAction.arguments.length == 6 : "create_cones action arity is expected to be 3";

			WorkingMemoryAddress objectBeliefAddress = addressFromFormula(_plannedAction.arguments[4]);
			WorkingMemoryAddress roomBeliefAddress = addressFromFormula(_plannedAction.arguments[5]);

			IndependentFormulaDistributionsBelief<GroundedBelief> gb = IndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(
							roomBeliefAddress, GroundedBelief.class));

			ComaRoom room = BeliefUtils.getMostRecentPerceptAncestor(this, gb,
					ComaRoom.class);

			CreateRelationalConesForModel act = newActionInstance(CreateRelationalConesForModel.class);
			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[1]);
			act.supportObjectCategory = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[2]);
			act.relation = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[3]);
			act.supportObject = objectBeliefAddress.id; // FIXME: check if this
			// works
			act.roomID = room.roomId;

			return act;

		} else if (_plannedAction.name.equals("process_all_cones_at_place")) {
			assert _plannedAction.arguments.length == 3 : "process_all_cones_at_place action arity is expected to be 3";

			ProcessConesAtPlace act = newActionInstance(ProcessConesAtPlace.class);

			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> placeUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
							dBelief.class));

			Formula placeProperty = placeUnion.getContent().get(
					PlaceTransferFunction.PLACE_ID_ID).getDistribution()
					.firstValue();
			act.placeID = placeProperty.getInteger();

			act.model = stringFromElementaryFormula((ElementaryFormula) _plannedAction.arguments[2]);

			return act;
		} else if (_plannedAction.name.equals("ask-for-category-polar")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-category-polar action arity is expected to be 3";
			AskPolarIdentity act = createBeliefPlusStringAction(AskPolarIdentity.class, _plannedAction.arguments[1], _plannedAction.arguments[2]);
			return act;
		} else if (_plannedAction.name.equals("ask-for-object-existence")) {
			assert _plannedAction.arguments.length == 5 : "ask-for-object-existence action arity is expected to be 5";

			ElementaryFormula label = (ElementaryFormula) _plannedAction.arguments[1];
			dFormula belief = _plannedAction.arguments[2];
			ElementaryFormula relation = (ElementaryFormula) _plannedAction.arguments[3];
			dFormula target = _plannedAction.arguments[5];
			
			AskForLabelExistence act = newActionInstance(AskForLabelExistence.class);
			beliefPointerToBeliefAction(belief, act);
			act.pointer = addressFromFormula(target);
			act.label = stringFromElementaryFormula(label);
			act.relation = stringFromElementaryFormula(relation);

			return act;
		} else if (_plannedAction.name.equals("process_conegroup")) {
			assert _plannedAction.arguments.length == 2 : "process_conegroup action arity is expected to be 2";
			// cone group, place

			ProcessConeGroupAction act = newActionInstance(ProcessConeGroupAction.class);

			// Belief of cone group
			WMPointer beliefPtr = WMPointer.create(_plannedAction.arguments[1]);

			// read the belief from WM
			IndependentFormulaDistributionsBelief<dBelief> coneGroupUnion = IndependentFormulaDistributionsBelief
					.create(dBelief.class, getMemoryEntry(beliefPtr.getVal(),
							dBelief.class));

			// Get the ID
			Formula coneGroupIDProperty = coneGroupUnion.getContent().get("id")
					.getDistribution().firstValue();
			act.coneGroupID = coneGroupIDProperty.getInteger();

			act.coneGroupBeliefID = beliefPtr.getVal();
			// act.coneGroupBeliefID.id =
			// stringFromElementaryFormula((ElementaryFormula)_plannedAction.arguments[2]);
			// act.coneGroupBeliefID.subarchitecture = "binder";

			return act;

		} else if (_plannedAction.name.equals("engage")) {
			assert _plannedAction.arguments.length == 2 : "engage is expected to be of arity 2";
			return createSingleBeliefAction(TurnToHuman.class,
					_plannedAction.arguments[1]);

		} else if (_plannedAction.name.equals("report_position")) {
			assert _plannedAction.arguments.length == 2 : "report_position is expected to be of arity 2";
			return createSingleBeliefAction(ReportPosition.class,
					_plannedAction.arguments[1]);
		}
		System.out.println(_plannedAction.name);
		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);

	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

	@Override
	public boolean isPaused() {
		return m_paused;
	}

}
