package dialogue.execution.dora;

import java.util.ArrayList;

import javax.swing.JOptionPane;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.slice.TriBool;
import execution.slice.actions.ReportPosition;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;

/**
 * Receives actions from the execution system and interfaces with the rest of
 * the dialogue system.
 * 
 * @author nah
 * 
 */
public class DialogueActionInterface extends
		dialogue.execution.DialogueActionInterface {

	public static class ReportPositionDialogue
			extends
			dialogue.execution.DialogueActionInterface.BeliefIntentionDialogueAction<ReportPosition> {

		public ReportPositionDialogue(ManagedComponent _component) {
			super(_component, ReportPosition.class);
		}

		@Override
		protected ArrayList<dFormula> getPostconditions(
				WorkingMemoryAddress _groundedBeliefAddress,
				WorkingMemoryAddress _sharedBeliefAddress) {
			ArrayList<dFormula> postconditions = super.getPostconditions(
					_groundedBeliefAddress, _sharedBeliefAddress);

			postconditions.add(PropositionFormula.create("grounded").get());

			// postcondition is about the grounded belief
			postconditions.add(new ModalFormula(-1, "about", WMPointer.create(
					_groundedBeliefAddress,
					CASTUtils.typeName(GroundedBelief.class)).get()));

			try {

				// get the grounded belief
				GroundedBelief belief = getComponent().getMemoryEntry(
						getAction().beliefAddress, GroundedBelief.class);

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, belief);

				// get place id that object is at
				WMPointer placePointer = WMPointer
						.create(gb
								.getContent()
								.get(eu.cogx.perceptmediator.dora.VisualObjectTransferFunction.IS_IN)
								.getDistribution().getMostLikely().get());

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> placeBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class,
								getComponent().getMemoryEntry(
										placePointer.getVal(),
										GroundedBelief.class));

				WMPointer roomPointer = WMPointer.create(placeBelief
						.getContent().get(RoomMembershipMediator.ROOM_PROPERTY)
						.getDistribution().getMostLikely().get());

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> roomBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class,
								getComponent().getMemoryEntry(
										roomPointer.getVal(),
										GroundedBelief.class));

				// start with a default room
				String room = "room";

				FormulaDistribution categoryDistribution = roomBelief
						.getContent().get(ComaRoomTransferFunction.CATEGORY_ID);
				if (categoryDistribution != null) {
					room = categoryDistribution.getDistribution()
							.getMostLikely().getProposition();
				}

				ModalFormula inRoom = new ModalFormula(-1, "inRoom",
						PropositionFormula.create(room).get());
				ModalFormula content = new ModalFormula(-1, "content", inRoom);
				postconditions.add(content);

			} catch (DoesNotExistOnWMException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			return postconditions;
		}

		@Override
		protected void actionComplete() {
			try {
				((dialogue.execution.DialogueActionInterface) getComponent())
						.addBooleanFeature(getAction().beliefAddress,
								"position-reported", true);
			} catch (CASTException e) {
				logException("Problem while updating belief", e);
			}
		}

	}

	public static class DirectReportPosition extends
			BlockingActionExecutor<ReportPosition> {

		public DirectReportPosition(ManagedComponent _component) {
			super(_component, ReportPosition.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {
				GroundedBelief belief = getComponent().getMemoryEntry(
						getAction().beliefAddress, GroundedBelief.class);
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, belief);

				// get something to describe the object
				String objectIdent = "object";

				FormulaDistribution distribution = gb.getContent().get(
						VisualObjectTransferFunction.LABEL_ID);
				if (distribution != null) {
					objectIdent = distribution.getDistribution()
							.getMostLikely().getProposition();
				}

				// get place id that object is at
				WMPointer placePointer = WMPointer
						.create(gb
								.getContent()
								.get(eu.cogx.perceptmediator.dora.VisualObjectTransferFunction.IS_IN)
								.getDistribution().getMostLikely().get());
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> placeBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class,
								getComponent().getMemoryEntry(
										placePointer.getVal(),
										GroundedBelief.class));
				int placeID = placeBelief.getContent()
						.get(PlaceTransferFunction.PLACE_ID_ID)
						.getDistribution().getMostLikely().getInteger();

				WMPointer roomPointer = WMPointer.create(placeBelief
						.getContent().get(RoomMembershipMediator.ROOM_PROPERTY)
						.getDistribution().getMostLikely().get());

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> roomBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class,
								getComponent().getMemoryEntry(
										roomPointer.getVal(),
										GroundedBelief.class));

				// start with a default room
				String room = "room";

				FormulaDistribution categoryDistribution = roomBelief
						.getContent().get(ComaRoomTransferFunction.CATEGORY_ID);
				if (categoryDistribution != null) {
					room = categoryDistribution.getDistribution()
							.getMostLikely().getProposition();
				}

				String message = "the " + objectIdent + " is in the " + room
						+ ", at place " + placeID;
				
//				JOptionPane.showMessageDialog(null, message);
				
				//HACK  for direct verbalisation of object location
				
				String ttsID = getComponent().newDataID();
				SpokenOutputItem tts = new SpokenOutputItem(ttsID, message, null, null);
				getComponent().addToWorkingMemory(ttsID, "dialogue", tts);
				
				//END HACK
				
				
				((DialogueActionInterface) getComponent()).addBooleanFeature(
						getAction().beliefAddress, "position-reported", true);
				
				result = TriBool.TRITRUE;

				

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	@Override
	protected void start() {
		super.start();

		if (m_fakeIt) {
			m_actionStateManager.registerActionType(ReportPosition.class,
					new ComponentActionFactory<DirectReportPosition>(this,
							DirectReportPosition.class));
		
		} else {
			m_actionStateManager.registerActionType(ReportPosition.class,
					new ComponentActionFactory<ReportPositionDialogue>(this,
							ReportPositionDialogue.class));
		}
	}

}
