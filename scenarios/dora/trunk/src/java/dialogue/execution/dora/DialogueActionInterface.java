package dialogue.execution.dora;

import java.util.Map;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.slice.TriBool;
import execution.slice.actions.EngageWithHuman;
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
public class DialogueActionInterface extends AbstractDialogueActionInterface {

	public static class ReportPositionDialogue extends
			BeliefIntentionDialogueAction<ReportPosition> {

		public ReportPositionDialogue(ManagedComponent _component) {
			super(_component, ReportPosition.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			_stringContent.put(INTENTION_TYPE_KEY, "assertion");
			_stringContent.put("subtype", "location-report");
		}

		@Override
		protected void actionComplete() {
			try {
				((DialogueActionInterface) getComponent()).addBooleanFeature(
						getAction().beliefAddress, "position-reported", true);
			} catch (CASTException e) {
				logException(e);
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

				// JOptionPane.showMessageDialog(null, message);

				// HACK for direct verbalisation of object location

				String ttsID = getComponent().newDataID();
				SpokenOutputItem tts = new SpokenOutputItem(ttsID, message, "",
						null);
				getComponent().addToWorkingMemory(ttsID, "dialogue", tts);

				// END HACK

				((DialogueActionInterface) getComponent()).addBooleanFeature(
						getAction().beliefAddress, "position-reported", true);

				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	/**
	 * Executor for generating engagement and disengagement dialogues (assuming
	 * I can do both in one struct). First pass just writes out intention then
	 * returns succcess.
	 */
	public static class HumanEngagementExecutor extends
			IntentionDialogueAction<EngageWithHuman> {

		public HumanEngagementExecutor(ManagedComponent _component) {
			super(_component, EngageWithHuman.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {

			if (getAction().disengage) {
				_stringContent.put(INTENTION_TYPE_KEY, "engagement-closing");
			} else {
				_stringContent.put(INTENTION_TYPE_KEY, "engagement-opening");
			}
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

			// TODO: ticket #296

			m_actionStateManager.registerActionType(EngageWithHuman.class,
					new ComponentActionFactory<HumanEngagementExecutor>(this,
							HumanEngagementExecutor.class));

		}
	}
}
