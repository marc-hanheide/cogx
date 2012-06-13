package dialogue.execution.dora;

import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.slice.TriBool;
import execution.slice.actions.AskForLabelExistence;
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
public class DialogueActionInterface extends
		AbstractDialogueActionInterface<GroundedBelief> {

	WMView<StandbyMode> standbyModeView = WMView
			.create(this, StandbyMode.class);

	public DialogueActionInterface() {
		super(GroundedBelief.class);
	}

	/**
	 * Class to represent the HGV dialogue action to ask if an object exists.
	 * see https://codex.cs.bham.ac.uk/trac/cogx/ticket/516
	 * 
	 * @author marc
	 * 
	 */
	public static class AskForLabelExistenceDialogue extends
			BeliefIntentionDialogueAction<AskForLabelExistence> {
		public AskForLabelExistenceDialogue(ManagedComponent _component) {
			super(_component, AskForLabelExistence.class);
		}

		@Override
		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			super.addAddressContent(_addressContent);
			_addressContent.put("about", getAction().beliefAddress);
			_addressContent.put("related-to" ,getAction().pointer);
			getComponent().println(
					AskForLabelExistenceDialogue.class.getSimpleName()
							+ ": created address content for intention");
			for (Entry<String, WorkingMemoryAddress> e : _addressContent
					.entrySet()) {
				getComponent().println(
						"  " + e.getKey() + " => "
								+ CASTUtils.toString(e.getValue()));
			}

		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			// overwrite it to be a polar question
			_stringContent.put("subtype", "polar");
			// TODO: modify according to Mira's instructions
			_stringContent.put("hypothesis", "exists");
			_stringContent.put("label", getAction().label);
			_stringContent.put("relation", getAction().relation);
			getComponent().println(
					AskForLabelExistenceDialogue.class.getSimpleName()
							+ ": created string content for intention");
			for (Entry<String, String> e : _stringContent.entrySet()) {
				getComponent().println(
						"  " + e.getKey() + " => " + e.getValue());
			}
		}

		@Override
		protected TriBool checkResponse(InterpretedIntention _ii)
				throws SubarchitectureComponentException {
			getComponent().println(
					AskForLabelExistenceDialogue.class.getSimpleName()
							+ ": check response:");
			for (Entry<String, String> e : _ii.stringContent.entrySet()) {
				getComponent().println(
						"  " + e.getKey() + " => " + e.getValue());
			}

			// TODO check if it was either "yes" or "no"
			String value = "false";
			if (_ii.stringContent.get("exists").equals("yes"))
				value = "true";

			try {

				((AbstractDialogueActionInterface<?>) getComponent())
						.addFeature(getAction().beliefAddress, "entity-exists",
								value);

			} catch (SubarchitectureComponentException e) {
				logException(e);
			}
			return TriBool.TRITRUE;
		}

	}

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
		protected void actionSuccessfullyCompleted() {
			try {
				((DialogueActionInterface) getComponent()).addFeature(
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

				((DialogueActionInterface) getComponent()).addFeature(
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

		try {
			standbyModeView.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}

		if (m_fakeIt) {
			m_actionStateManager
					.registerActionType(
							ReportPosition.class,
							new ComponentActionFactory<ReportPosition, DirectReportPosition>(
									this, DirectReportPosition.class));

		} else {
			m_actionStateManager
					.registerActionType(
							ReportPosition.class,
							new ComponentActionFactory<ReportPosition, ReportPositionDialogue>(
									this, ReportPositionDialogue.class));

			// TODO: ticket #296

			m_actionStateManager
					.registerActionType(
							EngageWithHuman.class,
							new ComponentActionFactory<EngageWithHuman, HumanEngagementExecutor>(
									this, HumanEngagementExecutor.class));

			m_actionStateManager
					.registerActionType(
							AskForLabelExistence.class,
							new ComponentActionFactory<AskForLabelExistence, AskForLabelExistenceDialogue>(
									this, AskForLabelExistenceDialogue.class));

		}
	}

	@Override
	protected void runComponent() {
		// make the robot deaf initially
		disableASR();
		super.runComponent();
	}

	@Override
	public void disableASR() {
		StandbyMode sbm = new StandbyMode();
		try {
			addToWorkingMemory(newDataID(), sbm);
		} catch (AlreadyExistsOnWMException e) {
			logException(e);
		}
		println("disabled ASR processing... robot is deaf now");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see dialogue.execution.AbstractDialogueActionInterface#enableASR()
	 */
	@Override
	public void enableASR() {
		Set<WorkingMemoryAddress> wmaSets = new HashSet<WorkingMemoryAddress>(
				standbyModeView.keySet());
		try {
			for (WorkingMemoryAddress wma : wmaSets) {
				deleteFromWorkingMemory(wma);
			}
			println("enabled ASR processing... robot is listening now");
		} catch (CASTException e) {
			logException(e);
		}

	}
}
