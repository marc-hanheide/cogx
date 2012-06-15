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
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.intentions.inst.DefCategoryLabelQuestionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.DefLabelRelationLabelQuestionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.HypothesisGenerationVerificationQuestionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.LabelQuestionIntention;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.HypotheticalBelief;
import eu.cogx.perceptmediator.components.RoomMembershipMediator;
import eu.cogx.perceptmediator.dora.VisualObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ConnectivityTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import execution.slice.TriBool;
import execution.slice.actions.AskForBKLabelInCategory;
import execution.slice.actions.AskForBKLabelRelLabel;
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
		AbstractDialogueActionInterface<dBelief> {

	WMView<StandbyMode> standbyModeView = WMView
			.create(this, StandbyMode.class);

	public DialogueActionInterface() {
		super(dBelief.class);
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
			_addressContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.PKEY_ABOUT, getAction().beliefAddress);
			_addressContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.PKEY_RELATED_ENTITY ,getAction().pointer);
			logAddressContent(_addressContent);
		}
	
		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			_stringContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.SKEY_TYPE, "question");
			_stringContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.SKEY_SUBTYPE, "polar");
			_stringContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.SKEY_SUBSUBTYPE, "hgv");
	
			_stringContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.SKEY_LABEL, getAction().label);
			_stringContent.put(HypothesisGenerationVerificationQuestionIntention.Transcoder.INSTANCE.SKEY_RELATION, getAction().relation);
			logStringContent(_stringContent);
		}
	
		@Override
		protected TriBool checkResponse(InterpretedIntention _ii)
				throws SubarchitectureComponentException {
			String value = "false";
			if (_ii.stringContent.get("asserted-polarity").equals("pos"))
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

	/**
	 * Class to represent the first UGV dialogue action to ask 
	 * see https://codex.cs.bham.ac.uk/trac/cogx/ticket/516
	 * 
	 * @author marc
	 * 
	 */
	public static class AskForBKLabelInCategoryAction extends
			IntentionDialogueAction<AskForBKLabelInCategory> {
		public static final String LABEL_ASSERTED_POLARITY = "asserted-polarity";
		public static final String REPLY_POS = "pos";
		public static final String LABEL_DORA_INROOM = "dora__inroom";
		public static final double PROB_BK_LABEL_IN_CAT_POS = 0.75;
		public static final double PROB_BK_LABEL_IN_CAT_NEG = 0.05;

		public AskForBKLabelInCategoryAction(ManagedComponent _component) {
			super(_component, AskForBKLabelInCategory.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			_stringContent.put(DefCategoryLabelQuestionIntention.Transcoder.INSTANCE.SKEY_TYPE, "question");
			_stringContent.put(DefCategoryLabelQuestionIntention.Transcoder.INSTANCE.SKEY_SUBTYPE, "polar");
			_stringContent.put(DefCategoryLabelQuestionIntention.Transcoder.INSTANCE.SKEY_SUBSUBTYPE, "ubk-category");

			_stringContent.put(LabelQuestionIntention.Transcoder.INSTANCE.SKEY_LABEL, getAction().label);
			_stringContent.put(DefCategoryLabelQuestionIntention.Transcoder.INSTANCE.SKEY_CATEGORY, getAction().category);
			logStringContent(_stringContent);
		}

		@Override
		protected TriBool checkResponse(InterpretedIntention _ii)
				throws SubarchitectureComponentException {
			CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> bel = CASTIndependentFormulaDistributionsBelief.create(HypotheticalBelief.class);
			bel.setType(ConnectivityTransferFunction.LABEL_RELATION);
			// fill belief
			FormulaDistribution fd=FormulaDistribution.create();
			fd.add(getAction().label, 1.0);
			bel.getContent().put(ConnectivityTransferFunction.LABEL_VAL0, fd);
			
			fd=FormulaDistribution.create();
			fd.add(getAction().category, 1.0);
			bel.getContent().put(ConnectivityTransferFunction.LABEL_VAL1, fd);
			
			fd=FormulaDistribution.create();
			double prob=PROB_BK_LABEL_IN_CAT_NEG;
			if (_ii.stringContent.get(LABEL_ASSERTED_POLARITY).equals(REPLY_POS))
				prob=PROB_BK_LABEL_IN_CAT_POS;
			fd.add(true, prob);
			bel.getContent().put(LABEL_DORA_INROOM, fd);

			String id = getComponent().newDataID();
			bel.setId(id);

			getComponent().println("belief to be submitted: "+bel.toString());
			try {
				getComponent().addToWorkingMemory(id, bel.get());
			} catch (CASTException e) {
				logException(e);
			}
			return TriBool.TRITRUE;
		}

	}

	/**
	 * Class to represent the first UGV dialogue action to ask 
	 * see https://codex.cs.bham.ac.uk/trac/cogx/ticket/516
	 * 
	 * @author marc
	 * 
	 */
	public static class AskForBKLabelRelLabelAction extends
			IntentionDialogueAction<AskForBKLabelRelLabel> {
		public static final String LABEL_VAL2 = "val2";
		public static final String LABEL_ASSERTED_POLARITY = "asserted-polarity";
		public static final String REPLY_POS = "pos";
		public static final String LABEL_RELATED = "dora__related";
		public static final double PROB_BK_LABEL_RELATION_POS = 0.75;
		public static final double PROB_BK_LABEL_RELATION_NEG = 0.05;

		public AskForBKLabelRelLabelAction(ManagedComponent _component) {
			super(_component, AskForBKLabelRelLabel.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			_stringContent.put(DefLabelRelationLabelQuestionIntention.Transcoder.INSTANCE.SKEY_TYPE, "question");
			_stringContent.put(DefLabelRelationLabelQuestionIntention.Transcoder.INSTANCE.SKEY_SUBTYPE, "polar");
			_stringContent.put(DefLabelRelationLabelQuestionIntention.Transcoder.INSTANCE.SKEY_SUBSUBTYPE, "ubk-label");

			_stringContent.put(LabelQuestionIntention.Transcoder.INSTANCE.SKEY_LABEL, getAction().label);
			_stringContent.put(DefLabelRelationLabelQuestionIntention.Transcoder.INSTANCE.SKEY_OTHERLABEL, getAction().otherLabel);
			_stringContent.put(DefLabelRelationLabelQuestionIntention.Transcoder.INSTANCE.SKEY_RELATION, getAction().relation);
			logStringContent(_stringContent);
		}

		@Override
		protected TriBool checkResponse(InterpretedIntention _ii)
				throws SubarchitectureComponentException {
			CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> bel = CASTIndependentFormulaDistributionsBelief.create(HypotheticalBelief.class);
			bel.setType(ConnectivityTransferFunction.LABEL_RELATION);
			// fill belief
			FormulaDistribution fd=FormulaDistribution.create();
			fd.add(getAction().label, 1.0);
			bel.getContent().put(ConnectivityTransferFunction.LABEL_VAL0, fd);
			
			fd=FormulaDistribution.create();
			fd.add(getAction().relation, 1.0);
			bel.getContent().put(ConnectivityTransferFunction.LABEL_VAL1, fd);
			
			fd=FormulaDistribution.create();
			fd.add(getAction().otherLabel, 1.0);
			bel.getContent().put(LABEL_VAL2, fd);
			
			fd=FormulaDistribution.create();
			double prob=PROB_BK_LABEL_RELATION_NEG;
			if (_ii.stringContent.get(LABEL_ASSERTED_POLARITY).equals(REPLY_POS))
				prob=PROB_BK_LABEL_RELATION_POS;
			fd.add(true, prob);
			bel.getContent().put(LABEL_RELATED, fd);

			String id = getComponent().newDataID();
			bel.setId(id);

			getComponent().println("belief to be submitted: "+bel.toString());
			try {
				getComponent().addToWorkingMemory(id, bel.get());
			} catch (CASTException e) {
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
			m_actionStateManager
			.registerActionType(
					AskForBKLabelInCategory.class,
					new ComponentActionFactory<AskForBKLabelInCategory, AskForBKLabelInCategoryAction>(
							this, AskForBKLabelInCategoryAction.class));
			m_actionStateManager
			.registerActionType(
					AskForBKLabelRelLabel.class,
					new ComponentActionFactory<AskForBKLabelRelLabel, AskForBKLabelRelLabelAction>(
							this, AskForBKLabelRelLabelAction.class));

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
