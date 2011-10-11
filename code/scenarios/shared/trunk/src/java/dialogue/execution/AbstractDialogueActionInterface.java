package dialogue.execution;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;

import javax.swing.JOptionPane;

import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.components.AbstractActionInterface;
import execution.slice.Action;
import execution.slice.ConfidenceLevel;
import execution.slice.TriBool;
import execution.slice.actions.AnswerOpenQuestion;
import execution.slice.actions.AnswerPolarQuestion;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForObjectWithFeatureValue;
import execution.slice.actions.AskForShape;
import execution.slice.actions.AskPolarColour;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskPolarShape;
import execution.slice.actions.BeliefPlusFeatureValueAction;
import execution.slice.actions.BeliefPlusStringAction;
import execution.slice.actions.SingleBeliefAction;
import execution.slice.actions.VerifyReference;
import execution.util.BlockingActionExecutor;
import execution.util.ComponentActionFactory;
import execution.util.LocalActionStateManager;

/**
 * Receives actions from the execution system and interfaces with the rest of
 * the dialogue system.
 * 
 * @author nah
 * 
 */
public abstract class AbstractDialogueActionInterface extends
		AbstractActionInterface {

	protected static final String INTENTION_TYPE_KEY = "type";

	// private GroundedToSharedBeliefMap m_groundedToShared;

	boolean madeup;

	protected boolean m_fakeIt;

	public abstract static class IntentionDialogueAction<T extends Action>
			extends BlockingActionExecutor<T> {

		private static final int DLG_TIMEOUT = 20;
		WMEventQueue eventQueue = new WMEventQueue();

		// the tribool to return on a timeout
		private final TriBool m_timeoutResponse;

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls, TriBool _timeoutResponse) {
			super(_component, _cls);
			m_timeoutResponse = _timeoutResponse;
		}

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			this(_component, _cls, TriBool.TRIFALSE);
		}

		@Override
		public TriBool execute() {
			log("IntentionDialogueAction.execute()");

			IntentionToAct actint = new IntentionToAct(
					new HashMap<String, String>(),
					new HashMap<String, WorkingMemoryAddress>());

			((AbstractDialogueActionInterface) getComponent()).enableASR();

			addStringContent(actint.stringContent);
			addAddressContent(actint.addressContent);
			TriBool res = TriBool.TRIFALSE;
			try {
				WorkingMemoryAddress id = newWorkingMemoryAddress();
				prepareCheckAndResponse(id);
				getComponent().addToWorkingMemory(id, actint);
				res = waitAndCheckResponse(id);
				actionComplete();
			} catch (CASTException e) {
				logException(e);
			} finally {
				((AbstractDialogueActionInterface) getComponent()).disableASR();
			}
			return res;
		}

		@Override
		public void stopExecution() {
			// TODO Auto-generated method stub

		}

		/**
		 * Hack to provide the same interface as the non-blocking version.
		 */
		protected void actionComplete() {
		}

		protected void prepareCheckAndResponse(WorkingMemoryAddress id) {
			getComponent().addChangeFilter(
					ChangeFilterFactory.createTypeFilter(
							PossibleInterpretedIntentions.class,
							WorkingMemoryOperation.ADD), eventQueue);
		}

		protected TriBool waitAndCheckResponse(WorkingMemoryAddress id) {
			try {
				boolean gotAnswer = false;
				// if we received an update we're happy
				println("wait for the intention of the human to pop up for "
						+ DLG_TIMEOUT + " seconds");
				while (!gotAnswer) {
					WorkingMemoryChange e = eventQueue.poll(DLG_TIMEOUT,
							TimeUnit.SECONDS);
					if (e == null) {
						println("didn't get the intention in time... action failed");
						return m_timeoutResponse;
					} else {
						println("got human intention... check if it is the right one");
						PossibleInterpretedIntentions possInterpretedIntentions = getComponent()
								.getMemoryEntry(e.address,
										PossibleInterpretedIntentions.class);
						InterpretedIntention bestIntention = null;
						WorkingMemoryAddress bestIntentionWMA = null;
						float mostLikely = -1.0f;
						// find most likely intention
						for (Entry<WorkingMemoryAddress, InterpretedIntention> i : possInterpretedIntentions.intentions
								.entrySet()) {
							if (i.getValue().confidence > mostLikely) {
								bestIntention = i.getValue();
								mostLikely = i.getValue().confidence;
								bestIntentionWMA = i.getKey();
							}
						}
						if (bestIntention == null) {
							getComponent()
									.getLogger()
									.warn(
											"no best intention found, we wait further...");
							continue;
						}
						WorkingMemoryAddress correspAddress = bestIntention.addressContent
								.get("answer-to");
						if (correspAddress == null) {
							getComponent()
									.getLogger()
									.warn(
											"this InterpretedIntention was not an answer to anything, hence, we wait further...");
							continue;
						}
						println("check if the received InterpretedIntention matches the one we are waiting for");
						if (correspAddress.equals(id)) {
							println("yes, it matched!");
							if (checkResponse(bestIntention) == TriBool.TRITRUE) {
								submitCorrespondingBeliefs(
										possInterpretedIntentions,
										bestIntention);
								getComponent().addToWorkingMemory(
										bestIntentionWMA, bestIntention);
								return TriBool.TRITRUE;
							} else {
								return TriBool.TRIFALSE;
							}
						}

					}
				}
			} catch (InterruptedException e) {
				logException(e);
			} catch (CASTException e) {
				logException(e);
			} finally {
				try {
					getComponent().removeChangeFilter(eventQueue);
				} catch (SubarchitectureComponentException e) {
					logException(e);
				}
			}
			return TriBool.TRITRUE;
		}

		private void submitCorrespondingBeliefs(
				PossibleInterpretedIntentions possInterpretedIntentions,
				InterpretedIntention bestIntention)
				throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
				UnknownSubarchitectureException {
			for (WorkingMemoryAddress wma : bestIntention.addressContent
					.values()) {
				dBelief belief = possInterpretedIntentions.beliefs.get(wma);
				if (belief != null) {
					debug("found a matching belief for this intention");
					getComponent().addToWorkingMemory(wma, belief);
				}
			}
		}

		/**
		 * Called when the matching response to an intention is received. The
		 * return result is passed back to the planner as the result of the
		 * action.
		 * 
		 * @param interpretedIntention
		 * @return
		 */
		protected TriBool checkResponse(
				InterpretedIntention interpretedIntention) {
			return TriBool.TRITRUE;
		}

		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {

		}

		protected void addStringContent(Map<String, String> _stringContent) {

		}

	}

	/**
	 * Generates an intention with the belief from the action in the "about"
	 * address content.
	 * 
	 * @author nah
	 * 
	 * @param <T>
	 */
	public abstract static class BeliefIntentionDialogueAction<T extends SingleBeliefAction>
			extends IntentionDialogueAction<T> {

		public BeliefIntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			super(_component, _cls);
		}

		public BeliefIntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls, TriBool _timeoutResponse) {
			super(_component, _cls, _timeoutResponse);
		}

		@Override
		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			_addressContent.put("about", getAction().beliefAddress);
		}

	}

	public abstract static class FeatureValueQuestionAnswer<T extends BeliefPlusFeatureValueAction>
			extends BeliefIntentionDialogueAction<T> {

		public FeatureValueQuestionAnswer(ManagedComponent _component,
				Class<T> _cls) {
			super(_component, _cls);
		}

		public FeatureValueQuestionAnswer(ManagedComponent _component,
				Class<T> _cls, TriBool _timeoutResponse) {
			super(_component, _cls, _timeoutResponse);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			// stringContent:
			// "type" -> "assertion"
			// "subtype" -> "answer" | "inform"
			// "feature" -> "color" | "shape" | "type"
			// "value" -> value of the feature
			//
			// addressContent:
			// "about" -> the WM address of the object in question
			// "answer-to" -> the WM address of the question's
			// InterpretedIntention (only when subtype=answer)
			//

			// ignoring answer/answer-to for now

			_stringContent.put(INTENTION_TYPE_KEY, "assertion");
			_stringContent.put("subtype", "inform");
			_stringContent.put("assertion-feature", getAction().feature);
			_stringContent.put("assertion-value", getAction().value);
		}

	}

	/**
	 * 
	 * Ask for a feature of the object in question
	 * 
	 * @author nah
	 * 
	 * @param <T>
	 */
	public abstract static class OpenFeatureQuestion<T extends SingleBeliefAction>
			extends BeliefIntentionDialogueAction<T> {

		private final String m_feature;

		public OpenFeatureQuestion(ManagedComponent _component, Class<T> _cls,
				String _feature) {
			super(_component, _cls);
			m_feature = _feature;
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			_stringContent.put(INTENTION_TYPE_KEY, "question");
			_stringContent.put("subtype", "open");
			_stringContent.put("feature", m_feature);
		}

	}

	public abstract static class PolarFeatureQuestion<T extends BeliefPlusStringAction>
			extends OpenFeatureQuestion<T> {

		public PolarFeatureQuestion(ManagedComponent _component, Class<T> _cls,
				String _feature) {
			super(_component, _cls, _feature);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			_stringContent.put("hypothesis", getAction().value);
			_stringContent.put("subtype", "polar");
		}

	}

	//
	// @Deprecated
	// public static class AskForObjectWithFeatureValueDialogue extends
	// IntentionDialogueAction<AskForObjectWithFeatureValue> {
	//
	// public AskForObjectWithFeatureValueDialogue(ManagedComponent _component)
	// {
	// super(_component, AskForObjectWithFeatureValue.class);
	// }
	//
	// @Override
	// ArrayList<dFormula> getPostconditions() {
	// ArrayList<dFormula> postconditions = super.getPostconditions();
	// postconditions.add(PropositionFormula.create("object-shown").get());
	// postconditions.add(new ModalFormula(-1, "feature",
	// PropositionFormula.create(getAction().feature).get()));
	// postconditions.add(new ModalFormula(-1, "hypo", PropositionFormula
	// .create(getAction().value).get()));
	// return postconditions;
	// }
	//
	// }
	//
	public static class AskForColourValueDialogue extends
			OpenFeatureQuestion<AskForColour> {
		public AskForColourValueDialogue(ManagedComponent _component) {
			super(_component, AskForColour.class, "color");
		}
	}

	public static class AskForShapeValueDialogue extends
			OpenFeatureQuestion<AskForShape> {
		public AskForShapeValueDialogue(ManagedComponent _component) {
			super(_component, AskForShape.class, "shape");
		}
	}

	public static class AskForIdentityValueDialogue extends
			OpenFeatureQuestion<AskForIdentity> {
		public AskForIdentityValueDialogue(ManagedComponent _component) {
			super(_component, AskForIdentity.class, "identity");
		}
	}

	public static class AskForColourPolarDialogue extends
			PolarFeatureQuestion<AskPolarColour> {
		public AskForColourPolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarColour.class, "color");
		}
	}

	public static class AskForShapePolarDialogue extends
			PolarFeatureQuestion<AskPolarShape> {
		public AskForShapePolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarShape.class, "shape");
		}
	}

	public static class VerifyReferenceExecutor extends
			BeliefIntentionDialogueAction<VerifyReference> {

		public VerifyReferenceExecutor(ManagedComponent _component) {
			super(_component, VerifyReference.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);

			// stringContent:
			// "type" -> "question"
			// "subtype" -> "verification"
			//

			_stringContent.put(INTENTION_TYPE_KEY, "question");
			_stringContent.put("subtype", "verification");
		}

		@Override
		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			super.addAddressContent(_addressContent);

			// addressContent:
			// "about" -> the WMA of the GroundedBelief you want to verify
			// "verification-of" -> the WMA of the intention the GroundedBelief
			// is
			// pointed from (the interpretation we'd like to verify)

			// TODO start here, working out how to keep track of verification-of
			// pointer across multiple calls of the executor
			// _addressContent.put("verification-of", ?);
		}

		@Override
		protected TriBool checkResponse(
				InterpretedIntention interpretedIntention) {
			// TODO Handle matched response looking like this:
			// stringContent:
			// type -> "verification"
			// polarity -> "pos" | "neg"
			//
			// addressContent:
			// "answer-to" -> the IntentionToAct asking "do you mean this one"
			// "about" -> the "this one" in the IntentionToAct
			// "verification-of" -> same as above

			// case: pos - return true, and don't touch the belief
			// case neg - return true, but remove the
			// "is-potential-object-in-question" feature

			return super.checkResponse(interpretedIntention);
		}

	}

	public static class AnswerOpenQuestionExecutor extends
			FeatureValueQuestionAnswer<AnswerOpenQuestion> {

		public AnswerOpenQuestionExecutor(ManagedComponent _component) {
			super(_component, AnswerOpenQuestion.class, TriBool.TRITRUE);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);

			ConfidenceLevel confidence = getAction().confidence;
			if (confidence == ConfidenceLevel.CONFIDENT) {
				_stringContent.put("certainty", "high");
			} else if (confidence == ConfidenceLevel.UNSURE) {
				_stringContent.put("certainty", "low");
			} else {
				_stringContent.put("subtype", "answer-unknown");
			}
		}

		@Override
		protected void actionComplete() {
			try {
				// global-type-question-answered
				String answeredPredictate = "global-" + getAction().feature
						+ "-question-answered";
				// record that we have looked at it

				((AbstractActionInterface) getComponent()).addBooleanFeature(
						getAction().beliefAddress, answeredPredictate, true);
			} catch (CASTException e) {
				getComponent().logException(e);
			}
		}

	}

	public static class AnswerPolarQuestionExecutor extends
			FeatureValueQuestionAnswer<AnswerPolarQuestion> {

		public AnswerPolarQuestionExecutor(ManagedComponent _component) {
			super(_component, AnswerPolarQuestion.class, TriBool.TRITRUE);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);

			ConfidenceLevel confidence = getAction().confidence;
			if (confidence == ConfidenceLevel.CONFIDENT) {
				_stringContent.put("certainty", "high");
			} else if (confidence == ConfidenceLevel.UNSURE) {
				_stringContent.put("certainty", "low");
			} else {
				// high confidence in a negative answer
				_stringContent.put("certainty", "high");

			}
		}

		@Override
		protected void actionComplete() {
			try {
				// global-type-question-answered
				String answeredPredictate = "polar-" + getAction().feature
						+ "-question-answered";
				// record that we have looked at it

				((AbstractActionInterface) getComponent()).addBooleanFeature(
						getAction().beliefAddress, answeredPredictate, true);
			} catch (CASTException e) {
				getComponent().logException(e);
			}
		}

	}

	public static class AskForIdentityPolarDialogue extends
			PolarFeatureQuestion<AskPolarIdentity> {
		public AskForIdentityPolarDialogue(ManagedComponent _component) {
			super(_component, AskPolarIdentity.class, "identity");
		}
	}

	public static class DirectShapeAnswer extends
			BlockingActionExecutor<AskForShape> {

		public DirectShapeAnswer(ManagedComponent _component) {
			super(_component, AskForShape.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {
				((AbstractDialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("shape", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}

	}

	public static class DirectIdentityAnswer extends
			BlockingActionExecutor<AskForIdentity> {

		public DirectIdentityAnswer(ManagedComponent _component) {
			super(_component, AskForIdentity.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {
				((AbstractDialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("identity", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}

	}

	public static class DirectPolarShapeAnswer extends
			BlockingActionExecutor<AskPolarShape> {

		public DirectPolarShapeAnswer(ManagedComponent _component) {
			super(_component, AskPolarShape.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((AbstractDialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("shape", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectPolarColourAnswer extends
			BlockingActionExecutor<AskPolarColour> {

		public DirectPolarColourAnswer(ManagedComponent _component) {
			super(_component, AskPolarColour.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((AbstractDialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("color", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectPolarIdentityAnswer extends
			BlockingActionExecutor<AskPolarIdentity> {

		public DirectPolarIdentityAnswer(ManagedComponent _component) {
			super(_component, AskPolarIdentity.class);
		}

		@Override
		public TriBool execute() {

			TriBool result = TriBool.TRIFALSE;
			try {

				((AbstractDialogueActionInterface) getComponent())
						.askPolarFeatureThenSetDirect("identity", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;

		}
	}

	public static class DirectAskForObject extends
			BlockingActionExecutor<AskForObjectWithFeatureValue> {
		public DirectAskForObject(ManagedComponent _component) {
			super(_component, AskForObjectWithFeatureValue.class);
		}

		@Override
		public TriBool execute() {
			JOptionPane.showMessageDialog(null,
					"OI YOU! Add a new object that looks " + getAction().value
							+ " DO AS I SAY! .... please xxx");
			return TriBool.TRITRUE;
		}
	}

	public static class DirectColourAnswer extends
			BlockingActionExecutor<AskForColour> {

		public DirectColourAnswer(ManagedComponent _component) {
			super(_component, AskForColour.class);
		}

		@Override
		public TriBool execute() {
			TriBool result = TriBool.TRIFALSE;
			try {
				((AbstractDialogueActionInterface) getComponent())
						.askForFeatureThenSetDirect("color", getAction());
				result = TriBool.TRITRUE;

			} catch (CASTException e) {
				logException(e);
			}

			return result;
		}

	}

	private void addFeatureDirectly(SingleBeliefAction _action,
			String _feature, String _value, double _prob)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		GroundedBelief belief = getMemoryEntry(_action.beliefAddress,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, _prob);
		pb.getContent().put(_feature, fd);
		overwriteWorkingMemory(_action.beliefAddress, pb.get());
	}

	/**
	 * called when the robot should STOP listening
	 */
	public void disableASR() {
		println("disbaling ASR not implemented in "
				+ AbstractDialogueActionInterface.class.toString());

	}

	/**
	 * called when the robot should listen
	 */
	public void enableASR() {
		println("enbaling ASR not implemented in "
				+ AbstractDialogueActionInterface.class.toString());
	}

	private static String askForFeatureValue(String _feature) {
		return (String) JOptionPane.showInputDialog(null, "What " + _feature
				+ " is the object?");
	}

	private static boolean askPolarFeatureValue(String _feature, String _value) {
		String answer = (String) JOptionPane.showInputDialog(null, "Is the "
				+ _feature + " " + _value + "?");
		if (answer.toLowerCase().startsWith("y")) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private void askForFeatureThenSetDirect(String _feature,
			SingleBeliefAction _action) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		addFeatureDirectly(_action, _feature, askForFeatureValue(_feature), 1d);
	}

	/**
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private boolean askPolarFeatureThenSetDirect(String _feature,
			BeliefPlusStringAction _action) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		if (askPolarFeatureValue(_feature, _action.value)) {
			addFeatureDirectly(_action, _feature, _action.value, 1d);
			return true;
		} else {
			addFeatureDirectly(_action, _feature, _action.value, 0d);
		}
		return false;
	}

	@Override
	protected void start() {

		m_actionStateManager = new LocalActionStateManager(this);

		// DoNothingActionExecutorFactory derelictFactory = new
		// DoNothingActionExecutorFactory(
		// this);

		if (m_fakeIt) {
			m_actionStateManager
					.registerActionType(
							AskForColour.class,
							new ComponentActionFactory<AskForColour, DirectColourAnswer>(
									this, DirectColourAnswer.class));

			m_actionStateManager.registerActionType(AskForShape.class,
					new ComponentActionFactory<AskForShape, DirectShapeAnswer>(
							this, DirectShapeAnswer.class));

			m_actionStateManager
					.registerActionType(
							AskForIdentity.class,
							new ComponentActionFactory<AskForIdentity, DirectIdentityAnswer>(
									this, DirectIdentityAnswer.class));

			m_actionStateManager
					.registerActionType(
							AskPolarColour.class,
							new ComponentActionFactory<AskPolarColour, DirectPolarColourAnswer>(
									this, DirectPolarColourAnswer.class));

			m_actionStateManager
					.registerActionType(
							AskPolarShape.class,
							new ComponentActionFactory<AskPolarShape, DirectPolarShapeAnswer>(
									this, DirectPolarShapeAnswer.class));

			m_actionStateManager
					.registerActionType(
							AskPolarIdentity.class,
							new ComponentActionFactory<AskPolarIdentity, DirectPolarIdentityAnswer>(
									this, DirectPolarIdentityAnswer.class));

			m_actionStateManager
					.registerActionType(
							AskForObjectWithFeatureValue.class,
							new ComponentActionFactory<AskForObjectWithFeatureValue, DirectAskForObject>(
									this, DirectAskForObject.class));

		} else {

			m_actionStateManager
					.registerActionType(
							AskForColour.class,
							new ComponentActionFactory<AskForColour, AskForColourValueDialogue>(
									this, AskForColourValueDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskForShape.class,
							new ComponentActionFactory<AskForShape, AskForShapeValueDialogue>(
									this, AskForShapeValueDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskForIdentity.class,
							new ComponentActionFactory<AskForIdentity, AskForIdentityValueDialogue>(
									this, AskForIdentityValueDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskPolarColour.class,
							new ComponentActionFactory<AskPolarColour, AskForColourPolarDialogue>(
									this, AskForColourPolarDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskPolarShape.class,
							new ComponentActionFactory<AskPolarShape, AskForShapePolarDialogue>(
									this, AskForShapePolarDialogue.class));

			m_actionStateManager
					.registerActionType(
							AskPolarIdentity.class,
							new ComponentActionFactory<AskPolarIdentity, AskForIdentityPolarDialogue>(
									this, AskForIdentityPolarDialogue.class));

			m_actionStateManager
					.registerActionType(
							AnswerOpenQuestion.class,
							new ComponentActionFactory<AnswerOpenQuestion, AnswerOpenQuestionExecutor>(
									this, AnswerOpenQuestionExecutor.class));

			m_actionStateManager
					.registerActionType(
							AnswerPolarQuestion.class,
							new ComponentActionFactory<AnswerPolarQuestion, AnswerPolarQuestionExecutor>(
									this, AnswerPolarQuestionExecutor.class));

			// m_actionStateManager
			// .registerActionType(
			// AskForObjectWithFeatureValue.class,
			// new ComponentActionFactory<AskForObjectWithFeatureValueDialogue>(
			// this,
			// AskForObjectWithFeatureValueDialogue.class));

			// // TODO replace this with one of Marc's magic thingys
			// WorkingMemoryChangeReceiver updater = new
			// WorkingMemoryChangeReceiver() {
			//
			// @Override
			// public void workingMemoryChanged(WorkingMemoryChange _wmc)
			// throws CASTException {
			// m_groundedToShared = getMemoryEntry(_wmc.address,
			// GroundedToSharedBeliefMap.class);
			// }
			// };

			// // look for the map between grounded and shared beliefs
			// addChangeFilter(
			// ChangeFilterFactory.createGlobalTypeFilter(
			// GroundedToSharedBeliefMap.class,
			// WorkingMemoryOperation.ADD), updater);
			// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
			// GroundedToSharedBeliefMap.class,
			// WorkingMemoryOperation.OVERWRITE), updater);

		}

	}

	@Override
	protected void configure(Map<String, String> _config) {
		m_fakeIt = _config.containsKey("--fake-it");
	}

	// private WorkingMemoryAddress getSharedBeliefAddress(
	// WorkingMemoryAddress _groundedBeliefAddress) {
	// WorkingMemoryAddress addr = null;
	// if (m_groundedToShared != null) {
	// addr = m_groundedToShared.map.get(_groundedBeliefAddress);
	// } else {
	// println("trying to get shared belief address before map has been generated");
	// }
	// return addr;
	// }

}
