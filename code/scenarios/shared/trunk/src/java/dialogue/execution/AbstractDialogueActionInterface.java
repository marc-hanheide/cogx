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
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.util.VerbalisationUtils;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.RichIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.FeatureAscriptionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.OpenFeatureQuestionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.PolarFeatureQuestionIntention;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
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
import execution.slice.actions.VerifyReferenceByFeatureValue;
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
public abstract class AbstractDialogueActionInterface<BeliefType extends dBelief>
		extends AbstractActionInterface<BeliefType> {

	// FIXME: hacky, but this must not be in scenarios/george!
	public static final String IS_POTENTIAL_OBJECT_IN_QUESTION = "is-potential-object-in-question";

	protected static final String INTENTION_TYPE_KEY = "type";

	// private GroundedToSharedBeliefMap m_groundedToShared;

	boolean madeup;

	protected boolean m_fakeIt;

	public AbstractDialogueActionInterface(Class<BeliefType> _beliefCls) {
		super(_beliefCls);
	}

	public abstract static class IntentionDialogueAction<T extends Action>
			extends BlockingActionExecutor<T> {

		private static final int DLG_TIMEOUT = 20;

		private final int m_timeoutSeconds;

		WMEventQueue eventQueue = new WMEventQueue();

		// the tribool to return on a timeout
		private final TriBool m_timeoutResponse;

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls, int _timeoutSeconds, TriBool _timeoutResponse) {
			super(_component, _cls);
			m_timeoutSeconds = _timeoutSeconds;
			m_timeoutResponse = _timeoutResponse;
		}

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			this(_component, _cls, DLG_TIMEOUT, TriBool.TRIFALSE);
		}

		@Override
		public TriBool execute() {
			log("IntentionDialogueAction.execute()");

			IntentionToAct actint = new IntentionToAct(
					new HashMap<String, String>(),
					new HashMap<String, WorkingMemoryAddress>());

			((AbstractDialogueActionInterface<?>) getComponent()).enableASR();

			addStringContent(actint.stringContent);
			addAddressContent(actint.addressContent);
			TriBool res = TriBool.TRIFALSE;
			try {
				WorkingMemoryAddress id = newWorkingMemoryAddress();
				prepareCheckAndResponse(id);
				getComponent().addToWorkingMemory(id, actint);
				res = waitAndCheckResponse(id);
				if (res == TriBool.TRITRUE)
					actionComplete();
			} catch (CASTException e) {
				logException(e);
			} finally {
				((AbstractDialogueActionInterface<?>) getComponent())
						.disableASR();
			}
			return res;
		}

		@Override
		public void stopExecution() {
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
						+ m_timeoutSeconds + " seconds");
				while (!gotAnswer) {
					WorkingMemoryChange e = eventQueue.poll(m_timeoutSeconds,
							TimeUnit.SECONDS);
					if (e == null) {
						println("didn't get the intention in time... action halting");
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
									.warn("no best intention found, we wait further...");
							continue;
						}
						WorkingMemoryAddress correspAddress = bestIntention.addressContent
								.get("answer-to");
						if (correspAddress == null) {
							getComponent()
									.getLogger()
									.warn("this InterpretedIntention was not an answer to anything, hence, we wait further...");
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
								VerbalisationUtils.verbaliseString(
										this.getComponent(), "ok");
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
		 * @throws SubarchitectureComponentException
		 */
		protected TriBool checkResponse(
				InterpretedIntention interpretedIntention)
				throws SubarchitectureComponentException {
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
				Class<T> _cls, int _timeoutSeconds, TriBool _timeoutResponse) {
			super(_component, _cls, _timeoutSeconds, _timeoutResponse);
		}

		@Override
		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			_addressContent.put("about", getAction().beliefAddress);
		}

	}

	public abstract static class FeatureValueQuestionAnswer<T extends BeliefPlusFeatureValueAction>
			extends BeliefIntentionDialogueAction<T> {

		// how long to block the planner after reporting the answer
		private static final int ANSWER_TIMEOUT_SECS = 5;

		public FeatureValueQuestionAnswer(ManagedComponent _component,
				Class<T> _cls) {
			super(_component, _cls, ANSWER_TIMEOUT_SECS, TriBool.TRITRUE);
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
			_stringContent.put("asserted-feature", getAction().feature);
			_stringContent.put("asserted-value", getAction().value);
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

		@Override
		protected void addAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			super.addAddressContent(_addressContent);

			// also flag grounded belief with reference potential
			try {
				((AbstractDialogueActionInterface<?>) getComponent())
						.addFeature(getAction().beliefAddress,
								IS_POTENTIAL_OBJECT_IN_QUESTION, true);
			} catch (SubarchitectureComponentException e) {
				logException(e);
			}
		}

		/**
		 * Input is the matching response
		 * 
		 * @throws UnknownSubarchitectureException
		 * @throws DoesNotExistOnWMException
		 */
		@Override
		protected TriBool checkResponse(InterpretedIntention _ii)
				throws SubarchitectureComponentException {
			// Handle matched response looking like this:
			// [33m[gg.ii: asserted-value -> red][0m
			// [33m[gg.ii: subtype -> answer][0m
			// [33m[gg.ii: asserted-polarity -> pos][0m
			// [33m[gg.ii: asserted-feature -> color][0m
			// [33m[gg.ii: type -> assertion][0m
			// [33m[gg.ii: ][0m

			// TODO what if the answer is a negation?

			String feature = _ii.stringContent.get("asserted-feature");
			String value = _ii.stringContent.get("asserted-value");
			try {

				((AbstractDialogueActionInterface<?>) getComponent())
						.addFeature(getAction().beliefAddress, "attributed-"
								+ feature, value);

			} catch (SubarchitectureComponentException e) {
				logException(e);
			}
			return TriBool.TRITRUE;
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

	public abstract static class AbstractVerifyReferenceExecutor<V extends SingleBeliefAction>
			extends BeliefIntentionDialogueAction<V> {

		public AbstractVerifyReferenceExecutor(ManagedComponent _component,
				Class<V> _verificationClass) {
			super(_component, _verificationClass);
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
			_addressContent
					.put("verification-of",
							((AbstractDialogueActionInterface<?>) getComponent()).m_lastPossibleIntentionsAddition);
		}

		/**
		 * Input is the matching response
		 * 
		 * @throws UnknownSubarchitectureException
		 * @throws DoesNotExistOnWMException
		 */
		@Override
		protected TriBool checkResponse(
				InterpretedIntention interpretedIntention)
				throws SubarchitectureComponentException {
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

			assert (getAction().beliefAddress
					.equals(interpretedIntention.addressContent.get("about")));
			// In the yes case I got:

			String polarity = interpretedIntention.stringContent
					.get("asserted-polarity");

			println("verification polarity: " + polarity);

			if (polarity.equals("pos")) {

				println("positive polarity: ");

				// this is the thing we care about
				// get the list of all potential intentions and thus referents

				WorkingMemoryAddress correctReferentAddr = interpretedIntention.addressContent
						.get("about");

				WorkingMemoryAddress piiAddr = interpretedIntention.addressContent
						.get("verification-of");

				PossibleInterpretedIntentions pii = getComponent()
						.getMemoryEntry(piiAddr,
								PossibleInterpretedIntentions.class);

				println("number of iis in pii found on WM: "
						+ pii.intentions.size());

				for (WorkingMemoryAddress addr : pii.intentions.keySet()) {
					InterpretedIntention iint = pii.intentions.get(addr);
					WorkingMemoryAddress potentialReferentAddr = iint.addressContent
							.get("about");
					if (!potentialReferentAddr.equals(correctReferentAddr)) {
						unmarkReferent(potentialReferentAddr);
					} else {
						// else decode and mark as accepted
						RichIntention decoded = AbstractDialogueActionInterface
								.extractRichIntention(iint);

						if (decoded == null) {
							getComponent().getLogger().warn(
									"Unable to decode intention",
									getComponent().getLogAdditions());

						} else {
							CASTEffect acceptEffect = decoded
									.getOnAcceptEffect();
							acceptEffect.makeItSo(getComponent());
							log("executed accept effect");
						}

					}
				}

			} else {
				// this is the wrong thing

				unmarkReferent(getAction().beliefAddress);

			}
			return TriBool.TRITRUE;

			// <de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention>
			// <stringContent>
			// <entry>
			// <string>subtype</string>
			// <string>verification-answer</string>
			// </entry>
			// <entry>
			// <string>asserted-polarity</string>
			// <string>pos</string>
			// </entry>
			// <entry>
			// <string>type</string>
			// <string>assertion</string>
			// </entry>
			// </stringContent>
			// <addressContent>
			// <entry>
			// <string>answer-to</string>
			// <cast.cdl.WorkingMemoryAddress>
			// <id>0:x</id>
			// <subarchitecture>dialogue</subarchitecture>
			// </cast.cdl.WorkingMemoryAddress>
			// </entry>
			// <entry>
			// <string>verification-of</string>
			// <cast.cdl.WorkingMemoryAddress>
			// <id>1:r</id>
			// <subarchitecture>dialogue</subarchitecture>
			// </cast.cdl.WorkingMemoryAddress>
			// </entry>
			// <entry>
			// <string>about</string>
			// <cast.cdl.WorkingMemoryAddress>
			// <id>1:T</id>
			// <subarchitecture>binder</subarchitecture>
			// </cast.cdl.WorkingMemoryAddress>
			// </entry>
			// </addressContent>
			// <state>READY</state>
			// <agent>human</agent>
			// <confidence>1.0</confidence>
			// </de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention>

		}

		private void unmarkReferent(WorkingMemoryAddress _refGroundBelAddr)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException, ConsistencyException,
				PermissionException {

			println("removing reference marker from belief at "
					+ CASTUtils.toString(_refGroundBelAddr));

			GroundedBelief belief = getComponent().getMemoryEntry(
					_refGroundBelAddr, GroundedBelief.class);

			if (!((AbstractDialogueActionInterface<?>) getComponent())
					.removeQuestionReference(_refGroundBelAddr, belief)) {
				getComponent().getLogger().warn(
						"Verified belief didn't have field"
								+ IS_POTENTIAL_OBJECT_IN_QUESTION,
						getComponent().getLogAdditions());
			}

		}

	}

	public static class VerifyReferenceExecutor extends
			AbstractVerifyReferenceExecutor<VerifyReference> {

		public VerifyReferenceExecutor(ManagedComponent _component) {
			super(_component, VerifyReference.class);
		}

	}

	public static class VerifyByFeatureValueReferenceExecutor extends
			AbstractVerifyReferenceExecutor<VerifyReferenceByFeatureValue> {

		public VerifyByFeatureValueReferenceExecutor(ManagedComponent _component) {
			super(_component, VerifyReferenceByFeatureValue.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);
			_stringContent.put("description", getAction().value);
		}

	}

	public static class AnswerOpenQuestionExecutor extends
			FeatureValueQuestionAnswer<AnswerOpenQuestion> {

		public AnswerOpenQuestionExecutor(ManagedComponent _component) {
			super(_component, AnswerOpenQuestion.class);
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

			// _stringContent.put("polarity","pos");
		}

		@Override
		protected void actionComplete() {
			try {
				// global-type-question-answered
				String answeredPredictate = "global-" + getAction().feature
						+ "-question-answered";
				// record that we have looked at it

				((AbstractActionInterface<?>) getComponent()).addFeature(
						getAction().beliefAddress, answeredPredictate, true);
			} catch (CASTException e) {
				getComponent().logException(e);
			}
		}

	}

	public static class AnswerPolarQuestionExecutor extends
			FeatureValueQuestionAnswer<AnswerPolarQuestion> {

		public AnswerPolarQuestionExecutor(ManagedComponent _component) {
			super(_component, AnswerPolarQuestion.class);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {
			super.addStringContent(_stringContent);

			ConfidenceLevel confidence = getAction().confidence;
			if (confidence == ConfidenceLevel.CONFIDENT) {
				_stringContent.put("certainty", "high");
				_stringContent.put("polarity", "pos");
			} else if (confidence == ConfidenceLevel.UNSURE) {
				_stringContent.put("certainty", "low");
				_stringContent.put("polarity", "pos");
			} else {
				// high confidence in a negative answer
				_stringContent.put("certainty", "high");
				_stringContent.put("polarity", "neg");
			}
		}

		@Override
		protected void actionComplete() {
			try {
				// global-type-question-answered
				String answeredPredictate = "polar-" + getAction().feature
						+ "-question-answered";
				// record that we have looked at it

				BeliefUtils.addFeature(getComponent(),
						getAction().beliefAddress, GroundedBelief.class,
						answeredPredictate, getAction().value);

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
				((AbstractDialogueActionInterface<?>) getComponent())
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
				((AbstractDialogueActionInterface<?>) getComponent())
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

				((AbstractDialogueActionInterface<?>) getComponent())
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

				((AbstractDialogueActionInterface<?>) getComponent())
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

				((AbstractDialogueActionInterface<?>) getComponent())
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
				((AbstractDialogueActionInterface<?>) getComponent())
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
	 * @param belief
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	protected boolean removeQuestionReference(WorkingMemoryAddress _beliefAddr,
			GroundedBelief _belief) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, _belief);

		// HACK - this is the more efficient place to do this
		belief.getContent().remove("attributed-color");
		belief.getContent().remove("attributed-shape");
		belief.getContent().remove("attributed-type");
		// END HACK

		boolean result = false;

		if (belief.getContent().containsKey(IS_POTENTIAL_OBJECT_IN_QUESTION)) {
			belief.getContent().remove(IS_POTENTIAL_OBJECT_IN_QUESTION);
			result = true;
		}

		overwriteWorkingMemory(_beliefAddr, belief.get());

		return result;
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

	private WorkingMemoryAddress m_lastPossibleIntentionsAddition;

	// public static void main(String[] args) throws SecurityException,
	// NoSuchMethodException {
	//
	// DialogueActionInterface diag = new DialogueActionInterface();
	// Constructor<VerifyReferenceExecutor> constructor =
	// VerifyReferenceExecutor.class
	// .getConstructor(ManagedComponent.class);
	//
	// System.out.println("asdasds");
	//
	// }

	@Override
	protected void start() {

		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(
						PossibleInterpretedIntentions.class,
						WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						m_lastPossibleIntentionsAddition = _wmc.address;
					}
				});

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

			m_actionStateManager
					.registerActionType(
							VerifyReference.class,
							new ComponentActionFactory<VerifyReference, VerifyReferenceExecutor>(
									this, VerifyReferenceExecutor.class));

			m_actionStateManager
					.registerActionType(
							VerifyReferenceByFeatureValue.class,
							new ComponentActionFactory<VerifyReferenceByFeatureValue, VerifyByFeatureValueReferenceExecutor>(
									this,
									VerifyByFeatureValueReferenceExecutor.class));

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

	/**
	 * Utility method to try all extractions when you don't know the type. I
	 * would like there to be a nicer way to do this. Is it even necessary?
	 * 
	 * @param ii
	 * @return
	 */
	public static RichIntention extractRichIntention(InterpretedIntention ii) {

		RichIntention decoded = PolarFeatureQuestionIntention.Transcoder.INSTANCE
				.tryDecode(ii);

		if (decoded == null) {
			decoded = OpenFeatureQuestionIntention.Transcoder.INSTANCE
					.tryDecode(ii);
		}

		if (decoded == null) {
			decoded = FeatureAscriptionIntention.Transcoder.INSTANCE
					.tryDecode(ii);
		}

		return decoded;
	}

}
