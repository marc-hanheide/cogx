package dialogue.execution;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;

import javax.swing.JOptionPane;

import vision.execution.george.VisionActionInterface;

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
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretationStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.NewIntentionRecognizer;
import de.dfki.lt.tr.dialogue.util.VerbalisationUtils;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.RichIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.FeatureAscriptionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.OpenFeatureQuestionIntention;
import de.dfki.lt.tr.dialogue.intentions.inst.PolarFeatureQuestionIntention;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.slice.VerifiedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import execution.components.AbstractActionInterface;
import execution.slice.Action;
import execution.slice.ConfidenceLevel;
import execution.slice.TriBool;
import execution.slice.actions.AnnounceAutonomousFeatureLearning;
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
import execution.util.NonBlockingActionExecutor;

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

	public static final String MOTIVE_TRANSFER = "motive-transfer";
	public static final String MOTIVE_TRANSFER_VALUE = MOTIVE_TRANSFER
			+ "-value";

	boolean madeup;

	protected boolean m_fakeIt;

	public AbstractDialogueActionInterface(Class<BeliefType> _beliefCls) {
		super(_beliefCls);
	}

	public static class AnnounceAutonomousLearningExecutor extends
			BlockingActionExecutor<AnnounceAutonomousFeatureLearning> {
		public AnnounceAutonomousLearningExecutor(ManagedComponent _component) {
			super(_component, AnnounceAutonomousFeatureLearning.class);
		}

		@Override
		public TriBool execute() {
			VerbalisationUtils.verbaliseString(getComponent(),
					"I know this object is " + getAction().value
							+ ". I'll update my model");
			return TriBool.TRITRUE;
		}
	}

	public abstract static class IntentionDialogueAction<T extends Action>
			extends NonBlockingActionExecutor<T> {

		private static final int DLG_TIMEOUT = 30;

		private final int m_timeoutSeconds;
		private final int m_pollFrequencyMilliseconds;

		WMEventQueue eventQueue = new WMEventQueue();

		// the tribool to return on a timeout
		private final TriBool m_timeoutResponse;

		private boolean m_stopped = false;

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls, int _timeoutSeconds, TriBool _timeoutResponse) {
			super(_component, _cls);
			m_timeoutSeconds = _timeoutSeconds;
			m_timeoutResponse = _timeoutResponse;
			m_pollFrequencyMilliseconds = 200;
		}

		public IntentionDialogueAction(ManagedComponent _component,
				Class<T> _cls) {
			this(_component, _cls, DLG_TIMEOUT, TriBool.TRIFALSE);
		}

		@Override
		public void executeAction() {
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
				if (m_stopped) {
					return;
				}
			} catch (CASTException e) {
				logException(e);
			} finally {
				((AbstractDialogueActionInterface<?>) getComponent())
						.disableASR();
			}
			if (res == TriBool.TRITRUE) {
				actionSuccessfullyCompleted();
			}
			executionComplete(res);
		}

		@Override
		public void stopExecution() {
			m_stopped = true;
		}

		/**
		 * Hack to provide the same interface as the non-blocking version.
		 */
		protected void actionSuccessfullyCompleted() {
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

				long timeoutMillis = TimeUnit.SECONDS
						.toMillis(m_timeoutSeconds);
				long elapsedMillis = 0;

				while (!gotAnswer) {
					WorkingMemoryChange e = eventQueue.poll(
							m_pollFrequencyMilliseconds, TimeUnit.MILLISECONDS);

					if (m_stopped) {
						println("stop has been triggered before intention arrived... action halting");
						return m_timeoutResponse;
					}

					if (e != null) {
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

					// the above statements will have returned if the action has
					// been stopped or if the matching intention was found,
					// therefore we only get here if no intention has been found

					if (elapsedMillis < timeoutMillis) {
						elapsedMillis += m_pollFrequencyMilliseconds;
					} else {
						println("didn't get the intention in time... action halting");
						return m_timeoutResponse;
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

		protected void logAddressContent(
				Map<String, WorkingMemoryAddress> _addressContent) {
			getComponent().println(
					this.getClass().getSimpleName()
							+ ": created address content for intention");
			for (Entry<String, WorkingMemoryAddress> e : _addressContent
					.entrySet()) {
				getComponent().println(
						"  " + e.getKey() + " => "
								+ CASTUtils.toString(e.getValue()));
			}
		}

		protected void logStringContent(Map<String, String> _stringContent) {
			getComponent().println(
					this.getClass().getSimpleName()
							+ ": created address content for intention");
			for (Entry<String, String> e : _stringContent.entrySet()) {
				getComponent().println(
						"  " + e.getKey() + " => " + e.getValue());
			}
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

	public static class AskForObjectWithFeatureExecutor extends
			IntentionDialogueAction<AskForObjectWithFeatureValue> {

		// TODO, what is happening here?
		private static final int ANSWER_TIMEOUT_SECS = 5;

		public AskForObjectWithFeatureExecutor(ManagedComponent _component) {
			super(_component, AskForObjectWithFeatureValue.class,
					ANSWER_TIMEOUT_SECS, TriBool.TRITRUE);
		}

		@Override
		protected void addStringContent(Map<String, String> _stringContent) {

			// http://codex.cs.bham.ac.uk/trac/cogx/ticket/466
			// type = "request"
			// subtype = "non-situated"
			// feature = "color" | "shape"
			// value = the desired value
			_stringContent.put("type", "request");
			_stringContent.put("subtype", "non-situated");
			_stringContent.put("feature", getAction().feature);
			_stringContent.put("value", getAction().value);
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
			// [33m[gg.ii: asserted-polarity -> pos][0m or neg
			// [33m[gg.ii: asserted-feature -> color][0m
			// [33m[gg.ii: type -> assertion][0m
			// [33m[gg.ii: ][0m

			// TODO what if the answer is a negation?

			// TODO do all this via a RichIntention just like input intentions

			WorkingMemoryPointer verifiedAncestorPtr = BeliefUtils
					.recurseAncestorsForType(getComponent(),
							getAction().beliefAddress,
							CASTUtils.typeName(VerifiedBelief.class));

			if (verifiedAncestorPtr == null) {
				warn("eek, verified was null");
			} else {
				log("adding attibution to verified belief ancestor at: "
						+ CASTUtils.toString(verifiedAncestorPtr.address));

				String feature = _ii.stringContent.get("asserted-feature");
				String value = _ii.stringContent.get("asserted-value");

				VerifiedBelief belief = getComponent().getMemoryEntry(
						verifiedAncestorPtr.address, VerifiedBelief.class);
				CASTIndependentFormulaDistributionsBelief<VerifiedBelief> pb = CASTIndependentFormulaDistributionsBelief
						.create(VerifiedBelief.class, belief);
				double prob = 1.0;

				if (_ii.stringContent.get("asserted-polarity").equals("neg")) {
					prob = 0;
					// HACK add unlearning goal via belief
					BeliefUtils.addFeature(pb, MOTIVE_TRANSFER, feature
							+ VisionActionInterface.UNLEARNED_FEATURE_POSTFIX);
					BeliefUtils.addFeature(pb, MOTIVE_TRANSFER_VALUE, value);
				}

				FormulaDistribution distr = FormulaDistribution.create();
				distr.add(value, prob);
				pb.getContent().put(feature, distr);

				distr = FormulaDistribution.create();
				distr.add(new FloatFormula(-1, 1.0f), 1);
				pb.getContent().put(feature + "-prob", distr);

				distr = FormulaDistribution.create();
				distr.add(value, prob);
				pb.getContent().put("attributed-" + feature, distr);

				try {

					getComponent().overwriteWorkingMemory(
							verifiedAncestorPtr.address, pb.get());

				} catch (SubarchitectureComponentException e) {
					logException(e);
				}

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
			// Handle matched response looking like this:
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

			// this is the thing we care about
			// get the list of all potential intentions and thus referents

			WorkingMemoryAddress verifiedReferentAddr = interpretedIntention.addressContent
					.get("about");

			WorkingMemoryAddress piiAddr = interpretedIntention.addressContent
					.get("verification-of");

			PossibleInterpretedIntentions pii = getComponent().getMemoryEntry(
					piiAddr, PossibleInterpretedIntentions.class);

			println("number of iis in pii found on WM: "
					+ pii.intentions.size());

			for (WorkingMemoryAddress addr : pii.intentions.keySet()) {

				InterpretedIntention iint = pii.intentions.get(addr);
				WorkingMemoryAddress potentialReferentAddr = iint.addressContent
						.get("about");

				// if the ii at addr was NOT about reference we just verified
				if (!potentialReferentAddr.equals(verifiedReferentAddr)) {
					// if the verification was successful, then this ii in
					// incorrect
					if (polarity.equals("pos")) {
						updateIncorrectIntention(pii, addr,
								potentialReferentAddr);
					} else {
						// we can't update this
					}
				}

				// if the ii at addr WAS about reference we just verified
				else {
					// if the verification was successful, then this ii in
					// CORRECT
					if (polarity.equals("pos")) {
						updateCorrectIntention(pii, piiAddr, iint, addr);
					}
					// if we were verifying this and they said no, then this is
					// incorrect
					else {
						updateIncorrectIntention(pii, addr,
								potentialReferentAddr);
					}
				}

			}

			// here we could be in the case where we've stated that all but
			// one interpretation is incorrect, thus we should mark the
			// remaining interpretation as correct
			WorkingMemoryAddress remainingCorrectAddr = checkForRemainder(pii);
			if (remainingCorrectAddr != null) {
				println("last remaining unchecked interpretation must be correct: "
						+ CASTUtils.toString(remainingCorrectAddr));
				updateCorrectIntention(pii, piiAddr,
						pii.intentions.get(remainingCorrectAddr),
						remainingCorrectAddr);
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

		private void updateCorrectIntention(PossibleInterpretedIntentions _pii,
				WorkingMemoryAddress _piiAddr, InterpretedIntention _correctII,
				WorkingMemoryAddress _correctIIAddr)
				throws DoesNotExistOnWMException, ConsistencyException,
				PermissionException, UnknownSubarchitectureException {
			_pii.statuses.put(_correctIIAddr, InterpretationStatus.CORRECT);

			assert (_pii.resolvedIntention
					.equals(NewIntentionRecognizer.EMPTY_ADDRESS));

			// store the correctly resolved intention address in
			// the
			// possibles structure
			_pii.resolvedIntention = _correctIIAddr;
			// write PII back to wm to reflect update to
			getComponent().overwriteWorkingMemory(_piiAddr, _pii);

			// else decode and mark as accepted
			RichIntention decoded = AbstractDialogueActionInterface
					.extractRichIntention(_correctII);

			if (decoded == null) {
				getComponent().getLogger().warn("Unable to decode intention",
						getComponent().getLogAdditions());

			} else {
				CASTEffect acceptEffect = decoded.getOnAcceptEffect();
				acceptEffect.makeItSo(getComponent());
				log("executed accept effect");
			}
		}

		private void updateIncorrectIntention(
				PossibleInterpretedIntentions _pii,
				WorkingMemoryAddress _incorrectIIAddr,
				WorkingMemoryAddress _incorrectIIReferentAddr)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException, ConsistencyException,
				PermissionException {
			_pii.statuses.put(_incorrectIIAddr, InterpretationStatus.INCORRECT);
			unmarkReferent(_incorrectIIReferentAddr);
		}

		private WorkingMemoryAddress checkForRemainder(
				PossibleInterpretedIntentions pii) {

			WorkingMemoryAddress lastUnchecked = null;
			for (WorkingMemoryAddress addr : pii.intentions.keySet()) {
				// we have found an unchecked ii
				if (pii.statuses.get(addr) == InterpretationStatus.UNCHECKED) {
					// if we previously found one, then we don't have a single
					// remainder
					if (lastUnchecked != null) {
						return null;
					} else {
						lastUnchecked = addr;
					}
				}
			}
			return lastUnchecked;
		}

		private void unmarkReferent(WorkingMemoryAddress _refGroundBelAddr)
				throws DoesNotExistOnWMException,
				UnknownSubarchitectureException, ConsistencyException,
				PermissionException {

			// we are not touching the list of beliefs in the motive here
			// because it doesn't really alter system behaviour if this list is
			// changed during execution by the motive that will ultimately clean
			// it at the end

			println("removing reference marker from belief at "
					+ CASTUtils.toString(_refGroundBelAddr));

			if (!((AbstractDialogueActionInterface<?>) getComponent())
					.removeQuestionReference(_refGroundBelAddr,
							_refGroundBelAddr)) {
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
		protected void actionSuccessfullyCompleted() {
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
		protected void actionSuccessfullyCompleted() {
			try {
				// global-type-question-answered
				String answeredPredictate = "polar-" + getAction().feature
						+ "-question-answered";
				// record that we have looked at it

				BeliefUtils.addFeature(getComponent(),
						getAction().beliefAddress, MergedBelief.class,
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
			setCompletion(true);
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
			setCompletion(true);
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
			setCompletion(true);
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
			setCompletion(true);
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
			setCompletion(true);
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
			setCompletion(true);
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
			setCompletion(true);
			return result;
		}

	}

	private void addFeatureDirectly(SingleBeliefAction _action,
			String _feature, String _value, double _prob)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		MergedBelief belief = getMemoryEntry(_action.beliefAddress,
				MergedBelief.class);
		CASTIndependentFormulaDistributionsBelief<MergedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, belief);

		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, _prob);
		pb.getContent().put(_feature, fd);
		overwriteWorkingMemory(_action.beliefAddress, pb.get());
	}

	/**
	 * called when the robot should STOP listening
	 */
	public void disableASR() {
		println("disabling ASR not implemented in "
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
			WorkingMemoryAddress _refGroundBelAddr)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		BeliefType rawBelief = getMemoryEntry(_refGroundBelAddr, m_beliefCls);

		CASTIndependentFormulaDistributionsBelief<BeliefType> belief = CASTIndependentFormulaDistributionsBelief
				.create(m_beliefCls, rawBelief);

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

			m_actionStateManager
					.registerActionType(
							AskForObjectWithFeatureValue.class,
							new ComponentActionFactory<AskForObjectWithFeatureValue, AskForObjectWithFeatureExecutor>(
									this, AskForObjectWithFeatureExecutor.class));

			m_actionStateManager
					.registerActionType(
							AnnounceAutonomousFeatureLearning.class,
							new ComponentActionFactory<AnnounceAutonomousFeatureLearning, AnnounceAutonomousLearningExecutor>(
									this,
									AnnounceAutonomousLearningExecutor.class));

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

		//

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
