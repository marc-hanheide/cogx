package eu.cogx.goals.george;

import motivation.components.generators.AbstractWMEntryMotiveGenerator;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import vision.execution.george.VisionActionInterface;
import autogen.Planner.Goal;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import eu.cogx.beliefs.slice.SharedBelief;

public class InterpretedIntentionMotiveGenerator
		extends
		AbstractWMEntryMotiveGenerator<TutorInitiativeMotive, InterpretedIntention> {

	public InterpretedIntentionMotiveGenerator() {
		super(TutorInitiativeMotive.class, InterpretedIntention.class);
	}

	// @Override
	// protected void start() {
	// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
	// InterpretedIntention.class, WorkingMemoryOperation.ADD),
	// new WorkingMemoryChangeReceiver() {
	//
	// @Override
	// public void workingMemoryChanged(WorkingMemoryChange _wmc)
	// throws CASTException {
	// newInterpretedIntention(
	// _wmc.address,
	// getMemoryEntry(_wmc.address,
	// InterpretedIntention.class));
	//
	// }
	// });
	// }

	private void logIntention(InterpretedIntention _intention) {
		println("stringContent");
		for (String key : _intention.stringContent.keySet()) {
			println(key + " -> " + _intention.stringContent.get(key));
		}

		println("");
		println("addressContent");
		for (String key : _intention.addressContent.keySet()) {
			println(key + " -> "
					+ CASTUtils.toString(_intention.addressContent.get(key)));
		}
	}

	private TutorInitiativeLearningMotive newAssertionIntention(
			InterpretedIntention _intention) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, AlreadyExistsOnWMException {
		String subtype = _intention.stringContent.get("subtype");
		if (subtype.equals("ascription")) {
			return tutorDrivenAscription(_intention);
		} else {
			log("unknown InterpretedIntention type");
			logIntention(_intention);
			return null;
		}
	}

	private TutorInitiativeQuestionMotive newQuestionIntention(
			InterpretedIntention _intention) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		// [LOG gg.ii: unknown InterpretedIntention type]
		// [gg.ii: stringContent]
		// [gg.ii: subtype -> polar]
		// [gg.ii: subclass -> info-request]
		// [gg.ii: class -> communication]
		// [gg.ii: feature -> color]
		// [gg.ii: type -> question]
		// [gg.ii: hypothesis -> red]
		// [gg.ii: ]
		// [gg.ii: addressContent]
		// [gg.ii: about -> [WMA id = 1:31 : sa = binder]]

		String subtype = _intention.stringContent.get("subtype");
		if (subtype.equals("open")) {
			return openQuestion(_intention.stringContent.get("feature"),
					getGroundedBeliefAddress(_intention.addressContent
							.get("about")));
		} else if (subtype.equals("polar")) {
			return polarQuestion(_intention.stringContent.get("feature"),
					_intention.stringContent.get("hypothesis"),
					getGroundedBeliefAddress(_intention.addressContent
							.get("about")));
		} else {
			log("unknown InterpretedIntention type");
			logIntention(_intention);
			return null;
		}
	}

	private TutorInitiativeQuestionMotive polarQuestion(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddress) {
		// TODO Auto-generated method stub
		return null;
	}

	private WorkingMemoryAddress getGroundedBeliefAddress(
			WorkingMemoryAddress _sharedBeliefAddress)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		SharedBelief belief = getMemoryEntry(_sharedBeliefAddress,
				SharedBelief.class);
		return ((CASTBeliefHistory) belief.hist).ancestors.get(0).address;
	}

	private TutorInitiativeQuestionMotive openQuestion(String _feature,
			WorkingMemoryAddress _groundedBeliefAddress) {

		// [LOG gg.ii: unknown InterpretedIntention type]
		// [gg.ii: stringContent]
		// [gg.ii: subtype -> open]
		// [gg.ii: subclass -> info-request]
		// [gg.ii: class -> communication]
		// [gg.ii: feature -> color]
		// [gg.ii: type -> question]
		// [gg.ii: ]
		// [gg.ii: addressContent]
		// [gg.ii: about -> [WMA id = 1:31 : sa = binder]]

		// global-color-question-answered (VisualObject)

		String predicate = CASTUtils.concatenate("global-", _feature,
				"-question-answered");

		TutorInitiativeQuestionMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeQuestionMotive.class, null,
						getCASTTime());

		String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(
				predicate, _groundedBeliefAddress.id);

		motive.goal = new Goal(100f, goal, false);

		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

		return motive;

	}

	private TutorInitiativeLearningMotive tutorDrivenAscription(
			InterpretedIntention _intention) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, AlreadyExistsOnWMException {

		// [gg.ii: tutorDrivenAscription]
		// [gg.ii: stringContent]
		// [gg.ii: asserted-value -> red]
		// [gg.ii: subtype -> ascription]
		// [gg.ii: asserted-polarity -> pos]
		// [gg.ii: type -> assertion]
		// [gg.ii: asserted-feature -> color]
		// [gg.ii: ]
		// [gg.ii: addressContent]
		// [gg.ii: asserted -> [WMA id = irecog:1 : sa = dialogue]]
		// [gg.ii: about -> [WMA id = 1:31 : sa = binder]]
		// [LOG gg.ii: goal is (color-learnt '0:U') with inf-gain 0.0]

		String feature = _intention.stringContent.get("asserted-feature");
		String polarity = _intention.stringContent.get("asserted-polarity");
		String postfix;

		if (polarity.equals("pos")) {
			postfix = VisionActionInterface.LEARNED_FEATURE_POSTFIX;
		} else {
			postfix = VisionActionInterface.UNLEARNED_FEATURE_POSTFIX;
		}

		println("tutorDrivenAscription");
		logIntention(_intention);

		TutorInitiativeLearningMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeLearningMotive.class, null,
						getCASTTime());

		String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(feature
				+ postfix, groundedBeliefID(_intention));

		motive.goal = new Goal(100f, goal, false);

		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

		return motive;
	}

	/**
	 * Get the id of the belief that this intention is "about".
	 * 
	 * @param _intention
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private String groundedBeliefID(InterpretedIntention _intention)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return getGroundedBeliefAddress(_intention.addressContent.get("about")).id;
	}

	@Override
	protected TutorInitiativeMotive checkForAddition(
			WorkingMemoryAddress _addr, InterpretedIntention _intention) {
		TutorInitiativeMotive motive = null;
		try {
			String type = _intention.stringContent.get("type");
			if (type.equals("assertion")) {
				motive = newAssertionIntention(_intention);
			} else if (type.equals("question")) {
				motive = newQuestionIntention(_intention);
			}
		} catch (CASTException e) {
			logException(e);
		}

		log("unknown InterpretedIntention type");
		logIntention(_intention);
		if (motive != null) {
			motive.referenceEntry = _addr;
		}
		return motive;

	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(
			InterpretedIntention newEntry, TutorInitiativeMotive existingMotive) {
		return existingMotive;
	}

}
