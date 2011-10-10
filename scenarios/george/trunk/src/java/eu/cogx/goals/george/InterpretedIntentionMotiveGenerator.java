package eu.cogx.goals.george;

import motivation.components.generators.AbstractWMEntryMotiveGenerator;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import vision.execution.george.VisionActionInterface;
import autogen.Planner.Goal;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import eu.cogx.beliefs.slice.GroundedBelief;

public class InterpretedIntentionMotiveGenerator
		extends
		AbstractWMEntryMotiveGenerator<TutorInitiativeMotive, InterpretedIntention> {

	public InterpretedIntentionMotiveGenerator() {
		super(TutorInitiativeMotive.class, InterpretedIntention.class);
	}

	@Override
	protected void start() {
		super.start();
		// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
		// BaseIntention.class, WorkingMemoryOperation.ADD),
		// new WorkingMemoryChangeReceiver() {
		//
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _wmc)
		// throws CASTException {
		// logIntention(getMemoryEntry(_wmc.address,
		// BaseIntention.class));
		//
		// }
		// });
	}

	private void logIntention(BaseIntention _intention) {
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

		log("question intention");
		String subtype = _intention.stringContent.get("subtype");
		if (subtype.equals("open")) {
			log("open question intention");
			return openQuestion(_intention.stringContent.get("feature"),
					_intention.addressContent.get("about"));
		} else if (subtype.equals("polar")) {
			log("polar question intention");
			return polarQuestion(_intention.stringContent.get("feature"),
					_intention.stringContent.get("hypothesis"),
					_intention.addressContent.get("about"));
		} else {
			log("unknown InterpretedIntention type");
			logIntention(_intention);
			return null;
		}
	}

	private TutorInitiativeQuestionMotive polarQuestion(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddress) {
		// [LOG gg.ii: polar question intention]
		// [LOG gg.ii: unknown InterpretedIntention type]
		// [gg.ii: stringContent]
		// [gg.ii: subtype -> polar]
		// [gg.ii: subclass -> info-request]
		// [gg.ii: class -> communication]
		// [gg.ii: feature -> color]
		// [gg.ii: type -> question]
		// [gg.ii: hypothesis -> red]

		String predicate = CASTUtils.concatenate("polar-", _feature,
				"-question-answered");

		TutorInitiativeQuestionMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeQuestionMotive.class, null,
						getCASTTime());

		// String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(
		// predicate, _groundedBeliefAddress.id);
		// String goal = predicate;

		String goal = "(exists (?v - VisualObject) (and (" + predicate + " ?v "
				+ _hypothesis + ")))";

		motive.goal = new Goal(100f, goal, false);

		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

		return motive;
	}

	// private WorkingMemoryAddress getGroundedBeliefAddress(
	// WorkingMemoryAddress _sharedBeliefAddress)
	// throws DoesNotExistOnWMException, UnknownSubarchitectureException {
	// SharedBelief belief = getMemoryEntry(_sharedBeliefAddress,
	// SharedBelief.class);
	// return ((CASTBeliefHistory) belief.hist).ancestors.get(0).address;
	// }

	private TutorInitiativeQuestionMotive openQuestion(String _feature,
			WorkingMemoryAddress _groundedBeliefAddress) {

		String predicate = CASTUtils.concatenate("global-", _feature,
				"-question-answered");

		TutorInitiativeQuestionMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeQuestionMotive.class, null,
						getCASTTime());

		// String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(
		// predicate, _groundedBeliefAddress.id);
		// String goal = predicate;

		String goal = "(exists (?v - VisualObject) (and (" + predicate
				+ " ?v)))";

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
		return _intention.addressContent.get("about").id;
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

			if (motive != null) {
				motive.referenceEntry = _addr;

				// mark referents
				markReferent(_intention.addressContent.get("about"));

			} else {
				log("unknown InterpretedIntention type");
				logIntention(_intention);
			}
		} catch (CASTException e) {
			// reset in case of error
			motive = null;
			logException(e);
		}

		return motive;

	}

	public void addBooleanFeature(WorkingMemoryAddress _groundedBeliefAddr,
			String _feature, boolean _value) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		GroundedBelief belief = getMemoryEntry(_groundedBeliefAddr,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, 1);

		pb.getContent().put(_feature, fd);
		overwriteWorkingMemory(_groundedBeliefAddr, pb.get());
	}

	/**
	 * Mark the referred-to belief as a potential referent in question answering.
	 * 
	 * TODO handle multiple intentions/ambiguous referents
	 * 
	 * @param _groundedBeliefAddr
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	private void markReferent(WorkingMemoryAddress _groundedBeliefAddr)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		println("marking referent");
		addBooleanFeature(_groundedBeliefAddr,
				"is-potential-object-in-question", true);
		// planning won't work without this in place anyway, but it probably
		// isn't required on faster machines
		sleepComponent(1000);
	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(
			InterpretedIntention newEntry, TutorInitiativeMotive existingMotive) {
		return existingMotive;
	}

}
