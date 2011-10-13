package eu.cogx.goals.george;

import java.util.ArrayList;
import java.util.List;

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
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.slice.Robot;

public abstract class AbstractInterpretedIntentionMotiveGenerator<T extends Ice.Object>
		extends AbstractWMEntryMotiveGenerator<TutorInitiativeMotive, T> {

	public AbstractInterpretedIntentionMotiveGenerator(Class<T> _entryCls) {
		super(TutorInitiativeMotive.class, _entryCls);
		monitorMotivesForDeletion(true);
	}

	protected WorkingMemoryAddress m_robotEntry;

	public WorkingMemoryAddress getRobotAddress() {
		assert (m_robotEntry != null);
		return m_robotEntry;
	}

	@Override
	protected void start() {
		super.start();

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Robot.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				m_robotEntry = _wmc.address;
				removeChangeFilter(this);
			}
		});

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

		// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
		// TutorInitiativeMotive.class, WorkingMemoryOperation.DELETE),
		// new WorkingMemoryChangeReceiver() {
		//
		// @Override
		// public void workingMemoryChanged(WorkingMemoryChange _wmc)
		// throws CASTException {
		// println("SOMEONE DELETED MY INTENTION");
		// println(CASTUtils.toString(_wmc));
		// }
		// });
	}

	// TODO HACK sanitise between Dora and George
	private WorkingMemoryAddress m_robotBeliefAddr;

	// TODO HACK sanitise between Dora and George
	@Override
	protected WorkingMemoryAddress getRobotBeliefAddr() {
		if (m_robotBeliefAddr == null) {
			List<CASTData<GroundedBelief>> groundedBeliefs = new ArrayList<CASTData<GroundedBelief>>();
			try {
				getMemoryEntriesWithData(GroundedBelief.class, groundedBeliefs,
						"binder", 0);
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				return null;
			}

			for (CASTData<GroundedBelief> beliefEntry : groundedBeliefs) {
				if (beliefEntry.getData().type.equals("Robot")) {
					m_robotBeliefAddr = new WorkingMemoryAddress(
							beliefEntry.getID(), "binder");
					break;
				}
			}
			if (m_robotBeliefAddr == null) {
				getLogger().warn("unable to find belief '" + "Robot" + "'");
			}
		}
		return m_robotBeliefAddr;

	}

	protected String getAdditionalGoals() throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		println("fetching robot belief from " + getRobotBeliefAddr());
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class,
						getMemoryEntry(getRobotBeliefAddr(),
								GroundedBelief.class));

		return VisualObjectMotiveGenerator.beliefPredicateGoal(
				"arm-in-resting-position", belief);

	}

	protected void logIntention(BaseIntention _intention) {
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

	protected TutorInitiativeLearningMotive newAssertionIntention(
			InterpretedIntention _intention) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, AlreadyExistsOnWMException,
			ConsistencyException, PermissionException {

		String subtype = _intention.stringContent.get("subtype");
		if (subtype.equals("ascription")) {
			return tutorDrivenAscription(_intention);
		} else {
			log("unknown InterpretedIntention type");
			logIntention(_intention);
			return null;
		}
	}

	protected TutorInitiativeQuestionMotive newQuestionIntention(
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

	protected abstract String getPolarQuestionGoalString(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddr);

	/**
	 * @param _addr
	 * @param _intention
	 * @param motive
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws AlreadyExistsOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 */
	protected TutorInitiativeMotive generateMotiveFromIntention(
			WorkingMemoryAddress _addr, InterpretedIntention _intention)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			AlreadyExistsOnWMException, ConsistencyException,
			PermissionException {

		TutorInitiativeMotive motive = null;
		String type = _intention.stringContent.get("type");
		if (type.equals("assertion")) {
			motive = newAssertionIntention(_intention);
		} else if (type.equals("question")) {
			motive = newQuestionIntention(_intention);
		}

		if (motive != null) {
			motive.referenceEntry = _addr;
		} else {
			log("unknown InterpretedIntention type");
			logIntention(_intention);
		}
		return motive;
	}

	private TutorInitiativeQuestionMotive polarQuestion(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddress)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		// [LOG gg.ii: polar question intention]
		// [LOG gg.ii: unknown InterpretedIntention type]
		// [gg.ii: stringContent]
		// [gg.ii: subtype -> polar]
		// [gg.ii: subclass -> info-request]
		// [gg.ii: class -> communication]
		// [gg.ii: feature -> color]
		// [gg.ii: type -> question]
		// [gg.ii: hypothesis -> red]

		TutorInitiativeQuestionMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeQuestionMotive.class, null,
						getCASTTime());

		String goalString = conjoinGoalStrings(new String[] {
				getAdditionalGoals(),
				getPolarQuestionGoalString(_feature, _hypothesis,
						_groundedBeliefAddress) });

		motive.goal = new Goal(100f, goalString, false);

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
			WorkingMemoryAddress _groundedBeliefAddress)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		TutorInitiativeQuestionMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeQuestionMotive.class, null,
						getCASTTime());

		String goalString = conjoinGoalStrings(new String[] {
				getAdditionalGoals(),
				getOpenQuestionGoalString(_feature, _groundedBeliefAddress) });

		motive.goal = new Goal(100f, goalString, false);

		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

		return motive;

	}

	protected abstract String getOpenQuestionGoalString(String _feature,
			WorkingMemoryAddress _groundedBeliefAddr);

	private TutorInitiativeLearningMotive tutorDrivenAscription(
			InterpretedIntention _intention) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, AlreadyExistsOnWMException,
			ConsistencyException, PermissionException {

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
		String value = _intention.stringContent.get("asserted-value");

		boolean learn;

		if (polarity.equals("pos")) {
			learn = true;
		} else {
			learn = false;
		}

		println("tutorDrivenAscription");
		logIntention(_intention);

		TutorInitiativeLearningMotive motive = VisualObjectMotiveGenerator
				.newMotive(TutorInitiativeLearningMotive.class, null,
						getCASTTime());

		String goalString = conjoinGoalStrings(new String[] {
				getAdditionalGoals(),
				getAscriptionGoalString(feature, learn,
						groundedBeliefID(_intention)) });

		// HACK used later for adding attribution to all possible referentss
		motive.assertedFeature = feature;
		motive.assertedValue = value;
		motive.assertedLearn = learn;
		// HACK END

		motive.goal = new Goal(100f, goalString, false);

		log("goal is " + motive.goal.goalString + " with inf-gain "
				+ motive.informationGain);

		// HACK or NOT-HACK? add attributed feature into ground belief
		addAttribution(_intention.addressContent.get("about"), feature, value,
				learn);

		return motive;
	}

	protected String getAscriptionPredicate(String feature, boolean learn) {

		String postfix = VisionActionInterface.LEARNED_FEATURE_POSTFIX;
		if (!learn) {
			postfix = VisionActionInterface.UNLEARNED_FEATURE_POSTFIX;
		}
		return feature + postfix;
	}

	protected abstract String getAscriptionGoalString(String feature,
			boolean learn, String groundedBeliefID);

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

	public void addStringFeature(WorkingMemoryAddress _groundedBeliefAddr,
			String _feature, String _value) throws DoesNotExistOnWMException,
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
	 * Mark the referred-to belief as a potential referent in question
	 * answering.
	 * 
	 * TODO handle multiple intentions/ambiguous referents
	 * 
	 * @param _groundedBeliefAddr
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	protected void markReferent(WorkingMemoryAddress _groundedBeliefAddr)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
		println("marking referent");
		addBooleanFeature(
				_groundedBeliefAddr,
				AbstractDialogueActionInterface.IS_POTENTIAL_OBJECT_IN_QUESTION,
				true);
		// planning won't work without this in place anyway, but it probably
		// isn't required on faster machines
		sleepComponent(500);
	}

	protected void addAttribution(WorkingMemoryAddress _groundedBeliefAddr,
			String _feature, String _value, boolean _polarity)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		String attributionPredication = "attributed-" + _feature;
		println("adding attribution: (" + attributionPredication + " " + _value
				+ ")");

		addStringFeature(_groundedBeliefAddr, attributionPredication, _value);

		// planning won't work without this in place anyway, but it probably
		// isn't required on faster machines
		sleepComponent(500);
	}

	// @Override
	// protected TutorInitiativeMotive checkForUpdate(
	// InterpretedIntention newEntry, TutorInitiativeMotive existingMotive) {
	// return existingMotive;
	// }

	/**
	 * Removes reference flag from belief. Does not update on WM.
	 * 
	 * @param _gb
	 */
	protected void unmarkReferent(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> _gb) {

		_gb.getContent()
				.remove(AbstractDialogueActionInterface.IS_POTENTIAL_OBJECT_IN_QUESTION);

	}

	private static final String[] POTENTIAL_ACTION_EFFECTS = new String[] {
			"global-color-question-answered", "global-shape-question-answered",
			"global-type-question-answered",
			"object-refering-color-question-answered",
			"object-refering-shape-question-answered",
			"object-refering-type-question-answered", "color-learned",
			"shape-learned", "type-learned", "color-unlearned",
			"shape-unlearned", "type-unlearned",
			"polar-color-question-answered", "polar-shape-question-answered",
			"polar-type-question-answered" };

	protected void removeActionEffects(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb) {

		for (String potentialEffect : POTENTIAL_ACTION_EFFECTS) {
			FormulaDistribution removed = gb.getContent().remove(
					potentialEffect);
			if (removed != null) {
				log("removed effect from belief: " + potentialEffect);
			}
		}
	}

	protected void cleanBelief(WorkingMemoryAddress _groundedBeliefAddr)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {

		GroundedBelief belief = getMemoryEntry(_groundedBeliefAddr,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		// remove marking for reference
		unmarkReferent(gb);
		// remove potential results of learning and dialoguate
		removeActionEffects(gb);

		overwriteWorkingMemory(_groundedBeliefAddr, gb.get());
	}
}
