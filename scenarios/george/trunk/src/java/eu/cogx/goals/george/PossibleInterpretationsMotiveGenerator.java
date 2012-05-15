package eu.cogx.goals.george;

import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.cast.dialogue.IntentionUnpacker;
import de.dfki.lt.tr.cast.dialogue.NewIntentionRecognizer;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.RichIntention;
import dialogue.execution.AbstractDialogueActionInterface;
import execution.slice.Robot;

/**
 * This component does one of two things with a PossibleInterpretedIntentions
 * entry. If it can select an {@link InterpretedIntention} then it does so and
 * writes it to WM. Else it creates a new goal to disambiguate the
 * interpretation.
 * 
 * @author nah
 * 
 */
public class PossibleInterpretationsMotiveGenerator
		extends
		AbstractInterpretedIntentionMotiveGenerator<PossibleInterpretedIntentions> {

	private static final double DISAMBIGUATION_CONFIDENCE_THRESHOLD = 0.9;

	public PossibleInterpretationsMotiveGenerator() {
		super(PossibleInterpretedIntentions.class);
	}

	private void updateFeatureExclusion(boolean _excludeColor,
			boolean _excludeShape) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException, ConsistencyException,
			PermissionException {

		lockEntry(getRobotAddress(), WorkingMemoryPermissions.LOCKEDODR);
		Robot rbt = getMemoryEntry(getRobotAddress(), Robot.class);
		rbt.excludeColor = _excludeColor;
		rbt.excludeShape = _excludeShape;
		overwriteWorkingMemory(getRobotAddress(), rbt);
		unlockEntry(getRobotAddress());
	}

	@Override
	protected TutorInitiativeMotive checkForAddition(
			WorkingMemoryAddress _piiAddr, PossibleInterpretedIntentions _pii) {

		// for now, just spit out most probable intention, then return null (so
		// no motive is generated)
		TutorInitiativeMotive motive = null;
		try {
			InterpretedIntention mostConfidentIntention = IntentionUnpacker
					.getMostConfidentIntention(_pii);

			// filter out answers and maybe other types later?
			if (!mostConfidentIntention.stringContent.get("subtype").contains(
					"answer")) {

				if (neeedsDisambiguation(_pii)) {
					println("generating motive to disambiguate");
					motive = generateDisambiguationMotive(_piiAddr, _pii);
				} else {

					println("unpacking most confident of "
							+ _pii.intentions.size()
							+ " possible interpretations");

					IntentionUnpacker.unpackMostConfidentIntention(this, _pii);
				}
			}
		} catch (SubarchitectureComponentException e) {
			logException(e);
		}

		return motive;

	}

	private TutorInitiativeMotive generateDisambiguationMotive(
			WorkingMemoryAddress _piiAddr, PossibleInterpretedIntentions _pii)
			throws DoesNotExistOnWMException, AlreadyExistsOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		// get most confident intention
		InterpretedIntention mostConfidentIntention = IntentionUnpacker
				.getMostConfidentIntention(_pii);

		// generate a motive structure based on most confident interpretation
		// this relies on the goal strings being existentially quantified rather
		// than referencing a particular belief

		// TODO Make this smarter next year...
		TutorInitiativeMotive motive = generateMotiveFromIntention(_piiAddr,
				mostConfidentIntention, true);

		// mark referents from /all/ interpretations.

		for (WorkingMemoryAddress addr : _pii.intentions.keySet()) {
			InterpretedIntention iint = _pii.intentions.get(addr);
			markReferent(aboutBeliefAddress(iint));
		}

		// if this is an attributed belief task, mark attributions in all
		// referents too
		if (motive instanceof TutorInitiativeLearningMotive) {

			TutorInitiativeLearningMotive tilm = (TutorInitiativeLearningMotive) motive;

			for (WorkingMemoryAddress addr : _pii.intentions.keySet()) {
				InterpretedIntention iint = _pii.intentions.get(addr);
				addAttribution(aboutBeliefAddress(iint), tilm.assertedFeature,
						tilm.assertedValue, tilm.assertedLearn);
			}
		}

		return motive;
	}

	private boolean neeedsDisambiguation(PossibleInterpretedIntentions _pii) {
		// TODO something more principled

		if (_pii.intentions.size() == 1) {
			return false;
		} else {

			for (InterpretedIntention ii : _pii.intentions.values()) {
				logIntention(ii);
			}

			InterpretedIntention mostConfidentIntention = IntentionUnpacker
					.getMostConfidentIntention(_pii);

			return mostConfidentIntention.confidence < DISAMBIGUATION_CONFIDENCE_THRESHOLD;
		}
	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(
			PossibleInterpretedIntentions newEntry,
			TutorInitiativeMotive existingMotive) {
		return existingMotive;
	}

	@Override
	protected String getPolarQuestionGoalString(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddr) {
		String predicate = CASTUtils.concatenate("polar-", _feature,
				"-question-answered");

		excludeFeatureFromReference(_feature);

		return "(exists (?v - VisualObject) (and (= (" + predicate + " ?v) "
				+ _hypothesis + ")))";

	}

	/**
	 * @param _feature
	 */
	private void excludeFeatureFromReference(String _feature) {
		try {
			if (_feature.equals("color")) {
				updateFeatureExclusion(true, false);
			} else if (_feature.equals("shape")) {
				updateFeatureExclusion(false, true);
			}
		} catch (SubarchitectureComponentException e) {
			logException(e);
		}
	}

	@Override
	protected String getOpenQuestionGoalString(String _feature,
			WorkingMemoryAddress _groundedBeliefAddr) {
		excludeFeatureFromReference(_feature);

		String predicate = CASTUtils.concatenate("global-", _feature,
				"-question-answered");
		return "(exists (?v - VisualObject) (and (" + predicate + " ?v)))";
	}

	@Override
	protected String getAscriptionGoalString(String _feature, boolean _learn,
			String _groundedBeliefID) {

		excludeFeatureFromReference(_feature);

		return "(exists (?v - VisualObject) (and ("
				+ getAscriptionPredicate(_feature, _learn) + " ?v)))";
	}

	@Override
	protected void motiveWasCompleted(TutorInitiativeMotive _motive)
			throws SubarchitectureComponentException {

		println("cleaning beliefs after disambiguation goal");

		PossibleInterpretedIntentions _pii = getMemoryEntry(
				_motive.referenceEntry, PossibleInterpretedIntentions.class);

		// clean up all beliefs involved in action
		for (WorkingMemoryAddress addr : _pii.intentions.keySet()) {
			InterpretedIntention iint = _pii.intentions.get(addr);
			cleanBelief(aboutBeliefAddress(iint));
		}

		// now execute success effects on resolved intention
		if (!_pii.resolvedIntention.equals(NewIntentionRecognizer.EMPTY_ADDRESS)) {
			InterpretedIntention iint = _pii.intentions
					.get(_pii.resolvedIntention);

			RichIntention decoded = AbstractDialogueActionInterface
					.extractRichIntention(iint);

			if (decoded == null) {
				getLogger().warn("Unable to decode intention",
						getLogAdditions());
			} else {
				CASTEffect successEffect = decoded.getOnSuccessEffect();
				successEffect.makeItSo(this);
				println("executed success effect");
			}

		} else {
			getLogger().warn(
					"diambiguation motive was completed but no referent set",
					getLogAdditions());
		}

		updateFeatureExclusion(false, false);
	}
}
