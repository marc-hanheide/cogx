package eu.cogx.goals.george;

import motivation.slice.TutorInitiativeMotive;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.RichIntention;
import dialogue.execution.AbstractDialogueActionInterface;

public class InterpretedIntentionMotiveGenerator extends
		AbstractInterpretedIntentionMotiveGenerator<InterpretedIntention> {

	public InterpretedIntentionMotiveGenerator() {
		super(InterpretedIntention.class);

	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(
			InterpretedIntention newEntry, TutorInitiativeMotive existingMotive) {
		return existingMotive;
	}

	@Override
	protected TutorInitiativeMotive checkForAddition(
			WorkingMemoryAddress _addr, InterpretedIntention _intention) {
		TutorInitiativeMotive motive = null;
		try {
			motive = generateMotiveFromIntention(_addr, _intention, false);
			// mark referents of chosen intention
			if (motive != null) {
				markReferent(_intention.addressContent.get("about"));
			}
		} catch (CASTException e) {
			// reset in case of exception in markReferent
			motive = null;
			logException(e);
		}
		return motive;
	}

	@Override
	protected String getOpenQuestionGoalString(String _feature,
			WorkingMemoryAddress _groundedBeliefAddr) {
		String predicate = CASTUtils.concatenate("global-", _feature,
				"-question-answered");
		String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(
				predicate, _groundedBeliefAddr.id);
		return goal;
	}

	@Override
	protected String getPolarQuestionGoalString(String _feature,
			String _hypothesis, WorkingMemoryAddress _groundedBeliefAddr) {
		String predicate = CASTUtils.concatenate("polar-", _feature,
				"-question-answered");
		return VisualObjectMotiveGenerator.beliefFunctionGoal(predicate,
				_groundedBeliefAddr.id, _hypothesis);
	}

	@Override
	protected String getAscriptionGoalString(String feature, boolean learn,
			String groundedBeliefID) {

		return VisualObjectMotiveGenerator.beliefPredicateGoal(
				getAscriptionPredicate(feature, learn), groundedBeliefID);
	}

	/**
	 * Interpreting all deletes as successes. This is will break at some point.
	 */
	@Override
	protected void motiveWasCompleted(TutorInitiativeMotive _motive) {
		println("Got back a deleted motive. Treating this as success.");
		try {
			// the intention that generated the motive
			InterpretedIntention ii = getMemoryEntry(_motive.referenceEntry,
					InterpretedIntention.class);

			cleanBelief(aboutBeliefAddress(ii));

			// and signal intention success

			// go through all types we know how to decode... can it be more
			// elegant than this?
			RichIntention decoded = AbstractDialogueActionInterface
					.extractRichIntention(ii);

			if (decoded == null) {
				getLogger().warn("Unable to decode intention",
						getLogAdditions());
				logIntention(ii);
			} else {
				CASTEffect successEffect = decoded.getOnSuccessEffect();
				successEffect.makeItSo(this);
				log("executed success effect");
			}

		} catch (SubarchitectureComponentException e) {
			logException(e);
		}

	}

}
