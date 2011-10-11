package eu.cogx.goals.george;

import motivation.slice.TutorInitiativeMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;

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
			motive = generateMotiveFromIntention(_addr, _intention);
			// mark referents of chosen intention
			markReferent(_intention.addressContent.get("about"));
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

		String goal = VisualObjectMotiveGenerator.beliefPredicateGoal(
				predicate, _groundedBeliefAddr.id);
		return goal;
	}

}
