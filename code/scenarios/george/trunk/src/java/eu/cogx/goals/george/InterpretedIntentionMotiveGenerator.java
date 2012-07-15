package eu.cogx.goals.george;

import java.util.LinkedList;
import java.util.List;

import motivation.slice.ComplexActionCommandMotive;
import motivation.slice.ObjectReferencingIntentionMotive;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
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
			motive = generateMotiveFromIntention(_addr, _intention, false);
			// mark referents of chosen intention
			if (motive != null
					&& motive instanceof ObjectReferencingIntentionMotive) {
				List<WorkingMemoryAddress> referenceList = new LinkedList<WorkingMemoryAddress>();
				WorkingMemoryAddress reference = aboutBeliefAddress(_intention);
				referenceList.add(reference);

				markReferent((ObjectReferencingIntentionMotive) motive,
						referenceList);

				executeAcceptEffect(this, _intention);

				if (motive instanceof TutorInitiativeLearningMotive) {
					TutorInitiativeLearningMotive tilm = (TutorInitiativeLearningMotive) motive;
					removeLearningEffect(reference, tilm.assertedFeature,
							tilm.assertedLearn);

				} else if (motive instanceof TutorInitiativeQuestionMotive) {
					TutorInitiativeQuestionMotive tiqm = (TutorInitiativeQuestionMotive) motive;
					removeQuestionEffect(reference, tiqm.questionedFeature);
				}
				monitorForObjectVisibility(_intention, motive);
			}
		} catch (CASTException e) {
			// reset in case of exception in markReferent
			motive = null;
			logException(e);
		}
		return motive;
	}

	private void monitorForObjectVisibility(InterpretedIntention _intention,
			TutorInitiativeMotive motive) {

		addChangeFilter(
				ChangeFilterFactory
						.createAddressFilter(aboutBeliefAddress(_intention)),
				new VOBeliefMonitor(motive));

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

		if (!(_motive instanceof ComplexActionCommandMotive)) {
			try {
				// the intention that generated the motive
				InterpretedIntention ii = getMemoryEntry(
						_motive.referenceEntry, InterpretedIntention.class);

				unmarkReferent(this, aboutBeliefAddress(ii), _motive);

				// and signal intention success
				executeSuccessEffect(this, ii);

			} catch (SubarchitectureComponentException e) {
				logException(e);
			}
		}

	}

}
