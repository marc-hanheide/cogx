package eu.cogx.goals.george;

import motivation.slice.TutorInitiativeMotive;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import eu.cogx.beliefs.slice.GroundedBelief;

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

	@Override
	protected void motiveWasDeleted(TutorInitiativeMotive _motive) {
		log("Got back a deleted motive");
		try {
			// the intention that generated the motive
			InterpretedIntention ii = getMemoryEntry(_motive.referenceEntry,
					InterpretedIntention.class);

			cleanBelief(ii.addressContent.get("about"));

		} catch (SubarchitectureComponentException e) {
			logException(e);
		}

	}

	protected void cleanBelief(WorkingMemoryAddress _groundedBeliefAddr)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		
		GroundedBelief belief = getMemoryEntry(_groundedBeliefAddr,
				GroundedBelief.class);
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, belief);

		//remove marking for reference
		unmarkReferent(gb);
		//remove potential results of learning and dialoguate
		removeActionEffects(gb);
		
		overwriteWorkingMemory(_groundedBeliefAddr, gb.get());
	}


}
