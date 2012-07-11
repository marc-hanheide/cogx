package eu.cogx.goals.george;

import java.util.LinkedList;
import java.util.List;

import motivation.slice.ComplexActionCommandMotive;
import motivation.slice.ObjectReferencingIntentionMotive;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import vision.execution.george.VisionActionInterface;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.RichIntention;
import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.MergedBelief;

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

	private void removeQuestionEffect(WorkingMemoryAddress reference,
			String questionedFeature) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		MergedBelief belief = getMemoryEntry(reference, MergedBelief.class);
		CASTIndependentFormulaDistributionsBelief<MergedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, belief);

		// "global-color-question-answered"
		// "object-refering-color-question-answered
		// polar-color-question-answered
		String postfix = questionedFeature + "-question-answered";
		String global = "global-" + postfix;
		String object = "object-refering-" + postfix;
		String polar = "polar-" + postfix;

		boolean overwriteNeeded = false;
		if (gb.getContent().remove(global) != null) {
			log("removed prior effect: " + global);
			overwriteNeeded = true;
		}

		if (gb.getContent().remove(object) != null) {
			log("removed prior effect: " + object);
			overwriteNeeded = true;
		}

		if (gb.getContent().remove(polar) != null) {
			log("removed prior effect: " + polar);
			overwriteNeeded = true;
		}

		if (overwriteNeeded) {
			overwriteWorkingMemory(reference, gb.get());
		} else {
			log("no prior question effects: " + questionedFeature);
		}

	}

	private void removeLearningEffect(WorkingMemoryAddress reference,
			String assertedFeature, boolean assertedLearn)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		MergedBelief belief = getMemoryEntry(reference, MergedBelief.class);
		CASTIndependentFormulaDistributionsBelief<MergedBelief> gb = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, belief);
		String learningEffect = assertedFeature;
		if (assertedLearn) {
			learningEffect += VisionActionInterface.LEARNED_FEATURE_POSTFIX;
		} else {
			learningEffect += VisionActionInterface.UNLEARNED_FEATURE_POSTFIX;
		}
		if (gb.getContent().remove(learningEffect) != null) {
			log("removed prior effect: " + learningEffect);
			overwriteWorkingMemory(reference, gb.get());
		} else {
			log("no prior effect: " + learningEffect);
		}

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

				cleanBelief(this, aboutBeliefAddress(ii), _motive);

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

}
