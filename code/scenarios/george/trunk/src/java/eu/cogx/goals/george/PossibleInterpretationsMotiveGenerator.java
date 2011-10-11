package eu.cogx.goals.george;

import motivation.components.generators.AbstractWMEntryMotiveGenerator;
import motivation.slice.TutorInitiativeMotive;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.cast.dialogue.IntentionUnpacker;

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
		AbstractWMEntryMotiveGenerator<TutorInitiativeMotive, PossibleInterpretedIntentions> {

	public PossibleInterpretationsMotiveGenerator() {
		super(TutorInitiativeMotive.class, PossibleInterpretedIntentions.class);
	}

	@Override
	protected TutorInitiativeMotive checkForAddition(
			WorkingMemoryAddress _piiAddr, PossibleInterpretedIntentions _pii) {

		// for now, just spit out most probable intention, then return null (so
		// no motive is generated)

		try {
			println("unpacking most confident of " + _pii.intentions.size()
					+ " possible interpretations");
			IntentionUnpacker.unpackMostConfidentIntention(this, _pii);
		} catch (SubarchitectureComponentException e) {
			logException(e);
		}

		return null;

	}

	@Override
	protected TutorInitiativeMotive checkForUpdate(
			PossibleInterpretedIntentions newEntry,
			TutorInitiativeMotive existingMotive) {
		return existingMotive;
	}

}
