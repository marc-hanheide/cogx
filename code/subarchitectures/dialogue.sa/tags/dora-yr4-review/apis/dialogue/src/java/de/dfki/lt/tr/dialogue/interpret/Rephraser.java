package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.slice.asr.UnclarifiedPossibleInterpretedIntentions;

public interface Rephraser {

	public String rephrase(UnclarifiedPossibleInterpretedIntentions pii);

}
