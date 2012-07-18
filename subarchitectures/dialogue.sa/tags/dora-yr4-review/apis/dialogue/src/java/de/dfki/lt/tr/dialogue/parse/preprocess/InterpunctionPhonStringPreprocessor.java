package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

public class InterpunctionPhonStringPreprocessor implements PhonStringPreprocessor {

	@Override
	public void apply(PhonString ps) {
		ps.wordSequence = ps.wordSequence.replaceAll("[.,!?]", "");
	}
	
}
