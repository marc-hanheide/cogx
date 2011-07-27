package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

public class CapitalizationPhonStringPreprocessor implements PhonStringPreprocessor {

	@Override
	public void apply(PhonString ps) {
		if (ps.wordSequence.equals("no")) {
			ps.wordSequence = "No";
		}
	}
	
}
