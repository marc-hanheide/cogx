package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

public class CapitalizationPhonStringPreprocessor implements PhonStringPreprocessor {

	@Override
	public void apply(PhonString ps) {
		String newWordSequence = ps.wordSequence;

		newWordSequence = newWordSequence.toLowerCase();

		if (newWordSequence.equals("no")) {
			newWordSequence = "No";
		}
		ps.wordSequence = newWordSequence;
	}
	
}
