package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.util.CogXLexicon;

public class ParaphrasingPhonStringPreprocessor
implements PhonStringPreprocessor {

	@Override
	public void apply(PhonString ps) {
		if (ps.wordSequence.equals("it is")) {
			ps.wordSequence = "yes";
		}
		if (ps.wordSequence.equals("it is not")) {
			ps.wordSequence = "no";
		}

                ps.wordSequence = CogXLexicon.simplifyObjectTypes(ps.wordSequence);
	}
	
}
