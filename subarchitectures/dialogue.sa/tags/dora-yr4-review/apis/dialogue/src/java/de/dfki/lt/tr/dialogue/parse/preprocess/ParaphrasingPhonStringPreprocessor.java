package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import java.util.HashMap;
import java.util.Map;

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

                Map<String, String> expansions = new HashMap<String, String>();
                expansions.put("tea box", "teabox");
                expansions.put("cereal box", "cerealbox");
                expansions.put("drink can", "drinkcan");
                
                for (Map.Entry<String, String> exp : expansions.entrySet()) {
                    ps.wordSequence = ps.wordSequence.replaceFirst(exp.getKey(), exp.getValue());
                }
                
	}
	
}
