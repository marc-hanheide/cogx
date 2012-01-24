package de.dfki.lt.tr.dialogue.parse.preprocess;

import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import java.util.Iterator;
import java.util.LinkedList;

public class BasicPhonStringPreprocessor implements PhonStringPreprocessor {

	@Override
	public void apply(PhonString ps) {
		String[] words = ps.wordSequence.split(" +");
		LinkedList<String> passed = new LinkedList<String>();
		boolean mainBody = false;

		for (int i = 0; i < words.length; i++) {
			if (!mainBody && (words[i].equals("robot") || words[i].equals("and"))) {
				// okay, ignore this one
			}
			else {
				if (words[i].equals("base")) {
					if (i > 0 && !words[i-1].equals("the")) {
						passed.add("the");
					}
					passed.add("entrance");
				}
				else {
					mainBody = true;
					passed.add(words[i]);
				}
			}
		}

		String result = "";
		Iterator<String> iter = passed.iterator();
		while (iter.hasNext()) {
			result += iter.next();
			result += (iter.hasNext() ? " " : "");
		}

		ps.wordSequence = result;
	}

}
