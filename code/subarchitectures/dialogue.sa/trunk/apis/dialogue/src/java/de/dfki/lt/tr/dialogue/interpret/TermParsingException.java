package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;

public class TermParsingException extends Exception {

	public TermParsingException(String expected, Term t) {
		super("unable to parse " + PrettyPrint.termToString(t) + " to " + expected);
	}
	
}
