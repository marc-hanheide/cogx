package de.dfki.lt.tr.cast.dialogue.realisation;

import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;

public class NoRealiser extends Object implements LFRealiser {

	public NoRealiser() {
		System.err.println("Using NoRealiser -- always return empty String!");
	}

	@Override
	public String realiseString(String lfString) throws BuildException,
			ParseException {
		return "";
	}

	@Override
	public String realiseLF(BasicLogicalForm blf) {
		return "";
	}

}
