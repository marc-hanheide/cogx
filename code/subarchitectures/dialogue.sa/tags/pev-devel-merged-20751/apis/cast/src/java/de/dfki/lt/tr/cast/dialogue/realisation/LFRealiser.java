package de.dfki.lt.tr.cast.dialogue.realisation;

import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;

public interface LFRealiser {

	/**
	 * This method tries to parse the lfString into a BasicLogicalForm
	 * and then invokes realiseLF. 
	 * 
	 * @param lfString
	 * @throws BuildException, ParseException if the provided lfString is not valid
	 * @return a natural language surface realisation of the lfString
	 * or the empty String in case no realisation is found
	 */
	public String realiseString(String lfString) throws BuildException, ParseException;

	/**
	 * This method realises a given BasicLogicalForm into a surface String 
	 * as defined by the grammar realiser. 
	 * 
	 * @param blf
	 * @return a natural language surface realisation of the blf 
	 * or the empty String in case no realisation is found
	 */
    public String realiseLF(BasicLogicalForm blf);
	
}
