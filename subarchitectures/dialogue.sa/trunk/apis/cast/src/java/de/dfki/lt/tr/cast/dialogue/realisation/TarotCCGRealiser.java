package de.dfki.lt.tr.cast.dialogue.realisation;

import static scala.collection.JavaConversions.seqAsJavaList;

import java.io.File;
import java.io.IOException;
import java.util.List;

import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.synsem.SignScorer;
import scala.Option;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.nlp.realisation.ccg.CCGRealiser;
import de.dfki.tarot.nlp.realisation.openccg.NgramPrecisionModelFactory;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;

public class TarotCCGRealiser implements LFRealiser {
	
	private CCGRealiser m_realiser;
	
	public TarotCCGRealiser(String grammarFile, String ngramFile) throws IOException {
		// "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt"
		// initialize grammar-related stuff
		Grammar grammar = new Grammar(grammarFile);
		SignScorer scorer = NgramPrecisionModelFactory.fromURL(new File(ngramFile).toURI().toURL(), "UTF-8");
		m_realiser = new CCGRealiser(grammar, scorer);
	}
	

	/**
	 * This method tries to parse the lfString into a BasicLogicalForm
	 * and then invokes realiseLF. 
	 * 
	 * @param lfString
	 * @throws BuildException, ParseException if the provided lfString is not valid
	 * @return a natural language surface realisation of the lfString
	 * or the empty String in case no realisation is found
	 */
	@Override
	public String realiseString(String lfString) throws BuildException, ParseException {
		BasicLogicalForm blf = BasicLogicalForm.checkedFromString(lfString);
		return realiseLF(blf);
	}
	
	
	/**
	 * This method realises a given BasicLogicalForm into a surface String 
	 * as defined by the grammar realiser. 
	 * 
	 * @param blf
	 * @return a natural language surface realisation of the blf 
	 * or the empty String in case no realisation is found
	 */
	@Override
    public String realiseLF(BasicLogicalForm blf) {
    	StringBuilder outputBldr = new StringBuilder(); 

    	// realise thru the CCG Realizer
    	Option<scala.collection.immutable.List<String>> result = m_realiser.bestRealisationFor(blf);
    	if (result.isDefined()) {
    		List<String> words =  seqAsJavaList(result.get());
    		for (String w: words) {
    			outputBldr.append(w + " ");
    		}
    	} 
    	return outputBldr.toString();
    }

}
