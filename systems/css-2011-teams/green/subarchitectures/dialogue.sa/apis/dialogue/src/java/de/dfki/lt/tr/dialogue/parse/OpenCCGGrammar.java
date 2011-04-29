// =================================================================
// Copyright (C) 2010 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
// IMPORTS

// Dialogue API 
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException; 

// Java
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.Iterator;
import java.util.Properties;
import java.util.Vector;

// OpenCCG
import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.lexicon.DefaultTokenizer;
import opennlp.ccg.lexicon.Lexicon;
import opennlp.ccg.lexicon.LexException;
import opennlp.ccg.lexicon.Tokenizer;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;

	/**
	 * The class <tt>OpenCCGGrammar</tt> provides a wrapper around the
	 * OpenCCG framework-specific representation of a grammar. 
	
	 * @version  100608
	 * @since	 070820
	 * @author	 Geert-Jan M. Kruijff (gj@dfki.de)
	*/ 

public class OpenCCGGrammar 
implements GrammarInterface, GrammarAccess
{

	// =================================================================
	// GLOBAL DATA STRUCTURES
	// =================================================================

	public opennlp.ccg.grammar.Grammar ccggrammar = null;

	// string position of the word currently analysed
	int stringPos = -1;
	
	// utterance increment
	int utteranceIncrement = -1;
	
	Tokenizer tokenizer = new DefaultTokenizer();
	
	// =================================================================
	// CONSTRUCTOR
	// =================================================================

	public OpenCCGGrammar (opennlp.ccg.grammar.Grammar g) { 
		ccggrammar = g;
	} // end constructor

	
	public OpenCCGGrammar () {	
	} // end constructor
	
	public OpenCCGGrammar (String fn)
	throws DialogueMissingValueException, IOException
	{
		this.setGrammar(fn);
	} // end constructor
	
	public Grammar getGrammar () { 
		return ccggrammar; 
	}
	
	public void setGrammar (String filename)
	throws DialogueMissingValueException, IOException 
	{
		if (filename == null || filename.equals(""))
		{
			throw new DialogueMissingValueException("Cannot set grammar: filename null or empty");
		}
		try {
			URL grammarURL = new File(filename).toURL();
			ccggrammar = new Grammar(grammarURL);
		}
		catch (Exception e) {
			throw new IOException("Cannot set grammar: " + e.getMessage());
		} // end try..catch
	} // end 
	
	/** 
	* The method <i>getLexicalEntries</i> provides the most basic type of access, 
	* namely retrieving grammar information about individual words. The entries are
	* stored as <b>(Object)SignHash</b> in the results. 
	* 
	* @param	word			A word to provide data about
	* @return	GrammarData		Information in the grammar about the word, cast from OpenCCG
	* @throws	ComsysException Thrown when there is a problem accessing the grammar
	*/ 
	
	public GrammarData getLexicalEntries (String word) 
		throws DialogueMissingValueException {
		if (ccggrammar == null) { 
			throw new DialogueMissingValueException("Cannot get lexical entries: Lexicon is null"); 
		} // end if check for non-null lexicon
		OpenCCGGrammarData results = new OpenCCGGrammarData();
		try { 
			Lexicon lexicon = ccggrammar.lexicon;
			lexicon.setWordPosition(stringPos);
			lexicon.setUtteranceIncrement(utteranceIncrement);
			SignHash wordHash = (SignHash)lexicon.getSignsFromWord(tokenizer.parseToken(word));
			SignHash prunedWSH = pruneSignHash(wordHash);
			results.signHash = prunedWSH;
		} 
		catch (LexException e) { 
			throw new DialogueMissingValueException(e.toString());
		} // end try..catch
		return results;
	} // end getLexicalEntries	
	
	/**
	 * The method <i>pruneSignHash</i> uses context information to (possibly) 
	 * prune the sign hash for a word. 
	 * @param lexData	  The lexical data for the word
	 * @return SignHash  Possibly pruned lexical data; the result may be null. 
	 */ 

	public SignHash pruneSignHash (SignHash lexData) { 
		// Initialize the Collection to construct the pruned SignHash from
		Vector<Sign> result = new Vector<Sign>();
		for(Iterator signsIter=lexData.iterator(); signsIter.hasNext();) {
			Sign sign = (Sign)signsIter.next();
			Category cat = sign.getCategory();
			Nominal index = cat.getIndexNominal();
			if (cat.getLF() != null) {
				LF lf = HyloHelper.compactAndConvertNominals(
						cat.getLF(),
						index);
				// now start pruning ... 								
				// ... at the moment, no pruning, just add the sign
				if (true) { 
					result.addElement(sign);
				} // end if.. check whether to add the sign
			} else { 
				// else we probably have an oblique or so
				result.addElement(sign);				
			} // end if..check for LF, we can only prune real meanings ... 
		} // end for over the categories in the SignHash
		// return a new SignHash on the basis of the Vector-Collection of filtered Signs
		SignHash resultSH = new SignHash(result);
		return resultSH;
	} // end pruneSignHash
	
	
	/**
	 * Set the word position of the word currently analysed
	 * @param stringPos
	 */
	public void setWordPosition(int stringPos) {
		this.stringPos = stringPos;
	}
	
	/**
	 * Set the word position of the word currently analysed
	 * @param stringPos
	 */
	public void setUtteranceIncrement(int utteranceIncrement) {
		this.utteranceIncrement = utteranceIncrement;
	}
	
	/**
	 * The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	 */ 

	public void configure (Properties props) {  


	} 
	
} // end class
