// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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
// =================================================================

package comsys.processing.parse;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------
import cast.core.CASTData;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------
import comsys.processing.parse.GrammarData;
import comsys.processing.parse.GrammarInterface;
import comsys.processing.parse.OpenCCGGrammar;
import comsys.processing.parse.OpenCCGGrammarData;
import comsys.arch.ComsysException;

// -----------------------------------------------------------------
// INTERCONNECTIVITY IMPORTS
// -----------------------------------------------------------------

import interconnectivity.processing.AttentionMechanism;
import interconnectivity.processing.ContextActiveProcess;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.io.File;
import java.net.URL;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Vector;

import org.jdom.*; 

// -----------------------------------------------------------------
// OPENCCG IMPORTS
// -----------------------------------------------------------------
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

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
The class <b>ActiveCCGLexicon</b> implements access to a CCG grammar, 
following the specifications of the <b>GrammarAccess</b> interface. The 
class provides methods for retrieving for a word a set of lexical meanings,
as specified in the grammar, possibly filtered by context information. 

@version 070814
@since	 070813
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 


public class ActiveCCGLexicon 
	implements GrammarAccess, ContextActiveProcess {


    // =================================================================
    // GLOBAL DATA STRUCTURES
    // =================================================================
	
	/** public log-level constants */ 
	public final int LOG_FALSE = 0;
	public final int LOG_TRUE  = 1;
	
	
	/** Logging parameter */
	private boolean logging = false;

	/** The OpenCCG grammar */
    private Grammar grammar;

    /** The lexicon defined in the grammar. */    
    private Lexicon lexicon;

	/** Tokenizer, to create Word objects from Strings */
	private Tokenizer tokenizer;

    // =================================================================
    // CONSTRUCTOR, CONFIGURATION METHODS
    // =================================================================

	public ActiveCCGLexicon () { 
		init(); 
	} // end constructor
	

	// Initializes the global variables
	private void init () { 
		grammar = null;
		lexicon = null;
		tokenizer = new DefaultTokenizer();
	} // end init

	/** 
	* The method <i>setGrammar</i> provides the module with information about 
	* what OpenCCG grammar to use. 
	* 
	* @param	fileName		The name of the grammar file 
	* @throws	ComsysException Thrown when there is a problem loading the grammar
	*/ 
	
	public void setGrammar (String fileName) 
		throws ComsysException 
	{
		 try {
            URL grammarURL = new File(fileName).toURL();
            log("Loading grammar from URL: " + grammarURL);
            grammar = new Grammar(grammarURL);
            if (grammar.getName() != null) {
                log("Grammar '" + grammar.getName() + "' loaded.");
				// set the access to the lexicon
				this.lexicon = grammar.lexicon; 
            }
            else {
                throw new ComsysException("Problem loading grammar in ActiveCCGLexicon: no name");
            } // end if..else check for proper grammar name
        }
        catch (Exception e) {
            throw new ComsysException("Fatal error in ActiveCCGLexicon while trying to load grammar: " + e.getMessage());
        } // end try..catch
	} // end setGrammar

	/**
		The method <i>getGrammar</i> returns the OpenCCG grammar. 
		
		@return org.cognitivesystems.comsys.data.OpenCCGGrammar The grammar
	*/ 
	
	public GrammarInterface getGrammar() { 
		return new OpenCCGGrammar(grammar); 
	} 


	/**
		The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	*/ 

	public void configure (Properties props) {  
	

	} 

    // =================================================================
    // ACTIVE PROCESSING METHODS [ ContextActiveProcess ]
    // =================================================================

	/**
	*   The method <i>getContextDataTypes</i> returns a list of
	*   CASTData types that contain context information which 
	*   can guide this process. 
	* 
	*   @return List A list of CASTData types
	*/
	public Iterator getContextDataTypes() { 
		Vector<CASTData> dataTypes = new Vector<CASTData>();
	
		return dataTypes.iterator();
	} // end getContextDataTypes
	
	/** 
	*   The method <i>updateContextData</i> gives a feed into
	*   the process to provide update context data. 
	* 
	*   @param updatedInfo A list of CASTData types with (updated) context info. 
	*/ 
	
	public void updateContextData(CASTData updatedInfo) {

	} // end updateContextData


	/**
		The method <i>pruneSignHash</i> uses context information to (possibly) 
		prune the sign hash for a word. 
		
		@param lexData	  The lexical data for the word
		@return SignHash  Possibly pruned lexical data; the result may be null. 
	*/ 

	public SignHash pruneSignHash (SignHash lexData) { 
		// Initialize the Collection to construct the pruned SignHash from
		Vector<Sign> result = new Vector<Sign>();
		for(Iterator signsIter=lexData.iterator(); signsIter.hasNext();) {
			Sign sign = (Sign)signsIter.next();
			log("Sign: "+sign.toString());
			Category cat = sign.getCategory();
			Nominal index = cat.getIndexNominal();
			log("cat.getLF: ["+cat.getLF()+"]");
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
		log("Result signhash size: "+resultSH.size());
		return resultSH;
	} // end pruneSignHash

	/**
		The method <i>registerAttentionMechanism</i> registers an attention mechanism 
		with the process, used to score (and possibly prune) input data before processing. 

		@param am The attention mechanism
	*/

	public void registerAttentionMechanism (AttentionMechanism am) { 
	

	} // end registerAttentionMechanism

	
	// string position of the word currently analysed
	int stringPos = -1;
	
	// utterance increment
	int utteranceIncrement = -1;
	
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
	
    // =================================================================
    // GRAMMAR ACCESS METHODS [ GrammarAccess ]
    // =================================================================

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
		throws ComsysException {
		if (lexicon == null) { 
			throw new ComsysException("Null lexicon in ActiveCCGLexicon!"); 
		} // end if check for non-null lexicon
		OpenCCGGrammarData results = new OpenCCGGrammarData();
		try { 
			lexicon.setWordPosition(stringPos);
			lexicon.setUtteranceIncrement(utteranceIncrement);
			SignHash wordHash = (SignHash)lexicon.getSignsFromWord(tokenizer.parseToken(word));
			log("Returned wordHash size: "+wordHash.size());
			SignHash prunedWSH = pruneSignHash(wordHash);
			results.signHash = prunedWSH;
		} 
		catch (LexException e) { 
			throw new ComsysException(e.toString());
		} // end try..catch
		return results;
	} // end getLexicalEntries	

    // =================================================================
    // LOG METHODS [ GrammarAccess ]
    // =================================================================

	/** 
		The method <i>setLogLevel</i> defines the logging level for the parser: 
		<ol>
		<li value="0"> no logging </li>
		<li value="1"> logging on system.out</li>
		</ol>
		To set the log level, it is best to use the public constants <tt>LOG_FALSE</tt>, <tt>LOG_TRUE</tt>. 
	
		@param l The log level 
	*/ 

	public void setLogLevel (int l) {
		logging = false; 
		switch (l) {
			case 0 : break; 
			case 1 : logging = true; break ; 
			case 2 : logging = true; break;
		} // end switch
	} //

	public void log (String msg) { 
		if (logging) { System.out.println("[ActiveCCGLexicon] "+msg); }
	} // end log



} // end class