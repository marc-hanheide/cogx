//=================================================================
//Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
//=================================================================

package comsys.processing.parse;

//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
//CAST IMPORTS
//-----------------------------------------------------------------
import cast.core.CASTData;

//-----------------------------------------------------------------
//COMSYS IMPORTS
//-----------------------------------------------------------------

import comsys.ContextInfo;
import comsys.InterpretationSupport;
import comsys.PhonString;
import comsys.components.parse.cc_Parser;
import comsys.processing.parse.GrammarInterface;
import comsys.processing.parse.OpenCCGGrammar;
import comsys.processing.parse.OpenCCGGrammarData;
import comsys.processing.parse.PackedLFParseResults;
import comsys.processing.asr.WordRecognitionLattice;
import comsys.processing.parse.SignHashParseResults;
import comsys.arch.ComsysException;

//-----------------------------------------------------------------
//INTERCONNECTIVITY IMPORTS
//-----------------------------------------------------------------
import interconnectivity.processing.AttentionMechanism;
import interconnectivity.processing.AttentionMechanismPipeline;
import interconnectivity.processing.ContextActiveProcess;

//-----------------------------------------------------------------
//JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Calendar;
import java.util.Date;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Stack;
import java.util.Vector;

//-----------------------------------------------------------------
//OPENCCG IMPORTS
//-----------------------------------------------------------------

import opennlp.ccg.grammar.RuleGroup;
import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.lexicon.DefaultTokenizer;
import opennlp.ccg.lexicon.LexException;
import opennlp.ccg.lexicon.Tokenizer;
import opennlp.ccg.lexicon.Word;
import opennlp.ccg.parse.Chart;
import opennlp.ccg.parse.ChartScorer;
import opennlp.ccg.parse.DerivationHistory;
import opennlp.ccg.parse.IncrCKYParser;
import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;
import opennlp.ccg.unify.UnifyControl;

//-----------------------------------------------------------------
//REPRESENTATION IMPORTS
//-----------------------------------------------------------------
import lf.*;
import lf.utils.ArrayIterator;
import lf.utils.LFPacking;
import lf.utils.LFUtils;
import cast.core.CASTUtils;
import java.util.Enumeration;

//=================================================================
//CLASS DOCUMENTATION
//=================================================================


/**
The class <b>ActiveIncrCCGParser</b> implements an environment around
an incremental chart parser for OpenCCG-style CCG grammars. The 
class maintains all the data structures, such as charts, produced by 
the parser. The class controls when the parser can / should take its
next step, and initializes the chart with which the parser works for
that next step. 
<p>
Due to the asynchronous nature in which components work, it is possible 
that the parser would like to make a next step while at the same time
the class is updating the latest chart on the basis of context-information. 
Currently, no blocking-constraints have been built in. 

@version 071031
@since	 070813
@author Geert-Jan M. Kruijff (gj@dfki.de)

 */ 


public class ActiveIncrCCGParser 
extends IncrGrammaticalInference
implements ContextActiveProcess {

	// =================================================================
	// GLOBAL DATA STRUCTURES
	// =================================================================

	/** public log-level constants */ 
	public final int LOG_FALSE = 0;
	public final int LOG_TRUE  = 1;
	public final int LOG_VISUALIZE = 2;

	/** Logging parameter */
	private boolean logging = false;
	private boolean visualization = false; 

	// last analysed utterance
	String lastUtterance = "";

	// Increment on the number of utterances
	int utteranceIncrement = 0;

	// The actual parser
	private opennlp.ccg.parse.IncrCKYParser parser; 

	// Charthistory is a collection of stacks with charts, the current 
	// chart is on top. A stack is keyed by the ID of the PhonString for 
	// which it maintains the (partial) analyses  
	private Hashtable<String,Stack> chartHistories;

	// Hashtable with for each phon string (keyed by id) the final packed logical form
	private Hashtable<String,PackedLogicalForm> finalPackedLFs; 

	// table with the string position for each phonString being parsed 
	private Hashtable<String,Integer> phonStringPositions;

	// table with the tokenized phonStrings being parsed
	private Hashtable<String,List> phonStringTokens;

	private Hashtable<String,Boolean> phonStringFinished;
	
	// table matching plf id's with phon string id's
	private Hashtable<String,String> plfToPhonStringId; 

	// The interpretation support arrays, stored as a Vector per phon string id
	private Hashtable<String,Vector> interpretationSupport; 


	// Access to the grammar is factorized out
	protected GrammarAccess grammar;

	// Attention mechanism
	protected AttentionMechanism attmech; 

	// The tokenizer.  (Defaults to DefaultTokenizer.) 
	protected Tokenizer tokenizer;

	// The LFPacking factory 
	protected LFPacking packingTool; 

	/**
	 * The function to score categories / signs before an LFCollection
	 * is constructed from parsing results
	 */
	ChartScorer chartScorer;

	/** Values for the flag indicating finalized status */ 
	public static final int NOTFINISHED = 0; 
	public static final int FINAL_PARSE = 1;
	public static final int FINAL_PRUNING = 2;

	/** Timing data, Long type */
	Hashtable<String,Vector> timingMap; 
	Vector timing;

	private String stateSemaphore; 

	private final String FREE_STATE    = "freeState";
	private final String UPDATE_STATE  = "updateState";
	private final String PARSING_STATE = "parsingState";	

	cc_Parser component;

	// set of dependencies whose interpretation is known to be unsupported
	private Vector<InterpretationSupport> unsupportedDependencies ;

	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	public ActiveIncrCCGParser () { 
		init();
	} // end constructor

	private void init () { 
		grammar = null;
		chartHistories = new Hashtable<String,Stack>();
		phonStringPositions = new Hashtable<String,Integer>();
		phonStringTokens = new Hashtable<String,List>();
		phonStringFinished = new Hashtable<String,Boolean>();
		plfToPhonStringId = new Hashtable<String,String>();
		attmech = new AttentionMechanismPipeline();
		tokenizer = new DefaultTokenizer();
		packingTool = new LFPacking();
		// packingTool.logging = logging;
		//	packingTool.logging = false;		
		chartScorer = null;
		timingMap = new Hashtable<String,Vector>();
		timing = new Vector();
		finalPackedLFs = new Hashtable<String,PackedLogicalForm>();
		interpretationSupport = new Hashtable<String,Vector>();
		stateSemaphore = FREE_STATE;
		component = null;
	} // end init

	// =================================================================
	// CONFIGURATION METHODS
	// =================================================================

	/** Registers the owning component, for CAST interactions like storing objects on WM.*/ 

	public void registerComponent (cc_Parser c) { 
		component = c;
	} // end register component


	/**
	 * The method <i>registerGrammarAccess</i> registers an object implementing <b>GrammarAccess</b> with the grammatical inference engine, 
	 * so that it is able to obtain grammatical information for words (etc.).  
	 * 
	 * @param	lg	A class implementing the GrammarAccess interface, to provide access to the grammar
	 * @throws	ComsysException Thrown when there is a problem connecting to the grammar
	 */	

	public void registerGrammarAccess(GrammarAccess lg) 
	throws ComsysException {
		if (lg != null) { 
			grammar = lg;
			parser = new IncrCKYParser(((OpenCCGGrammar)grammar.getGrammar()).ccggrammar);
			log("Initialized the incremental parser with the grammar");		
		} else { 
			throw new ComsysException("Trying to register null object for grammar access in ActiveIncrCCGParser");
		} // end if..else check for non-null grammar access
	} // end registerGrammarAccess

	/**
		The method <i>registerChartyScorer</i> registers a function which prunes signs from the sign-hash in the top cell of the chart. 
		This is step is taken after each parsing step, before we check whether to continue parsing.  

		@param shs The chart scorer to be registered
	 */ 

	public void registerChartScorer (ChartScorer shs) { 
		chartScorer = shs; 
	} // end registerChartScorer


	/**
		The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	 */ 

	public void configure (Properties props) {  


	} 

	// =================================================================
	// ACCESSOR METHODS
	// =================================================================

	/** 
		The method <i>getChartHistory</i> returns the Stack of Charts for
		the given PhonString id. Should there be no such stack, then <tt>null</tt> 
		is returned. 

		@param  id	  The identifier of the (parsed) PhonString
		@return Stack Stack of charts for the given PhonString
	 */ 

	public Stack getChartHistory (String id) { 
		if (chartHistories.containsKey(id)) { 
			return (Stack) chartHistories.get(id);
		} else { 
			return null;
		} // end if..else check for presence id
	} // end getChartHistory

	/** Returns the size of the given chart */

	public int getChartSize (opennlp.ccg.parse.Chart table) { return parser.getChartSize(table); }

	/** The method <i>getFinalPackedLF</i> returns the final packed logical form for the given phon string id, 
		or <tt>null</tt> if the id is unknown. 

		@param id The identifier of the phon string
		@return PackedLogicalForm The packed logical form for the final (complete) analyses of the phon string
	 */ 

	public PackedLogicalForm getFinalPackedLF (String id) { 
		if (finalPackedLFs.containsKey(id)) { 
			return (PackedLogicalForm) finalPackedLFs.get(id);
		} else { 
			return null;
		} // end if..else check for availability of id
	} // end getFinalPackedLF


	/** 
		The method <i>getTiming</i> returns a Vector with Long objects, 
		each Long representing the number of milliseconds it took to 
		parse up to (and including) the given string position (corresponding
		to the Vector position). The method returns <tt>null</tt> if there is
		no timing data for the PhonString with the given id. 

		@param  id		The id of the PhonString for which the timing data is stored
		@return Vector	A vector of Long objects (timing in ms). 
	 */ 

	public Vector getTiming (String id) { 
		if (timingMap.containsKey(id)) { 
			return (Vector) timingMap.get(id);
		} else { 
			return null;
		} // end if..else check for id 
	} // end getTiming

	// =================================================================
	// ACTIVE PROCESSING METHODS [ ContextActiveProcess ]
	// =================================================================

	/**
	 *   The method <i>getContextDataTypes</i> returns a list of
	 *   CASTData types (String) that contain context information which 
	 *   can guide this process. 
	 * 
	 *   @return List A list of CASTData types
	 */
	public Iterator getContextDataTypes() { 
		Vector<String> dataTypes = new Vector<String>();
		dataTypes.addElement(CASTUtils.typeName(ContextInfo.class));
		return dataTypes.iterator();
	} // end getContextDataTypes

	/** 
	 *   The method <i>updateContextData</i> gives a feed into
	 *   the process to provide update context data. At the moment, what the 
	 *   method does is that it stores the supported interpretations, which are
	 *   used as a post-filter on the PLF produced at each step -- to prune it. 
	 *   The reason for currently doing this is that it is the mechanism (pruning
	 *   the PLF directly) which we want to port into OpenCCG. 
	 * 
	 *   @param updatedInfo A list of CASTData types with (updated) context info. 
	 */ 

	public void updateContextData(CASTData updatedInfo) {
		log("Received context data of type ["+updatedInfo.getType()+"]");
		// Get the data out 
		String dataType = updatedInfo.getType(); 
		log(dataType);
		log(CASTUtils.typeName(ContextInfo.class));
		if (dataType.equals(CASTUtils.typeName(ContextInfo.class))) { 
			ContextInfo contextInfo = (ContextInfo) updatedInfo.getData();
			// Get the packed logical form 
			String strId = plfToPhonStringId.get(contextInfo.plfId);
			PackedLogicalForm plf = finalPackedLFs.get(strId);
			if (plf != null) { 
				if (contextInfo.interpretations != null) { 
					log("May need to wait to acquire state for updating");
					// Make sure that we can update the chart
					while (!stateSemaphore.equals(FREE_STATE)) { 
						wait(20); 
					} // end while
					log("State for updating acquired, blocking other processing");						
					// Block processing till the chart has been pruned
					stateSemaphore = UPDATE_STATE;
					// store the interpretation support for the phonString
					Vector supportArrays = new Vector();
					if (interpretationSupport.containsKey(strId)) { 
						supportArrays = (Vector) interpretationSupport.get(strId);
					} // end if.. check whether there are already arrays
					supportArrays.addElement(contextInfo.interpretations);
					interpretationSupport.put(strId,supportArrays);

					// check the finalized status; if we're finished parsing, 			
					// we need to call the final pruning step separately, 
					// then set the finalized status of the packed lf, 
					// and finally store the finalized one. 
					int length = ((List)phonStringTokens.get(strId)).size();															
					int stringPos = ((Integer)phonStringPositions.get(strId)).intValue();
					if (stringPos >= length-1) { 
						log(">>>> NEED TO UPDATE A FINISHED PARSE <<<<<");
						plf = prunePackedLogicalForm(plf,strId);
						log("Pruning PLF -- number of packing nodes after pruning unsupported dependencies: ["+plf.pNodes.length+"]");					
						log(">>>> STORING THE PACKED LF ON WM <<<<<");						
						component.storePackedLF(plf,2);
					} // 
					// Release	the semaphore
					log("Releasing processing state");												
					stateSemaphore = FREE_STATE;
				} // end check for there being interpretations
			} else {
				System.err.println("[ERR \"ActiveIncrCCGParser\"] Could not retrieve PLF with id ["+contextInfo.plfId+"] for update context data");
			} // end if..else check for PLF
		} // end if.. check for context info
	} // end updateContextData  

	/**
		The method <i>registerAttentionMechanism</i> registers an attention mechanism 
		with the process, used to score (and possibly prune) input data before processing. 

		@param am The attention mechanism
	 */

	public void registerAttentionMechanism (AttentionMechanism am) { 
		attmech = am;
	} // end registerAttentionMechanism


	// =================================================================
	// EXECUTION METHODS
	// =================================================================

	/** 
	 * The method <i>parse</i> parses the given PhonString, and returns the resulting analyses. 
	 *
	 * @param	str				The PhonString object with the utterance to be parsed
	 * @return	ParseResults	The resulting analyses, provided as PackedLFParseResults
	 * @throws	ComsysException	Thrown when a parsing error occurred	
	 * 
	 * @see		PackedLFParseResults
	 * @see		org.cognitivesystems.comsys.data.ParseResults
	 */ 

	public ParseResults parse (PhonString str) 
	throws ComsysException, ParseException {
		//	setLogLevel(1);
		unsupportedDependencies = new Vector<InterpretationSupport>();
		// initialize return result
		PackedLFParseResults results = new PackedLFParseResults();
		// tokenize the phonString
		List words = tokenizer.tokenize(str.wordSequence);
		// Run incremental parse steps over the length of the sentence

	       try {
	            UnifyControl.startUnifySequence();
	            Sign.resetCatInterner(false);
	            List entries = parser.lexicon.getEntriesFromWords(str.wordSequence);
	            //   List entries = grammar.getLexicalEntries(str.wordSequence);

	            log("Length: " + entries.size());
	    		// initialize the chart and chart history
	    		opennlp.ccg.parse.Chart table = new opennlp.ccg.parse.Chart(entries.size(), ((OpenCCGGrammar)grammar.getGrammar()).ccggrammar.rules);
	    		
				int i = 0;
				for (Iterator entryIt=entries.iterator(); entryIt.hasNext(); i++) {
					SignHash wh = (SignHash)entryIt.next();
					for(Iterator whI=wh.iterator(); whI.hasNext();) {
						Category cat = ((Sign)whI.next()).getCategory();
						//cat.setSpan(i, i);
						UnifyControl.reindex(cat);
					}
					table.set(i,i,wh);
				}
				parser.parse(table, entries.size());
				if (chartScorer != null) {
					table = chartScorer.score(table, entries.size()-1);
				}
				results.stringPos = entries.size()-1;
				results = packChartAnalyses(results, table);
				results.finalized = this.FINAL_PARSE;
	            
	        } catch (ParseException e) {
				log(e.getMessage());
			}
	        catch (Exception e) {
	            e.printStackTrace();
	        }
	        
	        results.phon2LFsMapping = new Hashtable<PhonString,Vector<String>>();
	        results.phon2LFsMapping.put(str, new Vector<String>());
	        if (results != null && results.plf != null) {
				LFPacking packingTool2 = new LFPacking();
				LogicalForm[] unpackedLFs = packingTool2.unpackPackedLogicalForm(results.plf);			
				Vector<String> vec = results.phon2LFsMapping.get(str);
				for (int i = 0; i < unpackedLFs.length; i++) {		
					vec.add(unpackedLFs[i].logicalFormId);
				}
	        }
		return (ParseResults)results;
	} // end parse




/** 
 * The method <i>incrParse</i> parses the given PhonString, takes the next incremental
 * step in parsing the PhonString (either initialize, or continue), and returns the 
 * resulting analyses. If we're already at the end of the sentence, then we just perform
 * a final pruning step. The method sets the <tt>finalized</tt> flag for the <tt>PackedLFs</tt>
 * struct: if we're at the end, we're FINAL_PARSE (1), if after that we perform the last 
 * pruning, we're FINAL_PRUNING (2). By default, we're NOTFINISHED (0). 
 *
 * @param	str				The PhonString object with the utterance to be parsed
 * @return	ParseResults	The resulting analyses
 * @throws	ComsysException	Thrown when <tt>start</tt> &lt; 0 or end &gt; the end of the utterance, or when any other parsing error occurred
 * @see		org.cognitivesystems.comsys.data.ParseResults
 */ 	
@Override
public ParseResults incrParse (PhonString str) 
throws ComsysException, ParseException { 

	// declare the stringpos, chart, initialization as per below
	int stringPos = -1;		
	if (!str.wordSequence.equals(lastUtterance)) {
		lastUtterance = str.wordSequence;
		utteranceIncrement++;
	}
	boolean continueParsing;
	Calendar rightNow;
	opennlp.ccg.parse.Chart chart = null;
	Stack chartHistory = null;
	List words = null;
	// the results
	PackedLFParseResults results = new PackedLFParseResults();
	results.finalized = this.NOTFINISHED;
	// check whether we've already started to parse the string
	// if so, initialize the string position and chart accordingly
	if (phonStringPositions.containsKey(str.id)) { 
		// get the string position (last analyzed position) and increment 
		stringPos = ((Integer)phonStringPositions.get(str.id)).intValue();
		// get the chart history for this phonString

		chartHistory = (Stack) chartHistories.get(str.id);
		chart = (opennlp.ccg.parse.Chart)chartHistory.peek();
		// this is the point where we should prune the chart using context information 
		
		if (phonStringFinished.containsKey(str.id) && 
				phonStringFinished.get(str.id)) {
			return null;
		}

		// get the list of words
		words = (List)phonStringTokens.get(str.id);
		// get the timing vector
		timing = (Vector) timingMap.get(str.id);
	} else { 
		// tokenize the phonString, store the resulting tokenization for later reference
		words = tokenizer.tokenize(str.wordSequence);
		phonStringTokens.put(str.id,words);
		phonStringFinished.put(str.id, new Boolean(false));
		
		// initialize the chart and chart history
		chart = new opennlp.ccg.parse.Chart(words.size(), ((OpenCCGGrammar)grammar.getGrammar()).ccggrammar.rules);
		chartHistory = new Stack();
		// initialize the unification control of the parser
		parser.initUnification();
		// initialize the timing vector
		timing = new Vector();
		// reset the unsupported dependencies

		unsupportedDependencies = new Vector<InterpretationSupport>();
	} // if...else for  retrieving, initializing string position, chart
	// now, loop until a frontier is reached
	try { 
		// checkk whether we are at the end already
		if (stringPos < words.size()-1) { 
			log("Not yet at the end of the utterance, continue parsing");
			continueParsing = true;
			while (continueParsing) { 
				rightNow = Calendar.getInstance();
				long startTime = rightNow.getTime().getTime();
				// move the string position one step forward
				stringPos++;
				// retrieve the lexical data for the word at the current position
				Word word = (Word)words.get(stringPos);

				// We send some info about the current word position to the grammar
				// (used to forge unique identifiers for the nominals)
				grammar.setWordPosition(stringPos);
				grammar.setUtteranceIncrement(utteranceIncrement);

				SignHash wordSigns = ((OpenCCGGrammarData)grammar.getLexicalEntries(word.getForm())).signHash;
				// log("Getting signs for the word at position "+stringPos+" ["+word.toString()+"] -- "+wordSigns.size()+" signs found"); 				

				// update the chart with the lexical data, re-indexing to provide unique indices 
				for(Iterator signsIter=wordSigns.iterator(); signsIter.hasNext();) {
					Category cat = ((Sign)signsIter.next()).getCategory();
					UnifyControl.reindex(cat);
				} // end for over categories to reindex

				chart.set(stringPos,stringPos,wordSigns);
				// log("Chart after lexical insertion:\n"+chart.toString());
				// initialize the parser, take a step, get the 

				try {
					chart = parser.stepIncrParser(chart,0,stringPos+1);
					// log("Chart after parsing step:\n"+chart.toString());	
				}
				catch (ParseException e) {
					log(e.getMessage());
				}

				SignHashParseResults topCell = new SignHashParseResults(chart.get(0,stringPos));
				// log("Top cell size before pruning: "+topCell.hash.size());
				// prune the chart using a SignScorer over sign hashes.
				if (chartScorer != null) {
					// apply the scorer and get the revised chart
					/**
						SignHash prunedTopCellHash = signHashScorer.score(topCell.hash,complete);
						log("Top cell size after pruning: "+prunedTopCellHash.size());					
						// set the SignHash in the topcell of the chart
						// to the revised hash
						chart.set(0, stringPos, prunedTopCellHash);
						topCell.hash = prunedTopCellHash;
					 */
					//log(">>>> START scoring the chart <<<<");
					chart = chartScorer.score(chart,stringPos);

					//log("Chart after pruning step:\n"+chart.toString());				
					//log("<<<< END scoring the chart >>>>");						

				} // end if check for chart scorer
				// check whether to continue, on the top cell hash at position (0,stringPos) 
				if (stringPos == (words.size()-1)) { 
					log("Stop parsing, end of the utterance");
					continueParsing = false; 
					results.finalized = this.FINAL_PARSE;

				} else { 
					continueParsing = !a_iffilter.eligibleFrontierReached(topCell);
					results.finalized = this.NOTFINISHED;
				} 
				log("Continue parsing? ["+continueParsing+"]");
				rightNow = Calendar.getInstance();
				long endTime = rightNow.getTime().getTime();
				timing.add(stringPos,new Long(endTime-startTime));
			} // end while 
		} else { 
			log("At the end of the utterance, need to wait on interpretation support to perform final pruning steps");
			results.finalized = this.FINAL_PRUNING;
			// do the final pruning
		} // end if..else check for whether we've reached the end already

		// store the chart on the chart history, store the history and the string position for this phonString
		chartHistory.push(chart);
		chartHistories.put(str.id,chartHistory);
		phonStringPositions.put(str.id,new Integer(stringPos));
		timingMap.put(str.id,timing);

		// return the packed results
		results.setStringPosition(stringPos);
		results = packChartAnalyses(results, chart);

		// now do the post pruning on the PLF 
		PackedLogicalForm prunedPLF = prunePackedLogicalForm(results.plf,str.id);

		results.plf = prunedPLF;
		if (results.plf != null) { 
			finalPackedLFs.put(str.id,results.plf);
			plfToPhonStringId.put(results.plf.packedLFId,str.id);
			log("Storing link between packed LF id ["+results.plf.packedLFId+"] and phonstring id ["+str.id+"]");
		} // end check for final packed LF
		else {
			System.err.println("[WARNING:ActiveIncrParser] phonString not parsable: \"" + str.wordSequence + "\"");
			phonStringFinished.put(str.id, new Boolean(true));
		}
		log("Finalized flag for the PLF: "+results.finalized);

	} 
	catch (IndexOutOfBoundsException iobe) { 
		throw new ComsysException (iobe.getMessage()); 
	} // end try..catch

	return results;
} // end parse


/** 
		The method <i>prunePackedLogicalForm</i> checks whether relations in the packed logical form 
		are supported or not, and if not, then they are deleted. Relations cover both LFRelation and 
		PackingEdge objects. 
 */ 

public PackedLogicalForm prunePackedLogicalForm (PackedLogicalForm plf, String strId) { 
	// Initialize the result
	PackedLogicalForm result = plf;
	
	if (interpretationSupport.containsKey(strId)) { 
		// Get the supported interpretations
		Vector interpretations = interpretationSupport.get(strId);
		for (Iterator suppintIter = interpretations.iterator(); suppintIter.hasNext(); ) { 
			InterpretationSupport[] supports = (InterpretationSupport[]) suppintIter.next();
			for (ArrayIterator supportsIter = new ArrayIterator(supports); supportsIter.hasNext(); ) {
				InterpretationSupport support = (InterpretationSupport) supportsIter.next();
				if (!support.isSupported) {
					log("Unsupported relation ["+support.mode+"] under nominal ["+support.headNomVar+"] ");
					log("Pruning PLF -- number of packing nodes before pruning unsupported dependencies: ["+result.pNodes.length+"]");
					result = LFUtils.plfRemoveDependence(result,support.headNomVar, support.depNomVar, support.mode); 
					log("Pruning PLF -- number of packing nodes after pruning unsupported dependencies: ["+result.pNodes.length+"]");						
					//		LFUtils.plfToGraph(plf,"./subarchitectures/comsys.mk4/graphs/parser/pruned-"+plf.packedLFId, true);
					log("We keep the unsupported dependency in memory for later repruning");
					unsupportedDependencies.add(support);
				} // end if check whether unsupported
				else { 
					log("Supported relation ["+support.mode+"] under nominal ["+support.headNomVar+"] ");
					log("Pruning PLF -- number of packing nodes before pruning using supported dependencies: ["+result.pNodes.length+"]");
					result = LFUtils.plfRemoveIncompatibleDependencies(result,support.headNomVar, support.depNomVar, support.mode); 
					log("Pruning PLF -- number of packing nodes after pruning using supported dependencies: ["+result.pNodes.length+"]");							
				} 
			} // end for over individual supports
		} // end for
	} else {
		// System.err.println("[WARNING:ActiveIncrParser] There are no interpretation support objects yet for ["+strId+"]");
	} // end if..else check for available supports 
	if (result == null) { 
		result = plf;
	//	System.err.println("[WARNING:ActiveIncrParser] Pruned to null, returning original -");
	}
	return result;
} // end prunePackedLogicalForm




/**
		The method <i>packChartAnalyses</i> takes a chart, reads out the parses, and converts the
		parses (particularly, the semantics in the parses) into logical forms from which we can 
		construct a packed representation. 

		@param chart		The chart with the parses
		@param stringPos	The position up to which there are analyses in the chart
		@return PackedLogicalForm The packed logical form representation

 */ 

public PackedLFParseResults packChartAnalyses(PackedLFParseResults results, opennlp.ccg.parse.Chart chart) throws ParseException {

	int stringPos = results.stringPos;		
	log("Packing parsing results at string position ["+stringPos+"]");

	Hashtable<String,Hashtable<String,Integer>> nonStandardRulesApplied = 
		new Hashtable<String,Hashtable<String,Integer>>();

	PackedLogicalForm plf = null;
	try { 
		log("Retrieving parses from the chart");		 

		List parses = (List<Sign>) parser.createResult(chart,stringPos); 

		//log("Setting up sign array");		 			

		Sign[] signs = new Sign[parses.size()];

		//log("Converting parses to array");

		parses.toArray(signs);		

		log("Going to pack "+parses.size()+" parses / logical forms");

		int nbEmptyLFs = 0 ;
		LogicalForm[] lfs = new LogicalForm[parses.size()];
		for (int i=0; i < parses.size(); i++) { 
			Category cat = signs[i].getCategory();
			if (cat.getLF() != null) {
				LF convertedLF = null;
				Nominal index = cat.getIndexNominal();	
				HyloHelper.setUtteranceIncrement(utteranceIncrement);
				convertedLF = HyloHelper.compactAndConvertNominals(cat.getLF(), index);
			//				log(convertedLF.prettyPrint(""));
				LogicalForm lf = LFUtils.convertFromLF(convertedLF);
				lf.logicalFormId = "lf"+i;

				nonStandardRulesApplied.put(lf.logicalFormId, getNonStandardRules(signs[i]));


				/**	if (signs[i] !=null && signs[i].nonStandardRulesApplied != null) {
					nonStandardRulesApplied.put(lf.logicalFormId, signs[i].nonStandardRulesApplied);
					} */

				// we verify the logical form is not known to be unsupported due to
				// one of its dependencies
				boolean supported = true;
				for (Iterator<InterpretationSupport> it = unsupportedDependencies.iterator() ; it.hasNext() ;) {

					InterpretationSupport support = it.next();
					if (LFUtils.hasDependency(lf, support.headNomVar,support.depNomVar, support.mode)) {
						supported = false;
						log("Unsupported LF is discarded: " + lf.logicalFormId);
					}
				}
				if (supported) {
					lfs[i-nbEmptyLFs] = lf;
				}

				// log("final lf: " + lfs[i-nbEmptyLFs]);
			}
			else {
				nbEmptyLFs++ ;
			}

		} // end for to convert the parse analyses into our logical forms

		lfs = (LogicalForm[]) LFUtils.resizeArray(lfs, lfs.length-nbEmptyLFs);

		// log("LF array size: " + lfs.length);

		//log("Done converting the logical forms");
		//log("Now starting packing logical forms");	
		packingTool.setUtteranceIncrement(utteranceIncrement);

		//	for (int i=0; i < lfs.length; i++) {
		//		log("lfs"+i+": " + LFUtils.lfToString(lfs[i]));
		//	}

		plf = packingTool.packLogicalForms(lfs); 
		//	LFUtils.plfToGraph(result, "testPS"+increment2, true);
		//	log("PFL written to" + "testPS"+increment2);
		//	increment2++;

		// visualize(result);
		log("Done packing");
	} 
	catch (ParseException e) {
		log("Parse exception");
		log(e.getMessage());
		//	e.printStackTrace();
	}
	catch (Exception e) {
		log("An exception occured while packing logical forms");
		log(e.getMessage()); 
		e.printStackTrace();
	} // end try..catch for parse exceptions when reading out the chart

	results.plf = plf;
	results.nonStandardRulesApplied = nonStandardRulesApplied;

	return results;
} // end packChartAnalyses


private Hashtable<String,Integer> getNonStandardRules(Sign sign) {
	Hashtable<String,Integer> nonStandardRules = 
		new Hashtable<String,Integer>();

	if (sign.getWords().size() == 1 && 
			sign.numberOfRulesType0Applied > 0) {
		nonStandardRules.put("recogError-"+sign.getOrthography(), 1);
	}

	DerivationHistory dh = sign.getDerivationHistory();
	Sign[] inputsigns = dh.getInputs();

	if (dh != null && dh.getRule() != null && 
			dh.getRule().name().contains("ROBUST")) {
		String rulename = dh.getRule().name();
		if (nonStandardRules.containsKey(rulename)) {
			Integer oldValue = nonStandardRules.get(rulename);
			nonStandardRules.put(rulename, 
					new Integer(oldValue.intValue() + 1));
		}
		else {
			nonStandardRules.put(rulename, new Integer(1));
		}
	}

	// recursion
	for (int i=0; inputsigns != null && i < inputsigns.length; i++) {
		Hashtable<String,Integer> hash = getNonStandardRules(inputsigns[i]);
		for (Enumeration<String> e = hash.keys(); e.hasMoreElements();) {
			String rulename = e.nextElement();
			Integer newValue = hash.get(rulename);
			if (nonStandardRules.containsKey(rulename)) {
				Integer oldValue = nonStandardRules.get(rulename);
				nonStandardRules.put(rulename, 
						new Integer(oldValue.intValue() + newValue.intValue()));
			}
			else {
				nonStandardRules.put(rulename, 
						new Integer(newValue.intValue()));
			}
		}
	}

	return nonStandardRules;
}

private void wait (int ms) { 
	try {
		Thread.sleep(ms);
	} catch (InterruptedException ie) {}
} // end wait

// =================================================================
// LOG METHODS [  ]
// =================================================================

/** 
		The method <i>setLogLevel</i> defines the logging level for the parser: 
		<ol>
		<li value="0"> no logging </li>
		<li value="1"> logging on system.out</li>
		<li value="2"> logging on system.out, and visualization of packed logical forms </li>		
		</ol>
		To set the log level, it is best to use the public constants <tt>LOG_FALSE</tt>, <tt>LOG_TRUE</tt>, <tt>LOG_VISUALIZE</tt>. 

		@param l The log level 
 */ 

public void setLogLevel (int l) {
	logging = false; visualization = false; 
	switch (l) {
	case 0 : break; 
	case 1 : logging = true; break ; 
	case 2 : logging = true; visualization = true; break;
	} // end switch
	packingTool.logging = true;
} //

/**
		The method <i>log</i> prints out the given message to system.out. 

		@param msg The log message to be printed 
 */

public void log (String msg) { 
	if (logging) { System.out.println("[LOG \"ActiveIncrCCGParser\"] "+msg); }
} // end log

/**
		The method <i>visualize</i> visualizes the given packed logical form. 

		@param msg The log message to be printed 
 */


public void visualize (PackedLogicalForm plf) { 

} // end visualize


public LFPacking getPackingTool () {
	return packingTool;
}

} // end class
