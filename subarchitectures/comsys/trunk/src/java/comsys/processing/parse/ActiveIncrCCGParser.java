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
import interconnectivity.processing.AttentionMechanism;
import interconnectivity.processing.AttentionMechanismPipeline;
import interconnectivity.processing.ContextActiveProcess;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Stack;
import java.util.Vector;

import comsys.arch.ComsysException;
import comsys.components.parse.cc_Parser;
import comsys.datastructs.comsysEssentials.ContextInfo;
import comsys.datastructs.comsysEssentials.InterpretationSupport;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.lf.LogicalForm;
import comsys.datastructs.lf.PackedLogicalForm;
import comsys.lf.utils.ArrayIterator;
import comsys.lf.utils.LFPacking;
import comsys.lf.utils.LFUtils;
import comsys.processing.parseselection.Decoder;
import comsys.processing.parseselection.ParameterVector;
import comsys.utils.ParsingUtils;

import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.lexicon.DefaultTokenizer;
import opennlp.ccg.lexicon.Tokenizer;
import opennlp.ccg.lexicon.Word;
import opennlp.ccg.parse.Chart;
import opennlp.ccg.parse.ChartScorer;
import opennlp.ccg.parse.Edge;
import opennlp.ccg.parse.EdgeHash;
import opennlp.ccg.parse.IncrCKYParser;
import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;
import opennlp.ccg.unify.UnifyControl;

import cast.core.CASTUtils;
import cast.core.CASTData;

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
	protected boolean logging = false;

	// last analysed utterance
	PhonString lastUtterance = new PhonString();

	// Increment on the number of utterances
	int utteranceIncrement = 0;

	// The actual parser
	private opennlp.ccg.parse.IncrCKYParser parser; 

	// Charthistory is a collection of stacks with charts, the current 
	// chart is on top. A stack is keyed by the ID of the PhonString for 
	// which it maintains the (partial) analyses  
	protected Hashtable<String,Stack> chartHistories;

	// Hashtable with for each phon string (keyed by id) the final packed logical form
	protected Hashtable<String,PackedLogicalForm> finalPackedLFs; 

	// table with the string position for each phonString being parsed 
	protected Hashtable<String,Integer> phonStringPositions;

	// table with the tokenized phonStrings being parsed
	protected Hashtable<String,List> phonStringTokens;

	protected Hashtable<String,PackedLFParseResults> phonToCompletedPLFs;

	protected Hashtable<String,Vector<PackedLFParseResults.SignInChart>> removedSigns;

	// table matching plf id's with phon string id's
	private Hashtable<String,String> plfToPhonStringId; 

	// incremental chart pruning
	public boolean incrementalPruning = false;
	public ParameterVector params;
	public int beamwidth;

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

	
	public Decoder decoder;
	
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
		removedSigns = new Hashtable<String,Vector<PackedLFParseResults.SignInChart>>();
		phonToCompletedPLFs = new Hashtable<String,PackedLFParseResults>();
	 decoder = new Decoder(params);
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
		if (props.containsKey("parameterVector")) {
			params = (ParameterVector)props.get("parameterVector");
		}
		if (props.containsKey("beamwidth")) {
			beamwidth = ((Integer)props.get("beamwidth")).intValue();
		}
		if (params != null && beamwidth > 0) {
			incrementalPruning = true;
		}
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
	public int getChartSize (opennlp.ccg.parse.Chart table) {
		return parser.getChartSize(table); 
	}


	/** The method <i>getFinalPackedLF</i> returns the final packed logical 
	 * form for the given phon string id, or <tt>null</tt> if the id is unknown. 

		@param id The identifier of the phon string
		@return PackedLogicalForm The packed logical form for the final 
		(complete) analyses of the phon string
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
	 * The method <i>parse</i> parses the given PhonString, and 
	 * returns the resulting analyses. 
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
			opennlp.ccg.parse.Chart table = 
				new opennlp.ccg.parse.Chart(entries.size(), 
						((OpenCCGGrammar)grammar.getGrammar()).ccggrammar.rules);

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
			//		results = packChartAnalyses(results, table);
			results.finalized = this.FINAL_PARSE;

		} catch (ParseException e) {
			log(e.getMessage());
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		results = ParsingUtils.createPhon2LFsMapping (results, str);
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
		if (!str.equals(lastUtterance)) {
			lastUtterance = str;
			utteranceIncrement++;
		}

		boolean continueParsing;
		Calendar rightNow;
		opennlp.ccg.parse.Chart chart = null;
		Stack chartHistory = null;
		List<Word> words = new ArrayList<Word>();
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

			if (phonToCompletedPLFs.containsKey(str.id)) {
				log("parse results are already available, simply retrieving it");
				results = phonToCompletedPLFs.get(str.id);
				results.finalized = FINAL_PRUNING;
				return results;
			}

			// get the list of words
			words = (List)phonStringTokens.get(str.id);
			// get the timing vector
			timing = (Vector) timingMap.get(str.id);
		} else { 
			// tokenize the phonString, store the resulting tokenization for later reference
			List<Word> words_temp = tokenizer.tokenize(str.wordSequence);

			for (Iterator<Word> it = words_temp.iterator(); it.hasNext(); ) {
				Word w = it.next();
				if (((OpenCCGGrammarData)grammar.getLexicalEntries(w.getForm())).signHash.size()>0) {
					words.add(w);
				}
			}
			phonStringTokens.put(str.id,words);

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

			results.parses = new ArrayList<Sign>();

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
						if (chartScorer != null) {
							chart = chartScorer.score(chart,stringPos);
						} 
						results.parses = parser.createResult(chart,stringPos+1);
						if (results.parses.size() > 500) {
							results.parses = new ArrayList<Sign>();
						}
						// log("Chart after parsing step:\n"+chart.toString());	
					}
					catch (ParseException e) {

						Vector<PackedLFParseResults.SignInChart> removedSignsForId =
							removedSigns.get(str.id);

						if (incrementalPruning && removedSignsForId.size() > 0) {

							log("[WARNING] First parsing attempt failed...");
							log("Now adding the removed signs to the chart (" 
									+ removedSignsForId.size() + " signs to reinsert)");

							for (PackedLFParseResults.SignInChart sign : removedSignsForId ) {
								chart.insert(sign.x, sign.y, sign.sign);
							}
							removedSignsForId = 
								new Vector<PackedLFParseResults.SignInChart>();
							log("Retry parsing...");

							try {
								chart = parser.stepIncrParser(chart,0,stringPos+1);
								if (chartScorer != null) {
									chart = chartScorer.score(chart,stringPos);
								} 
								results.parses = parser.createResult(chart,stringPos+1);
								if (results.parses.size() > 500) {
									results.parses = new ArrayList<Sign>();
								}
								log("Second parsing attempt successful");
							}
							catch (ParseException f) {
								log("[WARNING] Unable to parse at this incremental step");
							}
						}
						else {
							log("[WARNING] Unable to parse at this incremental step");
						}
					}

					SignHashParseResults topCell = 
						new SignHashParseResults(chart.getSigns(0,stringPos));

					// check whether to continue, on the top cell hash at position (0,stringPos) 
					if (stringPos == (words.size()-1)) { 
						log("Stop parsing, end of the utterance");
						continueParsing = false; 
						results.finalized = this.FINAL_PARSE;
						phonToCompletedPLFs.put(str.id, results);

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
				log("At the end of the utterance, need to wait " +
				"on interpretation support to perform final pruning steps");
				results.finalized = this.FINAL_PRUNING;
				// do the final pruning
			} // end if..else check for whether we've reached the end already

			// return the packed results
			results.setStringPosition(stringPos);

			results = packChartAnalyses(results);

			// create a mapping between the phonological string and the LFs
			results = ParsingUtils.createPhon2LFsMapping (results, str);

			// store the chart on the chart history, store the history and the string position for this phonString
			chartHistory.push(chart);
			chartHistories.put(str.id,chartHistory);
			phonStringPositions.put(str.id,new Integer(stringPos));
			timingMap.put(str.id,timing);

			// incremental chart scoring using parse selection
			if (!removedSigns.containsKey(str.id)) {
				Vector<PackedLFParseResults.SignInChart> removedSignsForId = 
					new Vector<PackedLFParseResults.SignInChart>();
				removedSigns.put(str.id, removedSignsForId);
			}
			if (incrementalPruning) {
				pruneChartAnalyses(results);
			}

			// now do the post pruning on the PLF 
			// PackedLogicalForm prunedPLF = prunePackedLogicalForm(results.plf,str.id);
			// results.plf = prunedPLF;

			if (results.finalized != this.FINAL_PRUNING && 
					results.plf == null) {
				log("Current incremental step failed, going directly to the next one");
				results = (PackedLFParseResults)incrParse(str);
			}

			else if (results.plf != null) { 
				finalPackedLFs.put(str.id,results.plf);
				plfToPhonStringId.put(results.plf.packedLFId,str.id);
				log("Storing link between packed LF id ["+
						results.plf.packedLFId+"] and phonstring id ["+str.id+"]");
			} 

			else {
				System.err.println("[WARNING:ActiveIncrParser] phonString" +
						" not parsable: \"" + str.wordSequence + "\"");
				phonToCompletedPLFs.put(str.id, new PackedLFParseResults());
			}
		} 
		catch (IndexOutOfBoundsException iobe) { 
			iobe.printStackTrace();
			throw new ComsysException (iobe.getMessage()); 
		} // end try..catch


		return results;
	} // end parse


	/** 
		The method <i>prunePackedLogicalForm</i> checks whether relations 
		in the packed logical form are supported or not, and if not, 
		then they are deleted. Relations cover both LFRelation and 
		PackingEdge objects. 
	 */ 
	public PackedLogicalForm prunePackedLogicalForm (PackedLogicalForm plf, 
			String strId) { 
		// Initialize the result
		PackedLogicalForm result = plf;

		if (interpretationSupport.containsKey(strId)) { 
			// Get the supported interpretations
			Vector interpretations = interpretationSupport.get(strId);
			for (Iterator suppintIter = interpretations.iterator(); 
			suppintIter.hasNext(); ) { 
				InterpretationSupport[] supports = 
					(InterpretationSupport[]) suppintIter.next();
				for (ArrayIterator supportsIter = 
					new ArrayIterator(supports); supportsIter.hasNext(); ) {
					InterpretationSupport support = 
						(InterpretationSupport) supportsIter.next();
					if (!support.isSupported) {
						log("Unsupported relation ["+support.mode+"] " +
								"under nominal ["+support.headNomVar+"] ");
						log("Pruning PLF -- number of packing nodes " +
								"before pruning unsupported dependencies: ["
								+result.pNodes.length+"]");
						result = LFUtils.plfRemoveDependence(result,
								support.headNomVar, support.depNomVar, support.mode); 
						log("Pruning PLF -- number of packing nodes after pruning " +
								"unsupported dependencies: ["+result.pNodes.length+"]");						
						log("We keep the unsupported dependency in memory " +
						"for later repruning");
						unsupportedDependencies.add(support);
					} // end if check whether unsupported
					else { 
						log("Supported relation ["+support.mode+"] under " +
								"nominal ["+support.headNomVar+"] ");
						log("Pruning PLF -- number of packing nodes before " +
								"pruning using supported dependencies: ["+result.pNodes.length+"]");
						result = LFUtils.plfRemoveIncompatibleDependencies
						(result,support.headNomVar, support.depNomVar, support.mode); 
						log("Pruning PLF -- number of packing nodes after pruning " +
								"using supported dependencies: ["+result.pNodes.length+"]");							
					} 
				} // end for over individual supports
			} // end for
		} else {
		} // end if..else check for available supports 
		if (result == null) { 
			result = plf;
		}
		return result;
	} // end prunePackedLogicalForm



	/**
	 * Tool for incremental chart pruning: for each logical form contained in the
	 * PLF, we compute their parse selection score, and we retain only the logical 
	 * forms with a high score (where the number of LFs to be kept is determined 
	 * by the <i>beamwidth</i> parameter).  The lower-score logical forms are 
	 * removed, and all the intermediate signs which are associated to it in the 
	 * chart are removed  as well.  
	 * 
	 * @param results the packed logical form results
	 * @param chart the CKY chart
	 * @return the pruned chart
	 * @throws ParseException
	 */
	protected void pruneChartAnalyses(PackedLFParseResults results) throws ParseException {

		log("Now pruning the chart analyses...");
		
		PackedLFs plf = ParsingUtils.createDummyPLF(results);

		if (plf != null && plf.packedLF != null) {
			log("Beam width applied: " + beamwidth);
 
			/** STEP 1: "lock" the signs contained in the high-score analyses
	    to ensure they are not deleted */
			decoder.params = params;
			Vector<String> LFsToKeep = decoder.getBestParses(plf, beamwidth);	

			for (String lfId : LFsToKeep) {
				PackedLFParseResults.SignInChart signInChart = results.lfIdToSignMapping.get(lfId);
				if (signInChart != null) {
					ParsingUtils.lockSignsInDerivationHistory(signInChart.sign, true);
				}
			}

			/** STEP 2: get the logical forms NOT contained in the beam width
        (ie. the parses with a low score) */
			Vector<String> LFsToRemove = new Vector<String>();
			String[] LFs = LFUtils.plfGetPackingNode(plf.packedLF,plf.packedLF.root).lfIds;
			for (int i = 0; i < LFs.length ; i++) {
				String lfId = LFs[i];
				if (!LFsToKeep.contains(lfId)) {
					LFsToRemove.add(lfId);
				}
			}

			for (String lfId : LFsToRemove) {
				LogicalForm lf = packingTool.extractLogicalForm(plf.packedLF, lfId);
			}

			for (PhonString phon :results.phon2LFsMapping.keySet()) {
				String strId = phon.id;

				log("currently under view: " + phon.wordSequence);

				Stack chartHistory = (Stack) chartHistories.get(strId);
				Chart chart = (opennlp.ccg.parse.Chart)chartHistory.peek();

				Vector<Sign> signsToRemove = new Vector<Sign>();

				Vector<PackedLFParseResults.SignInChart> removedSignsForId =
					removedSigns.get(strId);
				int initSize = removedSignsForId.size();

				if (results.plf != null && params != null && 
						beamwidth > 0 && 
						results.lfIdToSignMapping.keySet().size() > beamwidth) {

					/** STEP 3: collect all signs in the chart which are associated to 
			            these low-score LFs, and which are to be removed */
					for (String lfId : LFsToRemove) {
						PackedLFParseResults.SignInChart signInChart = 
							results.lfIdToSignMapping.get(lfId);
						if (signInChart != null) {
							signsToRemove.add(signInChart.sign);
							List<Sign> signsToCheck = 
								ParsingUtils.collectSignInDerivationHistory(signInChart.sign);
							for (Iterator<Sign> it =  signsToCheck.iterator() ; it.hasNext(); ) {
								Sign sign = it.next();
								if (!sign.locked) {
									signsToRemove.add(sign);
								}
							} 
						}
					}

					/** STEP 4: remove the lock on the signs */
					for (String lfId : LFsToKeep) {
						PackedLFParseResults.SignInChart signInChart = 
							results.lfIdToSignMapping.get(lfId);
						if (signInChart != null) {
							ParsingUtils.lockSignsInDerivationHistory(signInChart.sign, false);
						}
					}

					int end = results.stringPos;
					if (end >= chart._size) end = chart._size-1;

					/** STEP 5: perform the removal */
					for (int i = 0 ; i <= end; i++) {	
						for (int j = i ; j <= end; j++) {	
							EdgeHash oldSH = chart.unpack(i, j);
							SignHash newSH = new SignHash();
							for (Iterator<Edge> edges = oldSH.iterator(); edges.hasNext();) {
								Edge edge = edges.next();

								if (!signsToRemove.contains(edge.sign)) {
									newSH.add(edge.sign);		
								}
								else {
									PackedLFParseResults.SignInChart signInChart =
										results.new SignInChart();
									signInChart.sign = edge.sign;
									signInChart.x = i;
									signInChart.y = j;
									removedSignsForId.add(signInChart);
								}
							}
							chart.set(i, j, newSH);
						}
					}
					log("Chart successfully pruned (" + (removedSignsForId.size() - initSize) 
							+ " signs removed from chart)");
				}
			}
		}
	}



	/**
		The method <i>packChartAnalyses</i> takes a chart, reads out 
		the parses, and converts the parses (particularly, the semantics
		 in the parses) into logical forms from which we can 
		construct a packed representation. 

		@param chart		The chart with the parses
		@param stringPos	The position up to which there are analyses in the chart
		@return PackedLogicalForm The packed logical form representation

	 */ 

	public PackedLFParseResults packChartAnalyses(PackedLFParseResults results) 
	throws ParseException {

		int stringPos = results.stringPos;		
		log("Packing parsing results at string position ["+stringPos+"]");

		Hashtable<String,Hashtable<String,Integer>> nonStandardRulesApplied = 
			new Hashtable<String,Hashtable<String,Integer>>();

		PackedLogicalForm plf = null;
		try { 
			log("Retrieving parses from the chart");		 

			Sign[] signs = new Sign[results.parses.size()];

			if (results.parses.size()==0)
				return results;

			results.parses.toArray(signs);		

			log("Going to pack "+results.parses.size()+" parses / logical forms");

			int nbEmptyLFs = 0 ;
			LogicalForm[] lfs = new LogicalForm[results.parses.size()];
			for (int i=0; i < results.parses.size(); i++) { 
				Category cat = signs[i].getCategory();
				if (cat.getLF() != null) {
					LF convertedLF = null;
					Nominal index = cat.getIndexNominal();	
					HyloHelper.setUtteranceIncrement(utteranceIncrement);
					convertedLF = HyloHelper.compactAndConvertNominals(cat.getLF(), index);
					//				log(convertedLF.prettyPrint(""));
					LogicalForm lf = LFUtils.convertFromLF(convertedLF);
					lf.logicalFormId = "lf"+i;

					// maintain a reference to the sign in the chart
					// (necessary to perform later chart pruning)
					PackedLFParseResults.SignInChart signInChart = results.new SignInChart();
					signInChart.x = 0;
					signInChart.y = stringPos;
					signInChart.sign = signs[i];
					results.lfIdToSignMapping.put(lf.logicalFormId, signInChart);

					nonStandardRulesApplied.put(lf.logicalFormId, 
							ParsingUtils.getNonStandardRules(signs[i]));

					// we verify the logical form is not known to be unsupported due to
					// one of its dependencies
					boolean supported = true;
					/** for (Iterator<InterpretationSupport> it = 
						unsupportedDependencies.iterator() ; it.hasNext() ;) {

						InterpretationSupport support = it.next();
						if (LFUtils.hasDependency(lf, support.headNomVar,
								support.depNomVar, support.mode)) {
							supported = false;
							log("Unsupported LF is discarded: " + lf.logicalFormId);
						}
					} */
					if (supported) {
						lfs[i-nbEmptyLFs] = lf;
					} 
				}
				else {
					nbEmptyLFs++ ;
				}
			} // end for to convert the parse analyses into our logical forms

			lfs = (LogicalForm[]) LFUtils.resizeArray(lfs, lfs.length-nbEmptyLFs);

			packingTool.setUtteranceIncrement(utteranceIncrement);

			plf = packingTool.packLogicalForms(lfs); 

			log("Done packing");
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


	//	=================================================================
	//	LOG METHODS [  ]
	//	=================================================================

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
		logging = false; 
		switch (l) {
		case 0 : break; 
		case 1 : logging = true; break ; 
		case 2 : logging = true; break;
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

	static void wait (int ms) { 
		try {
			Thread.sleep(ms);
		} catch (InterruptedException ie) {}
	} // end wait


	public LFPacking getPackingTool () {
		return packingTool;
	}

} // end class
