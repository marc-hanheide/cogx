//=================================================================
//Copyright (C) 2007-2010 Geert-Jan M. Kruijff (gj@dfki.de)

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
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
//IMPORTS

// Java
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Stack;
import java.util.Vector;

// OpenCCG
import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.lexicon.DefaultTokenizer;
import opennlp.ccg.lexicon.Tokenizer;
import opennlp.ccg.lexicon.Word;
import opennlp.ccg.parse.ChartScorer;
import opennlp.ccg.parse.IncrCKYParser;
import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;
import opennlp.ccg.unify.UnifyControl;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.PackedLogicalForm;

// Dialogue API util
import de.dfki.lt.tr.dialogue.slice.parse.PhonStringLFPair;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;
import de.dfki.lt.tr.dialogue.util.LFPacking;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.util.ParsingUtils;

/**
 * The class <tt>IncrCCGStringParser</tt> defines an incremental CCG parser, 
 * to parse a single string with a given OpenCCG-style CCG grammar. The parser
 * includes the possibility for performing parse selection. 
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @since	100607 (based on earlier versions, heavily refactored)
 * @version	100607
 */

public class IncrCCGStringParser {

	// Access to the grammar is factorized out
	protected OpenCCGGrammar grammar;

	// The actual parser
	private IncrCKYParser parser; 
	// Parameter vector for parse selection 
//	private ParameterVector params;
	// Beamwidth for parse selection 
//	private int beamwidth;
	// Flag whether to do parse selection / incremental pruning
	private boolean incrementalPruning; 
	// last analysed utterance
	String lastUtterance = "";
	// Increment on the number of utterances
	int utteranceIncrement = 0;
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
	// Mapping strings to packed LFs
	protected Hashtable<String,PackedLFParseResults> phonToCompletedPLFs;
	// removed signs
	protected Hashtable<String,Vector<PackedLFParseResults.SignInChart>> removedSigns;
	// table matching plf id's with phon string id's
	private Hashtable<String,String> plfToPhonStringId;
	// Values for the flag indicating finalized status  
	public static final int NOTFINISHED = 0; 
	public static final int FINAL_PARSE = 1;
	public static final int FINAL_PRUNING = 2;
	// Timing data, Long type
	Hashtable<String,Vector> timingMap;
	Vector timing;
	// The tokenizer.  (Defaults to DefaultTokenizer.) 
	protected Tokenizer tokenizer;
	// The function to score categories / signs before an LFCollection is constructed from parsing results
	ChartScorer chartScorer;
	// The LFPacking factory 
	protected LFPacking packingTool;
	// Decoder used for parse selection
	// public Decoder decoder;

	
	/**
	 * The constructor calls an internal method for initializing the internal variables
	 */
	public IncrCCGStringParser () { 
		init();
	} // end constructor

	/**
	 * Initializes the internal variables
	 */
	private void init () 
	{ 
		grammar = null;
		parser  = null;
//		params  = null;
//		beamwidth = 0;
		incrementalPruning = false;

                chartHistories = new Hashtable<String, Stack>();
                finalPackedLFs = new Hashtable<String, PackedLogicalForm>();
                phonStringPositions = new Hashtable<String, Integer>();
                phonStringTokens = new Hashtable<String, List>();
                phonToCompletedPLFs = new Hashtable<String, PackedLFParseResults>();
                removedSigns = new Hashtable<String, Vector<PackedLFParseResults.SignInChart>>();
                plfToPhonStringId = new Hashtable<String, String>();
                timingMap = new Hashtable<String, Vector>();
                tokenizer = new DefaultTokenizer();
        	packingTool = new LFPacking();
	}	// end init
	
	
	/**
	 * The method <i>registerGrammarAccess</i> registers an object implementing <b>GrammarAccess</b> 
	 * with the parser, so that it is able to obtain grammatical information for words (etc.).  
	 * 
	 * @param	lg	A class implementing the GrammarAccess interface, to provide access to the grammar
	 * @throws	ComsysException Thrown when there is a problem connecting to the grammar
	 */	

	public void registerGrammarAccess(OpenCCGGrammar lg) 
	throws DialogueMissingValueException 
	{
		if (lg != null) { 
			grammar = lg;
			parser = new IncrCKYParser(grammar.getGrammar());
		} else { 
			throw new DialogueMissingValueException("Cannot register grammar access in IncrCCGStringParser: access is null");
		} // end if..else check for non-null grammar access
	} // end registerGrammarAccess
	

	/**
	 * The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	 */ 
	public void configure (Properties props) { 
	/**	if (props.containsKey("parameterVector")) {
			params = (ParameterVector)props.get("parameterVector");
		}
		if (props.containsKey("beamwidth")) {
			beamwidth = ((Integer)props.get("beamwidth")).intValue();
		}
		if (params != null && beamwidth > 0) {
			incrementalPruning = true;
		} */
	} // end configure
	

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
	 * @throws	DialolgueException	Thrown when <tt>start</tt> &lt; 0 or end &gt; the end of the utterance, or when any other parsing error occurred
	 */ 	
	public ParseResults incrParse (PhonString str) 
	throws DialogueException, ParseException { 
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
			chart = new opennlp.ccg.parse.Chart(words.size(), grammar.getGrammar().rules);
			chartHistory = new Stack();
			// initialize the unification control of the parser
			parser.initUnification();
			// initialize the timing vector
			timing = new Vector();
		} // if...else for  retrieving, initializing string position, chart
		// now, loop until a frontier is reached
		try {
			results.parses = new ArrayList<Sign>();
			// checkk whether we are at the end already
			if (stringPos < words.size()-1) { 
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
					// update the chart with the lexical data, re-indexing to provide unique indices 
					for(Iterator signsIter=wordSigns.iterator(); signsIter.hasNext();) {
						Category cat = ((Sign)signsIter.next()).getCategory();
						UnifyControl.reindex(cat);
					} // end for over categories to reindex
					chart.set(stringPos,stringPos,wordSigns);
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
					}
					catch (ParseException e) {
						Vector<PackedLFParseResults.SignInChart> removedSignsForId =
							removedSigns.get(str.id);
						if (incrementalPruning && removedSignsForId.size() > 0) {
							// First parsing attempt failed, Now adding the removed signs back into the chart
							for (PackedLFParseResults.SignInChart sign : removedSignsForId ) {
								chart.insert(sign.x, sign.y, sign.sign);
							}
							removedSignsForId = 
								new Vector<PackedLFParseResults.SignInChart>();
							try {
								chart = parser.stepIncrParser(chart,0,stringPos+1);
								if (chartScorer != null) {
									chart = chartScorer.score(chart,stringPos);
								} 
								results.parses = parser.createResult(chart,stringPos+1);
								if (results.parses.size() > 500) {
									results.parses = new ArrayList<Sign>();
								}
							}
							catch (ParseException f) {
								// Unable to parse at this incremental step
							}
						}
						else {
							// Unable to parse at this incremental step
						}
					}
					SignHashParseResults topCell = 
						new SignHashParseResults(chart.getSigns(0,stringPos));
					// check whether to continue, on the top cell hash at position (0,stringPos) 
					if (stringPos == (words.size()-1)) { 
						continueParsing = false; 
						results.finalized = this.FINAL_PARSE;
						phonToCompletedPLFs.put(str.id, results);
					} else { 
				//		continueParsing = !a_iffilter.eligibleFrontierReached(topCell);
						results.finalized = this.NOTFINISHED;
					} 
					rightNow = Calendar.getInstance();
					long endTime = rightNow.getTime().getTime();
					timing.add(stringPos,new Long(endTime-startTime));
				} // end while 
			} else { 
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
		//		pruneChartAnalyses(results);
			}
			// now do the post pruning on the PLF 
			// PackedLogicalForm prunedPLF = prunePackedLogicalForm(results.plf,str.id);
			// results.plf = prunedPLF;
			if (results.finalized != this.FINAL_PRUNING && 
					results.plf == null) {
				//Current incremental step failed, going directly to the next one
				results = (PackedLFParseResults)incrParse(str);
			}
			else if (results.plf != null) { 
				finalPackedLFs.put(str.id,results.plf);
				plfToPhonStringId.put(results.plf.packedLFId,str.id);
				// Storing link between packed LF id and phonstring id
			} 
			else {
				// phonString not parsable
				phonToCompletedPLFs.put(str.id, new PackedLFParseResults());
			}
		} 
		catch (IndexOutOfBoundsException iobe) { 
			iobe.printStackTrace();
			throw new DialogueException (iobe.getMessage()); 
		} // end try..catch
		return results;
	} // end parse
	
	
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
	/**
	protected void pruneChartAnalyses(PackedLFParseResults results) throws ParseException {
		PackedLFs plf = ParsingUtils.createDummyPLF(results);
		if (plf != null && plf.packedLF != null) { 
			// STEP 1: "lock" the signs contained in the high-score analyses
	    	// to ensure they are not deleted
	//		decoder.params = params;
	//		Vector<String> LFsToKeep = decoder.getBestParses(plf, beamwidth);	
			for (String lfId : LFsToKeep) {
				PackedLFParseResults.SignInChart signInChart = results.lfIdToSignMapping.get(lfId);
				if (signInChart != null) {
					ParsingUtils.lockSignsInDerivationHistory(signInChart.sign, true);
				}
			} // end for
			// STEP 2: get the logical forms NOT contained in the beam width
			// (ie. the parses with a low score) 
			Vector<String> LFsToRemove = new Vector<String>();
			String[] LFs = LFUtils.plfGetPackingNode(plf.packedLF,plf.packedLF.root).lfIds;
			for (int i = 0; i < LFs.length ; i++) {
				String lfId = LFs[i];
				if (!LFsToKeep.contains(lfId)) {
					LFsToRemove.add(lfId);
				}
			} // end for
			for (String lfId : LFsToRemove) {
				LogicalForm lf = packingTool.extractLogicalForm(plf.packedLF, lfId);
			} // end for
			for (PhonString phon :results.phon2LFsMapping.keySet()) {
				String strId = phon.id;
				Stack chartHistory = (Stack) chartHistories.get(strId);
				Chart chart = (opennlp.ccg.parse.Chart)chartHistory.peek();
				Vector<Sign> signsToRemove = new Vector<Sign>();
				Vector<PackedLFParseResults.SignInChart> removedSignsForId =
					removedSigns.get(strId);
				int initSize = removedSignsForId.size();
				
			
			if (results.plf != null && params != null && 
						beamwidth > 0 && 
						results.lfIdToSignMapping.keySet().size() > beamwidth) {
					// STEP 3: collect all signs in the chart which are associated to 
			        //  these low-score LFs, and which are to be removed
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
					} // end for
					// STEP 4: remove the lock on the signs 
					for (String lfId : LFsToKeep) {
						PackedLFParseResults.SignInChart signInChart = 
							results.lfIdToSignMapping.get(lfId);
						if (signInChart != null) {
							ParsingUtils.lockSignsInDerivationHistory(signInChart.sign, false);
						}
					} // end for
					int end = results.stringPos;
					if (end >= chart._size) end = chart._size-1;
					// STEP 5: perform the removal 
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
					} // end for 
				} 
			}
		}
	} // end prune
	*/
	
	
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
		Hashtable<String,Hashtable<String,Integer>> nonStandardRulesApplied = 
			new Hashtable<String,Hashtable<String,Integer>>();
		PackedLogicalForm plf = null;
		try { 
			Sign[] signs = new Sign[results.parses.size()];
			if (results.parses.size()==0)
				return results;
			results.parses.toArray(signs);		
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
		} 
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch for parse exceptions when reading out the chart
		results.plf = plf;
		results.nonStandardRulesApplied = nonStandardRulesApplied;
		return results;
	} // end packChartAnalyses
	
	public static PhonStringLFPair[] convertPhonString2LFPairs (PackedLFParseResults results) {

		if (results.phon2LFsMapping != null) {
			Hashtable<PhonString,Vector<String>> hash = results.phon2LFsMapping;
			Vector<PhonStringLFPair> pairs = new Vector<PhonStringLFPair>();
			for (Enumeration<PhonString> e = hash.keys(); e.hasMoreElements();) {
				PhonString phon = e.nextElement();
				Vector<String> LFs = hash.get(phon);
				for (Enumeration<String> f = LFs.elements(); f.hasMoreElements();) {
					PhonStringLFPair pair = new PhonStringLFPair();
					pair.phonStr = phon;
					pair.logicalFormId = f.nextElement();
					pairs.add(pair);
				}
			}
	
			PhonStringLFPair[] pairsArray = new PhonStringLFPair[pairs.size()];
			pairsArray = pairs.toArray(pairsArray);
			return pairsArray;
		}
		else {
//			Vector<PhonStringLFPair> pairs = new Vector<PhonStringLFPair>();
			return new PhonStringLFPair[0];
		}
	} // end convertPhonString2LFPairs
	
	

	public void registerChartScorer (ChartScorer shs) { 
		chartScorer = shs; 
	} // end registerChartScorer

} // end class
