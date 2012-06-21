// =================================================================
// Copyright (C) 2007-2010 Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.cast.dialogue;

// =================================================================
// IMPORTS

// CAST 
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.SubarchitectureComponentException;
import cast.AlreadyExistsOnWMException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.core.CASTUtils;

// CAST-extension of Dialogue API 
import de.dfki.lt.tr.cast.ProcessingData;

// Dialogue API slice

// Dialogue API 
import de.dfki.lt.tr.dialogue.parse.IncrCCGStringParser;
import de.dfki.lt.tr.dialogue.parse.PackedLFParseResults;
import de.dfki.lt.tr.dialogue.parse.OpenCCGGrammar;
import de.dfki.lt.tr.dialogue.parse.preprocess.BasicPhonStringPreprocessor;
import de.dfki.lt.tr.dialogue.parse.preprocess.CapitalizationPhonStringPreprocessor;
import de.dfki.lt.tr.dialogue.parse.preprocess.InterpunctionPhonStringPreprocessor;
import de.dfki.lt.tr.dialogue.parse.preprocess.ParaphrasingPhonStringPreprocessor;
import de.dfki.lt.tr.dialogue.parse.preprocess.PhonStringPreprocessor;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.lf.PackedLogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.NonStandardRulesAppliedForLF;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.parse.PhonStringLFPair;
import de.dfki.lt.tr.dialogue.util.DialogueException;

// Java
import de.dfki.lt.tr.dialogue.util.ParsingUtils;
import java.io.IOException;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

// OpenCCG
// import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;
import opennlp.ccg.parse.ParseException;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

	/**
	 * The class <tt>IncrStringParser</tt> provides a CAST wrapper around 
	 * an incremental CCG parser, processing individual strings that appear
	 * as PhonString objects on working memory. 
	 * 
	 * <h4>CAST-file component configuration options </h4>
	 * <ul>
	 * <li> <tt>--grammar FILENAME</tt>: specifies the location of the CCG grammar (grammar.xml file) 
	 *      to be used (REQUIRED)
	 *    
	 * <li> <tt>--asr_subarch SAID</tt>: specifies the ID of the ASR subarchitecture. If specified, only 
	 * 		data (</tt>PhonString</tt>) from this subarchitecture is listened for (OPTIONAL)
	 * 
	 * <li> <tt>--generateLFGraphs [BOOL]</tt>: specifies whether to generate DOT graphs for each logical form. 
	 * 	    Specifying the BOOL is optional; default (when option is used) is true. 
	 * 		(OPTIONAL; recommended to use the <tt>--graphsDir</tt> as well)
	 * 
	 * <li> <tt>--generatePackedGraph [BOOL]</tt>: specifies whether to generate DOT graphs for each 
	 *  	packed logical form. Specifying the BOOL is optional; default (when option is used) is true.
	 * 		(OPTIONAL; recommended to use the <tt>--graphsDir</tt> as well)
	 * 
	 * <li> <tt>--graphsDir DIRNAME</tt>: specifies the directory DIRNAME where DOT graphs are to be written to. 
	 * 		(OPTIONAL; recommended to be used in conjunction with any <tt>--generateXYZ</tt> option)
	 * </ul> 
	 *
	 * 
	 * 
	 * @version 100608
	 * @started 070813
	 * @author  Geert-Jan M. Kruijff (gj@dfki.de)
	 * @author  Pierre Lison (plison@dfki.de)
	*/ 


public class IncrStringParser
extends AbstractDialogueComponentUsingTaskManager {

	// Processing Data id counter
	private int pdIdCounter;

	// Filename of the utterance grammar to be used 
	public String grammarFile; 

	// The grammatical inference engine
	public IncrCCGStringParser parser;
	
	// The grammar access
	OpenCCGGrammar grammar; 

	// Map with (unique) identifiers of PackedLogicalForm objects to PackedLFs in which they are stored
	Hashtable<String,String> plfToPackedLFsId;


	// Whether to generate DOT graphs
	boolean generateLFGraphs = false ;
	boolean generatePackedGraph = false ;
	
	// the subdirectory in which to store the DOT and PNG files
	String graphsDir = "graphs";

	private PackedLFs lastAddedPLf = null;
	
	private final List<PhonStringPreprocessor> preprocessors = new LinkedList<PhonStringPreprocessor>();

	@Override
	public void configure(Map<String, String> _config) {

		if (_config.containsKey("--grammar")) {
			grammarFile = _config.get("--grammar");
		}
		else {
			grammarFile = "./grammars/openccg/moloko.v4/grammar.xml";
		}

		// generate a PNG file (using DOT) for each logical form
		if (_config.containsKey("--generateLFGraphs")) {
			log("generate DOT graphs representing the logical forms");
			String value = _config.get("--generateLFGraphs");
			if (value.startsWith("true") || value.equals("")) {
				generateLFGraphs = true;
			}
		}

		// generate a PNG file (using DOT) for the packed logical form
		if (_config.containsKey("--generatePackedGraph")) {
			log("generate a DOT graph representing the packed logical form");
			String value = _config.get("--generatePackedGraph");
			if (value.startsWith("true") || value.equals("")) {
				generatePackedGraph = true;
			}
		}

		if (_config.containsKey("--graphsDir")) {
			graphsDir = _config.get("--graphsDir");
		}

	}

	@Override
	public void start() {
		super.start();

		// Component
		log("Initializing utterance interpretation component");
		// Make all the comsys queue changes to avoid missing updates
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

		// Data processing
		pdIdCounter = 0;
		plfToPackedLFsId = new Hashtable<String, String>();

		try {
			// Parsing processes: set up the grammar
			grammar = new OpenCCGGrammar();
			grammar.setGrammar(grammarFile);
			// Parsing processes: set up the actual parser, connect it to the grammar
			parser = new IncrCCGStringParser();
			// Register the grammar: this initializes the connection with the lexicon, 
			// and creates the ccg parser with the corresponding openccg grammar
			parser.registerGrammarAccess(grammar);

			CategoryChartScorer cshs = new CategoryChartScorer();
			cshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			parser.registerChartScorer(cshs);

		}
		catch (IOException ex) {
			getLogger().fatal("fatal exception in start: " + ex.getMessage(), ex);
			throw new IllegalStateException(ex);
		}

		preprocessors.add(new ParaphrasingPhonStringPreprocessor());
		preprocessors.add(new CapitalizationPhonStringPreprocessor());
		preprocessors.add(new BasicPhonStringPreprocessor());
		preprocessors.add(new InterpunctionPhonStringPreprocessor());
        	
		// register change filters for PhonString, which triggers parsing
		addChangeFilter(
			ChangeFilterFactory.createGlobalTypeFilter(PhonString.class, WorkingMemoryOperation.ADD),
			new WorkingMemoryChangeReceiver() {
				@Override
				public void workingMemoryChanged(WorkingMemoryChange wmc) {
					handleNewPhonString(wmc);
				}
			});


		WorkingMemoryChangeReceiver packedLFsReceiver = new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange wmc) {
				handlePackedLFs(wmc);
			}
		};

		addChangeFilter(
			ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class, WorkingMemoryOperation.ADD),
			packedLFsReceiver);

		addChangeFilter(
			ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class, WorkingMemoryOperation.OVERWRITE),
			packedLFsReceiver);

	}

	private void handleNewPhonString(WorkingMemoryChange wmc) {
		try {
			CASTData<PhonString> data = getMemoryEntryWithData(wmc.address, PhonString.class);
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			addProposedTask(taskID, pd);
			String taskGoal = DialogueGoals.INCREMENTAL_PARSING_STEP_TASK;
			proposeInformationProcessingTask(taskID, taskGoal);
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("exception in handling PhonString: " + ex.message, ex);
		}
	}

	private void handlePackedLFs(WorkingMemoryChange wmc) {
		try {
			CASTData<PackedLFs> data = getMemoryEntryWithData(wmc.address, PackedLFs.class);
			PackedLFs arg = data.getData();

			if (!ParsingUtils.arePLFsFinalized(arg)) {
				String taskID = newTaskID();
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(data);
				addProposedTask(taskID, pd);
				String taskGoal = DialogueGoals.INCREMENTAL_PARSING_STEP_TASK;
				proposeInformationProcessingTask(taskID, taskGoal);
			}
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("exception in handling PackedLFs: " + ex.message, ex);
		}
	}

	
	/** 
		Stores the given packed logical form. The method retrieves the 
		PackedLFs object from working memory, updates the plf object in 
		it, and then stores the PackedLFs object again, with updated
		finalized flag.
	*/ 
	public void storePackedLF(PackedLogicalForm plf, int finalizedFlag) {
		log("Storing pruned packed logical form on working memory");
		if (plfToPackedLFsId.containsKey(plf.packedLFId)) {
			// get the packedlf id = wmc address 
			String packedLFId = plfToPackedLFsId.get(plf.packedLFId);
			try {
				// try to get the packed lf from WM
				CASTData data = getWorkingMemoryEntry(packedLFId);
				String dataType = data.getType();
				if (dataType.equals(CASTUtils.typeName(PackedLFs.class))) {
					// get the object
					PackedLFs packedLFs = (PackedLFs) data.getData();
					// set the plf
					packedLFs.packedLF = plf;
					// set the finalized flag
					packedLFs.finalized = finalizedFlag;
					// store the object
					overwriteWorkingMemory(data.getID(), packedLFs);

					log("Successfully stored packed logical form on working memory");
				}
				else {
					System.err.println("[ERROR:UtteranceInterpretation] Retrieved data does not contain PackedLFs [" + packedLFId + "]");
				}
			}
			catch (SubarchitectureComponentException e) {
				System.err.println("[ERROR:UtteranceInterpretation] Could not retrieve PackedLFs [" + packedLFId + "]");
				e.printStackTrace();
			}
		}
		else {
			System.err.println("[WARNING:UtteranceInterpretation] No PackedLFs ID found for PLF [" + plf.packedLFId + "]");
		}
	}

	@Override
	public void executeTask(ProcessingData pd) throws DialogueException {
		try {
			List<String> pdTypes = pd.getTypes();

			if (pdTypes.contains(CASTUtils.typeName(PhonString.class))) {
				CASTData data = pd.getByType(CASTUtils.typeName(PhonString.class));
				String dataType = data.getType();
				log("Execute parse task on data item [" + dataType + "]");
				if (data != null) {
					log("Now try to start parsing with the phon string");
					// get the input string
					PhonString phonString = (PhonString) data.getData();

					getLogger().debug("before preprocessing: \"" + phonString.wordSequence + "\"");
					for (PhonStringPreprocessor pp : preprocessors) {
						pp.apply(phonString);
						getLogger().debug("applied " + pp.getClass().getCanonicalName() + " => " + phonString.wordSequence);
					}
					getLogger().debug("preprocessing done, final result: \"" + phonString.wordSequence + "\"");

					// let the parser start the incremental analysis
					if (phonString.wordSequence.length() != 0) {
						PackedLFParseResults results = (PackedLFParseResults) parser.incrParse(phonString);

						PhonStringLFPair[] pairs = IncrCCGStringParser.convertPhonString2LFPairs(results);

						String id = newDataID();

						PackedLFs plf = new PackedLFs(
								id,
								pairs, 
								new PhonString[0],
								results.stringPos, 
								results.plf,
								results.finalized, 
								"interpretation", 
								new NonStandardRulesAppliedForLF[0],
								phonString.confidenceValue,
								phonString.maybeOOV,
								phonString.ival,
								phonString.wordSequence
								);

						log("Add the newly formed PackedLF to working memory, finalized is ["+results.finalized+"]");

						// Update the maps 
						String plfId = "";
						if (results != null) { 
							if (results.plf != null) { 
								plfId = results.plf.packedLFId;
							}
							else { 
								throw new DialogueException("Cannot parse this thing");
							} 
						}
						else { 
							throw new DialogueException("Cannot parse this thing");
						} 	

						try {
							addToWorkingMemory(id, plf);
							lastAddedPLf = plf;
							plfToPackedLFsId.put(plfId,id);
						}
						catch (AlreadyExistsOnWMException ex) {
							getLogger().error(ex.message, ex);
						}
						catch (NullPointerException ex) {
							getLogger().fatal("Got a null pointer exception (propagating it further)", ex);
							throw ex;
						}
					}
				}
				else {
					throw new DialogueException("Error: parsing task on empty data for type [" + dataType + "]");
				}
			} 

			else if (pdTypes.contains(CASTUtils.typeName(PackedLFs.class))) {
				CASTData data = pd.getByType(CASTUtils.typeName(PackedLFs.class));
				String dataType = data.getType();
				log("Execute parse task on data item [" + dataType + "]");
				if (data != null) {
					// get the packed logical forms
					PackedLFs plfs = (PackedLFs) data.getData();
					// get the input string
					PhonString phonString = plfs.phonStringLFPairs[0].phonStr;

					// trigger the parser to continue the incremental analysis
					// -- question is of course whether we SHOULD continue at all! 
					// at the same time, we may want to do one final round of pruning
					// after parsing has finished, to prune the final set of analyses. 
					log("Check whether the parsing has finished: [" + plfs.finalized + "]");

					if (plfs.finalized < IncrCCGStringParser.FINAL_PARSE) {
						log("Now try to continue parsing with the phon string");
						PackedLFParseResults results = (PackedLFParseResults) parser.incrParse(phonString);

						// now write the results to the WM, as a PackedLFs 
						if (results.plf != null) {

							PhonStringLFPair[] pairs = IncrCCGStringParser.convertPhonString2LFPairs(results);

							PackedLFs plf = new PackedLFs(
								data.getID(),
								pairs,
								new PhonString[0],
								results.stringPos,
								results.plf,
								results.finalized,
								"interpretation",
								new NonStandardRulesAppliedForLF[0],
								plfs.phonStringConfidence,
								plfs.phonStringMaybeOOV,
								plfs.phonStringIval,
								plfs.phonStringWordList);

							log("Updating PackedLF in working memory, finalized flag is [" + results.finalized + "]");
							try {
								overwriteWorkingMemory(data.getID(), plf);
								lastAddedPLf = plf;
								plfToPackedLFsId.put(results.plf.packedLFId, data.getID());
							}
							catch (DoesNotExistOnWMException ex) {
								getLogger().error(ex.message, ex);
							}
							catch (ConsistencyException ex) {
								getLogger().error(ex.message, ex);
							}
							catch (PermissionException ex) {
								getLogger().error(ex.message, ex);
							}
							catch (NullPointerException ex) {
								getLogger().fatal("Got a null pointer exception (propagating it further)", ex);
								throw ex;
							}
						}
						else {
							log("Packed logical form is empty --> not inserting into working memory");
							throw new ParseException("Packed logical form is empty --> not inserting into working memory");
						}
					}
					else {
						log("PackedLFs provide a complete, and finally pruned, interpretation -- no need to parse");
					}
				}
				else {
					throw new DialogueException("Error: parsing task on empty data for type [" + dataType + "]");
				}
			}
			else {
				throw new DialogueException("Error: parsing task on illegal data type [" + pd.getTypes() + "]");
			}
		}
		catch (ParseException ex) {
			throw new DialogueException(ex);
		}
	}

}
