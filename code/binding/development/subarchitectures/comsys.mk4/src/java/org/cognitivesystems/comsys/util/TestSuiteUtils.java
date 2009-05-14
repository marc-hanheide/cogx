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

package org.cognitivesystems.comsys.util;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.data.PackedLFParseResults;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.processing.ActiveCCGLexicon;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;

// -----------------------------------------------------------------
// INTERCONNECTIVITY IMPORTS
// -----------------------------------------------------------------
import org.cognitivesystems.interconnectivity.processing.AttentionMechanism;
import org.cognitivesystems.interconnectivity.processing.AttentionMechanismPipeline;
import org.cognitivesystems.interconnectivity.processing.ContextActiveProcess;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.MalformedURLException; 
import java.net.URL;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Stack;
import java.util.Vector; 

// -----------------------------------------------------------------
// OPENCCG IMPORTS
// -----------------------------------------------------------------
import opennlp.ccg.lexicon.DefaultTokenizer;
import opennlp.ccg.lexicon.Tokenizer;
import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.Chart;
import opennlp.ccg.parse.EmptyChartScorer;
import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;
import opennlp.ccg.synsem.SignHash;
import opennlp.ccg.test.RegressionInfo; 

// -----------------------------------------------------------------
// REPRESENTATION IMPORTS
// -----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
	The class <b>TestSuiteUtils</b> provides a range of methods to 
	compute statistics over a test suite. 

	<h4>Grammar-level statistics</h4>
	
	<ul>
	<li> <b>Ambiguity</b>: lexical ambiguity per word, ambiguity limit 
		 (product of lexical ambiguities), #final analyses; 
		 over the test suite: distribution of lexical ambiguity, 
		 ambiguity limits, #final analyses per #sentences (possibility
		 to output to R-format tables, for statistical tests, bar charts), 
		 either for all categories or per category.  
		 </li>
	<li> <b>Parsing</b>: #analyses per step (unpruned chart, grammatically
		 scored chart), #final analyses; output to table for curve chart with 
		 plot bars (deviation) of #final analyses plotted to length of the 
		 sentence, of (per length grouping, per sentence) #analyses per step  
		 </li> 
	</ul>
	
	
	
	
*/

public class TestSuiteUtils {

	// =================================================================
	// GLOBAL DATA STRUCTURES
	// =================================================================

    // -----------------------------------------------------------------
	// GRAMMAR, PARSER RELATED DATA STRUCTURES
    // -----------------------------------------------------------------	

	/** The parser which prunes based on grammatical constraints */
	ActiveIncrCCGParser pruningParser;
	
	/** The parser which just uses the grammar to parse an utterance, no pruning */
	ActiveIncrCCGParser bareParser; 
	
	/** The lexicon */
	ActiveCCGLexicon grammar; 
	
	/** The grammar to be used by the parser and the lexicon */
	String grammarFile; 

    // The tokenizer.  (Defaults to DefaultTokenizer.) 
    private Tokenizer tokenizer;

    // -----------------------------------------------------------------
	// TEST-SUITE DATA STRUCTURES
	// -----------------------------------------------------------------

	/** The test suite */ 
	URL testSuiteURL; 

	/** The baseline test results, using the bare parser */
	Hashtable<String,TestSuiteUtils.UtteranceTestResults> baselineResults;

	/** The test results using a parser with (grammatical) pruning */
	Hashtable<String,TestSuiteUtils.UtteranceTestResults> pruningResults;

	/** The utterances grouped by length */
	Hashtable<Integer,Vector> lengthIndexMap; 

	/** Configurable: whether to show statistics for individual items on System.out */ 
	boolean showItemStats; 

	/** Configurable: whether to show statistics for the entire test suite on System.out */ 
	boolean showTSStats; 

    // -----------------------------------------------------------------
	// CONFIGURATION DATA STRUCTURES
    // -----------------------------------------------------------------	

	/** Whether to provide logging output */
	boolean logging;

	/** The directory to which statistics, tables should be written */ 
	String outputDir; 
	
	/** Vector with tests to be run */
	Vector tests; 
	
	/** Level of verbosity when logging output */
	int verbosityLevel; 
	
    // -----------------------------------------------------------------
	// INTERNAL CLASSES
    // -----------------------------------------------------------------		

	/**
		The (internal) class <b>UtteranceTestResults</b> collects the basic
		test results for an utterance: its lexical ambiguity (per string 
		position/word), its grammatical ambiguity limit, the number of analyses
		per string position, and the number of final analyses. 		
	*/ 

	public class UtteranceTestResults { 
		/** The test utterance */
		public PhonString utterance = null; 
		/** The lexical ambiguity, vector over Integer objects; position corresponds to string position */
		public Vector lexAmb = new Vector();
		/** The grammatical ambiguity limit */
		public int grammAmbLimit = -1;
		/** The number of final analyses */
		public int numFinalAnalyses = -1; 
		/** The number of intermediate analyses, per string position */
		public Vector numIntermAnalyses = new Vector(); 
		/** Parsing time */ 
		public Vector timing = new Vector();
		/** Total parsing time, in milliseconds */
		long totalTime = -1; 
	} // end internal class TestData

	/**		
		The (internal) class <b>TestSuiteResults</b> collects the combined results over
		the statistics for the individual utterances in the test suite: AVG, MAX, StandardDev (SD) over 
		final analyses per utterance length, over all lengths; per length, AVG, MAX, SD per position; 
		AVG, MAX, SD overall parse times per utterance length; 
	*/ 

	public class TestSuiteResults { 
		/** Hashtable with AVG over final analyses, per utterance length (key) */ 
		public Hashtable finalToLengthAVG = new Hashtable();
		/** Hashtable with MAX over final analyses, per utterance length (key) */ 
		public Hashtable finalToLengthMAX = new Hashtable();		
		/** Hashtable with SD over final analyses, per utterance length (key) */ 		
		public Hashtable finalToLengthSD = new Hashtable();		
		/** Hashtable with AVG over final PLF sizes, per utterance length (key) */ 
		public Hashtable finalPLFToLengthAVG = new Hashtable();
		/** Hashtable with MAX over final PLF sizes, per utterance length (key) */ 
		public Hashtable finalPLFToLengthMAX = new Hashtable();		
		/** Hashtable with SD over final PLF sizes, per utterance length (key) */ 		
		public Hashtable finalPLFToLengthSD = new Hashtable();		
		
		/** Hashtable with AVG over analyses per position, per utterance length (key) */ 
		public Hashtable positionAnalysesToLengthAVG = new Hashtable();				
		/** Hashtable with MAX over analyses per position, per utterance length (key) */ 
		public Hashtable positionAnalysesToLengthMAX = new Hashtable();						
		/** Hashtable with SD over analyses per position, per utterance length (key) */ 		
		public Hashtable positionAnalysesToLengthSD = new Hashtable();				
		/** Hashtable with AVG over time, per utterance length (key) */
		public Hashtable timeToLengthAVG = new Hashtable();						
		/** Hashtable with MAX over time, per utterance length (key) */
		public Hashtable timeToLengthMAX = new Hashtable();						
		/** Hashtable with SD over time, per utterance length (key) */
		public Hashtable timeToLengthSD = new Hashtable();								
	} // end internal class TestSuiteResults

	// =================================================================
	// CONSTRUCTORS
	// =================================================================

	/** Unary constructor. This method does not call <i>init</i> for initialization;
		that method should only be called after the class has been properly 
		configured. 
	*/

	public TestSuiteUtils () { 

	} // end constructor


	/** The method <i>init</i> initializes the internal variables. 
		This method should be called after <i>configure</i>, to ensure
		that external variables (e.g. pointers to grammar files) have
		been properly initialized based on the input from the command line.
	*/ 
	
	private void init() { 
		// Set logging, verbosity level
		verbosityLevel = 1; 
	
		System.out.print("Initializing grammar, parsers ");


	
		try { 
			// Parsing processes: set up the grammar
			grammar = new ActiveCCGLexicon();
			grammar.setLogLevel(grammar.LOG_FALSE);
			// initialize the grammar with an empty attention mechanism pipeline, 
			grammar.registerAttentionMechanism(new AttentionMechanismPipeline());		
			// set the grammar to be used
			grammar.setGrammar(grammarFile);
			
			System.out.print("..");
			// Parsing processes: set up the pruning parser, connect it to the grammar
			pruningParser	= new ActiveIncrCCGParser();
			pruningParser.setLogLevel(pruningParser.LOG_FALSE);			
			// Register the grammar: this initializes the connection with the lexicon, 
			// and creates the ccg parser with the corresponding openccg grammar
			pruningParser.registerGrammarAccess(grammar); 
			// initialize the parser with an empty attention mechanism pipeline, 
			// i.e no filtering (required as per ContextActiveProcess)
			pruningParser.registerAttentionMechanism(new AttentionMechanismPipeline());
			// initialize the parser with a category scorer on sign hashes, as post-parsing step pruning
			CategoryChartScorer cshs = new CategoryChartScorer();
			cshs.setLogging(false);
			cshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			pruningParser.registerChartScorer(cshs);
			// register a frontier filter that looks at categories, at the moment all pass so single-word parsing
			FrontierCatFilter cfilter = new FrontierCatFilter();
			cfilter.setLogging(false);
			pruningParser.registerFrontierFilter(cfilter);		
			System.out.print("..");			
			// Parsing processes: set up the bare parser
			bareParser	= new ActiveIncrCCGParser();
			bareParser.setLogLevel(pruningParser.LOG_FALSE);			
			bareParser.registerGrammarAccess(grammar); 
			bareParser.registerAttentionMechanism(new AttentionMechanismPipeline());
			EmptyChartScorer eshs = new EmptyChartScorer();
			eshs.setLogging(false);
			eshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			bareParser.registerChartScorer(eshs);
			FrontierCatFilter ecfilter = new FrontierCatFilter();
			ecfilter.setLogging(false);
			bareParser.registerFrontierFilter(ecfilter);			
			tokenizer = new DefaultTokenizer();
			System.out.print("..");			
		} catch (ComsysException ce) { 
			System.out.println("Failure to initialize: \n"+ce.getMessage()); 
			System.exit(0);
		} 
		// Test suite 
		baselineResults = new Hashtable<String,TestSuiteUtils.UtteranceTestResults>();		
		pruningResults	= new Hashtable<String,TestSuiteUtils.UtteranceTestResults>();
		lengthIndexMap  = new Hashtable<Integer,Vector>(); 
		System.out.println("done");
		System.out.println("---------------------------------------------------------");		
		
	} // end init
	
	// =================================================================
	// ACCESSOR METHODS
	// =================================================================

	/** The method <i>configure</i> takes a list of properties (obtained
		from parsing the command line) and configures the class. Minimal
		properties we need are: 
		<ul> 
		<li> --grammar: the grammar file </li> 
		<li> --testsuite: the test suite file </li> 
		</ul>
	*/

	public void configure (Properties pList) { 
	
		// Initialize the grammar
		grammarFile = "./grammar.xml";
		if (pList.containsKey("--grammar")) {
			grammarFile = pList.getProperty("--grammar"); 
		} // end if.. check for grammar
		
		// Initialize the test suite
		String tsFile = "./grammars/openccg/moloko.v4/testbed.xml";
		if (pList.containsKey("--testsuite")) { 
			tsFile = pList.getProperty("--testsuite");
		} // end if.. check for test suite
		try { 
			testSuiteURL = new File(tsFile).toURL();
		} catch (MalformedURLException me) { 
			System.out.println("[TestSuiteUtils] Malformed filename for test suite\n"+me.getMessage());
			System.out.println("We will unceremoniously exit.");
			System.exit(0);
		} // end try..catch
	
		// Initialize the directory for outputting graphs to	
		outputDir = "./graphs/testsuite/";
		if (pList.containsKey("--output")) {
			outputDir = pList.getProperty("--output"); 
		} // end if.. check for grammar		
	
		// Initialize showItemStats
		showItemStats = false;
		if (pList.containsKey("--showItemStats")) { 
			String status = pList.getProperty("--showItemStats"); 
			if (status.equals("true")) { showItemStats = true; }
		} 

		// Initialize showTSStats
		showTSStats = false;
		if (pList.containsKey("--showTestSuiteStats")) { 
			String status = pList.getProperty("--showTestSuiteStats"); 
			if (status.equals("true")) { showTSStats = true; }
		} 		
		
		// Initialize logging
		logging = false;
		if (pList.containsKey("--log")) { 
			String status = pList.getProperty("--log"); 
			if (status.equals("true")) { logging = true; }
		} 			
		
		
		System.out.println("---------------------------------------------------------");
		System.out.println("TESTSUITE CONFIGURATION\n");
		System.out.println("Grammar: ["+grammarFile+"]");
		System.out.println("Test suite: ["+tsFile+"]");		
		System.out.println("Output : ["+outputDir+"]");		
		System.out.println("Show item stats: ["+showItemStats+"]");
		System.out.println("Show test suite stats: ["+showTSStats+"]");		
		System.out.println("---------------------------------------------------------");
		
	} // end configure

	// =================================================================
	// COMPUTATION METHODS
	// =================================================================


	/** The method <i>runTests</i> takes a URL pointing to the test suite,
		and then runs the (selected) tests on the items in the test suite. 

		@param	tsURL The URL for the test suite
		@throws IOException If there is a problem reading from the test suite
	*/ 


	public void runTests () throws IOException { 
		// Load the test suite
		// The null argument is a pointer to the grammar; this is only 
		// necessary if we want to generate test items
		System.out.println("Test suite: ["+testSuiteURL+"]");
        RegressionInfo rinfo = new RegressionInfo(null, testSuiteURL.openStream());
        // Cycle through the test items in the test suite
        int numItems = rinfo.numberOfItems();
		System.out.println("Processing "+numItems+" item(s) in test suite");
		for (int i=0; i < numItems; i++) { 
			if (i%10 == 0) { System.out.print(i); } else { System.out.print("."); }
			// do some garbage collection ... 
			System.gc(); 
			// now get the item and do the tests
			RegressionInfo.TestItem testItem = rinfo.getItem(i);
			// Create a new id for the PhonString for the utterance, initialize its length too
			String phStrId = "ph"+i;
			List words = tokenizer.tokenize(testItem.sentence);
			PhonString phStr = new PhonString(phStrId,testItem.sentence,words.size(),1.0f, 1.0f, 1);

			try { 
				// Incrementally parse the sentence, baseline 
				PackedLFParseResults baseline = (PackedLFParseResults) bareParser.parse(phStr);						
				// Get the chart history
				Stack baselineHistory = bareParser.getChartHistory(phStrId);
				// Compute the statistics
				TestSuiteUtils.UtteranceTestResults baselineStats = this.computeStats(baselineHistory,phStr,bareParser);
				// Store the results
				baselineResults.put(phStrId,baselineStats);

				// Incrementally parse the sentence, with pruning
				PackedLFParseResults pruning = (PackedLFParseResults) pruningParser.parse(phStr);	
				// Get the chart history
				Stack pruningHistory = pruningParser.getChartHistory(phStrId);
				// Compute the statistics
				TestSuiteUtils.UtteranceTestResults pruningStats = this.computeStats(pruningHistory,phStr,pruningParser);
				// Store the results
				pruningResults.put(phStrId,pruningStats);
				
				// Add the id of the phon string to the length-to-index map
				Vector indices = new Vector();			
				Integer lengthKey = new Integer(phStr.length);
				if (lengthIndexMap.containsKey(lengthKey)) { 
					indices = (Vector) lengthIndexMap.get(lengthKey); 
				} // end if.. check for length as key
				indices.addElement(phStrId);
				lengthIndexMap.put(lengthKey,indices);
			} catch (ComsysException ce) { 
				log("Error while trying to parse ["+testItem.sentence+"]\n"+ce.getMessage());
			} catch (ParseException pe) { 
				log("Error while trying to parse ["+testItem.sentence+"]\n"+pe.getMessage());				
			}	
		} // end for over test items
		System.out.println("");
		System.out.println("---------------------------------------------------------");
		System.out.print("Computing statistics over the entire testsuite ..");
		TestSuiteUtils.TestSuiteResults baselineTSStats = this.computeTestSuiteStats(baselineResults,bareParser);
		System.out.print("..");
		TestSuiteUtils.TestSuiteResults pruningTSStats = this.computeTestSuiteStats(pruningResults,pruningParser);
		System.out.print(".. done");		
		System.out.println("");
		System.out.println("=====================================================");
		if (showTSStats) { 
			double avgLength = 0.0;
			Iterator lengthIter = lengthIndexMap.keySet().iterator();
			while (lengthIter.hasNext()) { 
				Integer length = (Integer) lengthIter.next();
				Vector indices = (Vector) lengthIndexMap.get(length);
				avgLength += (length.intValue()) * (indices.size()) ; 
			} // end while
			avgLength = (avgLength / numItems);
			System.out.println("Average utterance length in test suite: ["+avgLength+"]");
			System.out.println("BASELINE TEST SUITE STATISTICS:");
			this.showTSStats(baselineTSStats);
			System.out.println("\nPRUNING TEST SUITE STATISTICS:");
			this.showTSStats(pruningTSStats);
			System.out.println("=====================================================");			
		} 
		
		if (showItemStats) { 
			Iterator indexIter = baselineResults.keySet().iterator();
			while (indexIter.hasNext()) { 
				String index = (String) indexIter.next();
				TestSuiteUtils.UtteranceTestResults baselineData = (TestSuiteUtils.UtteranceTestResults) baselineResults.get(index);
				TestSuiteUtils.UtteranceTestResults pruningData  = (TestSuiteUtils.UtteranceTestResults) pruningResults.get(index);
				System.out.println("BASELINE STATISTICS:");
				this.showItemStats(baselineData);
				System.out.println("\nPRUNING STATISTICS:");
				this.showItemStats(pruningData);
				System.out.println("=====================================================");
			} // end while over indices
		} // end if.. check for showing item stats
		
		writeRTables("baseline",baselineTSStats);
		writeRTables("pruning",pruningTSStats);		
		
	} // end runTests


	/** 
		The method <i>computeStats</i> computes statistics over the chart history for a 
		given phonological string. 
		
		@param history	The chart history
		@param phonStr	The phonological string
		@return TestSuiteUtils.UtteranceTestResults The resulting statistics
	*/ 

	public TestSuiteUtils.UtteranceTestResults computeStats (Stack history, PhonString phonStr, ActiveIncrCCGParser parser) { 
		TestSuiteUtils.UtteranceTestResults results = new TestSuiteUtils.UtteranceTestResults();
		opennlp.ccg.parse.Chart topChart = (opennlp.ccg.parse.Chart)history.peek();
		// Set the utterance
		results.utterance = phonStr;
		// Compute the lexical ambiguity on the basis of the final chart
		results.lexAmb = retrieveLexAmb(topChart,parser.getChartSize(topChart));
		// The grammatical ambiguity limit
		results.grammAmbLimit = 1;
		for (int laC=0; laC < results.lexAmb.size(); laC++) { 
			results.grammAmbLimit = results.grammAmbLimit * ((Integer)results.lexAmb.elementAt(laC)).intValue(); 
		} // end 
		// The number of final analyses 
		SignHash topCell = topChart.getSigns(0,(results.lexAmb.size())-1);
		results.numFinalAnalyses = topCell.size();
		// The number of intermediate analyses, per string position
		results.numIntermAnalyses = this.retrieveIntermediateAnalyses(topChart,parser.getChartSize(topChart));
		// Parsing time
		results.timing = parser.getTiming(phonStr.id);
		// Total parsing time, in milliseconds 
		results.totalTime = 0; 
		for (int tC=0; tC < results.timing.size(); tC++) { 
			results.totalTime += ((Long)results.timing.elementAt(tC)).longValue();
		} // end for over timing elements
		return results; 
	} // end computeStats


	/**
		The method <i>computeTestSuiteStats</i> computes statistics over the given test 
		results for the entire test suite. 
		
		@param  testResults	The test results for the individual test items
		@return TestSuiteUtils.TestSuiteResults The results
	*/ 

	public TestSuiteUtils.TestSuiteResults computeTestSuiteStats (Hashtable testResults, ActiveIncrCCGParser parser) { 
		// initialize the results
		TestSuiteUtils.TestSuiteResults results = new TestSuiteUtils.TestSuiteResults();
		// Compute the AVG, MAX, and SD over the final analyses, per utterance length
		// System.out.println("Length index map size: "+lengthIndexMap.size());
		Iterator lengthIter = lengthIndexMap.keySet().iterator();
		while (lengthIter.hasNext()) {
			Integer length = (Integer) lengthIter.next();		 
			// initialize helper variables for num. final analyses
			double AVG = 0.0;
			double MAX = 0.0;
			double SD  = 0.0;
			// initialize helper variables for timing
			long timeAVG = 0;
			long timeMAX = 0;
			double timeSD  = 0;			
			// initialize helper variables for size final packed LFs
			double plfAVG = 0.0;
			double plfMAX = 0.0;
			double plfSD  = 0.0;			
		
			// cycle over the indices for the given length		
			Vector indices = (Vector) lengthIndexMap.get(length);
			Vector finalAnalyses = new Vector();
			Vector finalTimings  = new Vector();
			Vector finalPLFSizes = new Vector();
			Iterator indexIter = indices.iterator();
			while (indexIter.hasNext()) { 
				String index = (String) indexIter.next();
				// get the results for the given utterance
				TestSuiteUtils.UtteranceTestResults uttResults = (TestSuiteUtils.UtteranceTestResults) testResults.get(index);
				// compute the avg (sum) and max
				AVG += uttResults.numFinalAnalyses; 
				finalAnalyses.addElement(new Integer(uttResults.numFinalAnalyses));
				if (uttResults.numFinalAnalyses > MAX) { MAX = uttResults.numFinalAnalyses; }
				// get the timing data from the parser for the given phon string object
				Vector timingData = parser.getTiming(uttResults.utterance.id);
				Long finalTime = (Long) timingData.lastElement();
				timeAVG += finalTime.longValue();
				if (timeMAX < finalTime.longValue()) { timeMAX = finalTime.longValue(); } 
				finalTimings.addElement(new Integer(finalTime.intValue()));
				// get the packed LF data from the parser for the given phon string object
				PackedLogicalForm plf = parser.getFinalPackedLF(index);
				if (plf != null) { 
					Integer size = new Integer(plf.pNodes.length);
					finalPLFSizes.addElement(size);
					plfAVG += size.doubleValue();
					if (size.intValue() > plfMAX) { plfMAX = size.doubleValue(); } 
				} else { 
					log("Null PLF for phon string with id ["+index+"]");
				} // end if.. check for non-null plf
			} // end while
			// compute avg, standard deviation over number of final analyses
			AVG = (AVG / indices.size());
			SD  = sigmaI(finalAnalyses,AVG);
			results.finalToLengthAVG.put(length,new Double(AVG));
			results.finalToLengthMAX.put(length,new Double(MAX));
			results.finalToLengthSD.put(length,new Double(SD));
			// compute avg, standard deviation over number of final packing nodes
			plfAVG = (plfAVG / indices.size());
			plfSD  = sigmaI(finalPLFSizes,plfAVG);
			results.finalPLFToLengthAVG.put(length,new Double(plfAVG));
			results.finalPLFToLengthMAX.put(length,new Double(plfMAX));
			results.finalPLFToLengthSD.put(length,new Double(plfSD));
			// compute avg, standard deviation over timing for final analyses 
			timeAVG = (timeAVG / indices.size());
			timeSD  = sigmaI(finalTimings,timeAVG);
			results.timeToLengthAVG.put(length,new Long(timeAVG));
			results.timeToLengthMAX.put(length,new Long(timeMAX));
			results.timeToLengthSD.put(length,new Double(timeSD));
		} // end while over the different lengths

		// Compute the AVG, MAX, SD over analyses per position, per utterance length. 
		
		Iterator lengthIter2 = lengthIndexMap.keySet().iterator();
		while (lengthIter2.hasNext()) { 
			Integer length = (Integer) lengthIter2.next();		 
			// initialize helper variables
			Vector dAVG = initializeZeroDVector(length); // vector over Double objects
			Vector dMAX = initializeZeroDVector(length); // vector over Double objects
			Vector dSD  = initializeZeroDVector(length);	// vector over Double objects		
			Hashtable<Integer,Vector> positionScores = new Hashtable<Integer,Vector>(); 
			Vector indices = (Vector) lengthIndexMap.get(length);
			Iterator indexIter = indices.iterator();
			while (indexIter.hasNext()) { 
				String index = (String) indexIter.next();
				TestSuiteUtils.UtteranceTestResults uttResults = (TestSuiteUtils.UtteranceTestResults) testResults.get(index);
				Iterator imAnIter = uttResults.numIntermAnalyses.iterator(); 
				int stringPos = 0; 
				while (imAnIter.hasNext()) { 
					Integer numIntermAnalyses = (Integer) imAnIter.next();
					// Update the average (total)
					double avg = ((Double)dAVG.elementAt(stringPos)).doubleValue();
					avg += numIntermAnalyses.doubleValue();
					dAVG.setElementAt(new Double(avg),stringPos);
					// Update the maximum 
					double max = ((Double)dMAX.elementAt(stringPos)).doubleValue();
					if (numIntermAnalyses.doubleValue() > max) { dMAX.setElementAt(new Double(numIntermAnalyses),stringPos); }
					// Update the list of scores for this position
					Vector scores = new Vector();
					Integer posKey = new Integer(stringPos);
					if (positionScores.containsKey(posKey)) { scores = (Vector) positionScores.get(posKey); }
					scores.addElement(numIntermAnalyses);
					positionScores.put(posKey,scores);
					// Incr the string position
					stringPos++;
				} // end while over intermediate analyses
			} // end while over indices
			// Now update the AVG and compute the SD
			try { 
				for (int avgC=0; avgC < dAVG.size()-1; avgC++) { 
					double avg = ((Double) dAVG.get(avgC)).doubleValue();
					avg = (avg / indices.size());
					dAVG.setElementAt(new Double(avg),avgC);
				} // end while over the AVG scores
			} catch (ArrayIndexOutOfBoundsException ae) { } 
			Iterator posIter = positionScores.keySet().iterator();
			while (posIter.hasNext()) { 
				Integer posKey = (Integer) posIter.next();
				Vector scores  = (Vector) positionScores.get(posKey); 
				try { 
					Double avgD = (Double)dAVG.elementAt(posKey.intValue());
					if (avgD != null) { 
						double avg = avgD.doubleValue();
						dSD.setElementAt(new Double(sigmaI(scores,avg)),posKey.intValue());
					} // end if.. check valid value
				} catch (ArrayIndexOutOfBoundsException e) { 
					log(e.getMessage()); 
				} // end try..catch
			} // end while over positions in utterances of the current length
			results.positionAnalysesToLengthAVG.put(length,dAVG);
			results.positionAnalysesToLengthMAX.put(length,dMAX);
			results.positionAnalysesToLengthSD.put(length,dSD);
		} // end while over different lengths

		return results;
	} // end computeTestSuiteStats

	private Vector initializeZeroVector (int size) { 
		Vector result = new Vector(size);
		for (int i=0; i < size; i++) { 
			result.addElement(new Integer(0));
		} // end for
		return result;
	} // end initializeZeroVector

	private Vector initializeZeroDVector (int size) { 
		Vector result = new Vector(size);
		for (int i=0; i < size; i++) { 
			result.addElement(new Double(0));
		} // end for
		return result;
	} // end initializeZeroVector


	/** 
		The method <i>retrieveIntermediateAnalyses</i> retrieves, for each string position, the number of intermediate analyses
		at that position. This number is the size of the SignHash stored at (0,string position) in the chart. 
	*/ 
	
	private Vector retrieveIntermediateAnalyses (Chart chart, int size) { 
		Vector results = new Vector();
		for (int i = 0; i < size; i++) { 
			SignHash topHash = chart.getSigns(0,i); 
			results.add(i,new Integer(topHash.size())); 
		} // end for over chart elements
		return results; 
	} // end retrieveIntermediateAnalyses

	/** 
		The method <i>retrieveLexAmb</i> retrieves, for each word i.e. string position, the number of lexical entries
		stored at that position. These numbers are returned as a Vector over Integer objects. The position in the Vector 
		corresponds to the string position. 
		
		@param chart The chart 
		@return Vector A Vector over Integer objects with the number of lexical entries per string position 
	*/

	private Vector retrieveLexAmb (Chart chart, int size) { 
		Vector results = new Vector();
		for (int i = 0; i < size; i++) { 
			SignHash lexHash = chart.getSigns(i,i); 
			results.add(i,new Integer(lexHash.size())); 
		} // end for over chart elements
		return results; 
	} // end retrieveLexAmb
	
    /** Returns the standard deviation over a vector of Doubles */
    public double sigmaD (Vector data, double mean) {
        if (data.size() < 2) return -1; // NA
        double numerator = 0;
        for (int i = 0; i < data.size(); i++) {
            Double number = (Double)data.get(i);
            numerator += Math.pow(number.doubleValue() - mean, 2);
        }
        int denominator = data.size() - 1;
        return Math.sqrt(numerator / denominator);
    } // end 	
	
    /** Returns the standard deviation over a vector of Integer objects */
    public double sigmaI (Vector data, double mean) {
        if (data.size() < 2) return -1; // NA
        double numerator = 0;
        for (int i = 0; i < data.size(); i++) {
            Integer number = (Integer)data.get(i);
            numerator += Math.pow(number.doubleValue() - mean, 2);
        }
        int denominator = data.size() - 1;
        return Math.sqrt(numerator / denominator);
    } // end 		

	// =================================================================
	// I/O METHODS
	// =================================================================

	/** The method <i>cLineArgsToConfiguration</i> parses the command line
		arguments to a Properties object, to be used to configure the 
		class. 
	*/ 
	
	public Properties cLineArgsToConfiguration (String[] args) { 
		Properties result = new Properties();
		for (int i=0; i < args.length; i++) { 
			String cArg = args[i];
			// check whether we have an option
			if (cArg.startsWith("-")) { 
				// check whether the option has a value
				if (i < args.length-1) { 
					String oVal = args[i+1];
					if (oVal.startsWith("-")) { 
						// set the option to be true
						result.setProperty(cArg,"true"); 
					} else {
						// set the option with the given value
						result.setProperty(cArg,oVal);
						// increment i to look past the retrieved value
						i++;
					} // end if..else check for option/value
				} // end if.. check for option value availability
			} // end if
		} // end for over arguments
		return result;
	} // end cLineArgsToConfiguration


	/** The method <i>log</i> prints the given message to System.out if
		logging has been turned on. The assumed log-level of the message 
		is 3, i.e. the highest level of verbosity. 
		
		@param msg	The message to be printed out
	*/
	public void log(String msg) { 
		if (logging && verbosityLevel == 3) { 
			System.out.println("[TestSuiteUtils] "+msg); 
		} // end if.. check whether to print
	} // end log

 	/** The method <i>log</i> prints the given message to System.out if
		logging has been turned on, and the given log-level is at least 
		as high as the set level of verbosity. 
		
		@param l	The level of verbosity
		@param msg	The message to be printed out
	*/
	public void log(int l, String msg) { 
		if (logging && l >= verbosityLevel) { 
			System.out.println("[TestSuiteUtils] "+msg); 
		} // end if.. check whether to print
	} // end log


	/**
		The method <i>showItemStats</i> prints the statistics for 
		each individual item to System.out
		
		@param data		The data to be printed
	*/ 

	public void showItemStats (TestSuiteUtils.UtteranceTestResults data) { 
		System.out.println("-------------------------------------------------");
		System.out.println("\""+data.utterance.wordSequence+"\"\n");
		System.out.println("Lexical ambiguity:");
		Iterator lexAmbIter = data.lexAmb.iterator();
		int i = 0;
		while (lexAmbIter.hasNext()) { 
			Integer ambI = (Integer) lexAmbIter.next();
			System.out.print("["+i+":"+ambI+"] ");
			i++;
		} // end while over lexical ambiguities
		System.out.println("");
		System.out.println("Grammatical ambiguity limit: "+data.grammAmbLimit);
		System.out.println("Number of final analyses: "+data.numFinalAnalyses);
		System.out.println("Number of intermediate analyses, per string position: ");
		Iterator imAnIter = data.numIntermAnalyses.iterator();
		int j = 0;
		while (imAnIter.hasNext()) { 
			Integer imAnI = (Integer) imAnIter.next();
			System.out.print("["+j+":"+imAnI+"] ");
			j++;
		} // end while over intermediate analyses
		System.out.println("");
		System.out.println("Timing of intermediate analyses, per string position: ");
		Iterator timeIter = data.timing.iterator();
		int k = 0;
		while (timeIter.hasNext()) { 
			Long timeI = (Long) timeIter.next();
			System.out.print("["+k+":"+timeI+"] ");
			k++;
		} // end while over timing
		System.out.println("");		
		System.out.println("Total parsing time: "+data.totalTime); 
		System.out.println("");				
	} // end showItemStats


	/**
		The method <i>showTSStats</i> prints the overall statistics for 
		the test suite to System.out
		
		@param data		The data to be printed
	*/ 

	public void showTSStats (TestSuiteUtils.TestSuiteResults data) { 
		System.out.println("-------------------------------------------------");
		System.out.println("Length distribution: ");
		Iterator lengthIter = lengthIndexMap.keySet().iterator();
		while (lengthIter.hasNext()) { 
			Integer length	= (Integer) lengthIter.next();
			Vector  indices = (Vector) lengthIndexMap.get(length);
			System.out.println("Length ["+length+"]: ["+indices.size()+"] items");
		} // end while over length distribution 
		System.out.println("");
		System.out.println("AVG, MAX, SD over final analyses, per utterance length: ");
		Iterator AMSIter = data.finalToLengthAVG.keySet().iterator();
		while (AMSIter.hasNext()) { 
			Integer length	= (Integer) AMSIter.next();
			Double avgD = (Double) data.finalToLengthAVG.get(length);
			Double maxD = (Double) data.finalToLengthMAX.get(length);			
			Double sdD = (Double) data.finalToLengthSD.get(length);		
			System.out.println("Length ["+length+"]: AVG ["+avgD+"] MAX ["+maxD+"] SD ["+sdD+"]");
		} // end while over 
		System.out.println("");		
		System.out.println("AVG, MAX, SD over PLF sizes, per utterance length: ");
		Iterator plfAMSIter = data.finalPLFToLengthAVG.keySet().iterator();
		while (plfAMSIter.hasNext()) { 
			Integer length	= (Integer) plfAMSIter.next();
			Double avgD = (Double) data.finalPLFToLengthAVG.get(length);
			Double maxD = (Double) data.finalPLFToLengthMAX.get(length);			
			Double sdD = (Double) data.finalPLFToLengthSD.get(length);		
			System.out.println("Length ["+length+"]: AVG ["+avgD+"] MAX ["+maxD+"] SD ["+sdD+"]");
		} // end while over 
		System.out.println("");			

		System.out.println("");		
		System.out.println("AVG, MAX, SD over number of intermediate analyses, per string position per utterance length: ");
		Iterator posAMSIter = data.positionAnalysesToLengthAVG.keySet().iterator();
		while (posAMSIter.hasNext()) { 
			Integer length	= (Integer) posAMSIter.next();
			Vector avgV = (Vector) data.positionAnalysesToLengthAVG.get(length);
			Vector maxV = (Vector) data.positionAnalysesToLengthMAX.get(length);			
			Vector sdV	= (Vector) data.positionAnalysesToLengthSD.get(length);		
			System.out.println("Length ["+length+"]: \nAVG ["+avgV+"] \nMAX ["+maxV+"] \nSD ["+sdV+"]");
		} // end while over 
		System.out.println("");	
		
		System.out.println("");		
		System.out.println("AVG, MAX, SD over final parsing times, per utterance length: ");
		Iterator timeAMSIter = data.timeToLengthAVG.keySet().iterator();
		while (timeAMSIter.hasNext()) { 
			Integer length	= (Integer) timeAMSIter.next();
			Long avgL = (Long) data.timeToLengthAVG.get(length);
			Long maxL = (Long) data.timeToLengthMAX.get(length);			
			Double sdD =  (Double) data.timeToLengthSD.get(length);		
			System.out.println("Length ["+length+"]: AVG ["+avgL+"] MAX ["+maxL+"] SD ["+sdD+"]");
		} // end while over 
		System.out.println("");	
	} // end showTSStats

	/** 
		The method <i>writeRTables</i> writes out tables to the output dir, for later use with R. 
		The following table data files are created: 
		<ul> 
		<li> Sentence length to number of final analyses
		<li> Sentence length to size of final packed logical form
		<li> Sentence length to maximum number of final analyses
		<li> Sentence length to maximum number of intermediate analyses
		<li> Sentence length to average total time
		<li> Sentence length to maximum total time 
		</ul> 
		
		@param fileNamePrefix The prefix to be used for creating file names for the individual tables
	*/ 

	public void writeRTables (String fileNamePrefix, TestSuiteUtils.TestSuiteResults data) { 
		System.out.println("");
		System.out.println("-------------------------------------------------");
		System.out.println("WRITING R TABLES ["+outputDir+"]\n");
		// Sentence length to avg, max number of final analyses, and sd
		writeRTableLengthToFinalAnalyses(fileNamePrefix,data);
		// Sentence length to avg, max size of final packed logical form, and sd
		writeRTableLengthToPLFSizes(fileNamePrefix,data);
		// Sentence length to maximum number of intermediate analyses
		writeRTableLengthToMAXIntermediate(fileNamePrefix,data);
		// Sentence length to avg, max total time
		writeRTableLengthToFinalTime(fileNamePrefix,data);
	} // end 

	private void writeRTableLengthToFinalAnalyses (String fileNamePrefix, TestSuiteUtils.TestSuiteResults data) { 
		String outFileName = outputDir+fileNamePrefix+"-lengthToFinalAnalyses.data";
		System.out.println("Writing table data ["+outFileName+"]");
		try
		{ 
			// Initialize the data output stream
			PrintStream out = new PrintStream(
				new BufferedOutputStream(
				new FileOutputStream( outFileName )));
			// Write the file header	
			out.println("length \t AVG \t MAX \t SD");
			// Iterate over the length
			Iterator lengthIter = data.finalToLengthAVG.keySet().iterator();
			while (lengthIter.hasNext()) { 
				Integer length = (Integer) lengthIter.next();
				Double avgD = (Double) data.finalToLengthAVG.get(length);
				Double maxD = (Double) data.finalToLengthMAX.get(length);			
				Double sdD = (Double) data.finalToLengthSD.get(length);		
				out.println(length+" \t "+avgD+" \t "+maxD+" \t "+sdD);				
			} // end while over length clusters
			out.close();
	   } catch (IOException iox ) {
			System.out.println("Problem writing " + outFileName );
	   } // end try..catch		
	} // end writeRTableLengthToFinalAnalyses


	private void writeRTableLengthToPLFSizes (String fileNamePrefix, TestSuiteUtils.TestSuiteResults data) { 
		String outFileName = outputDir+fileNamePrefix+"-lengthToFinalPLFSizes.data";
		System.out.println("Writing table data ["+outFileName+"]");
		try
		{ 
			// Initialize the data output stream
			PrintStream out = new PrintStream(
				new BufferedOutputStream(
				new FileOutputStream( outFileName )));
			// Write the file header	
			out.println("length \t AVG \t MAX \t SD");
			// Iterate over the length
			Iterator lengthIter = data.finalToLengthAVG.keySet().iterator();
			while (lengthIter.hasNext()) { 
				Integer length = (Integer) lengthIter.next();
				Double avgD = (Double) data.finalPLFToLengthAVG.get(length);
				Double maxD = (Double) data.finalPLFToLengthMAX.get(length);			
				Double sdD = (Double) data.finalPLFToLengthSD.get(length);		
				out.println(length+" \t "+avgD+" \t "+maxD+" \t "+sdD);				
			} // end while over length clusters
			out.close();
	   } catch (IOException iox ) {
			System.out.println("Problem writing " + outFileName );
	   } // end try..catch		
	} // end writeRTableLengthToPLFSizes

	private void writeRTableLengthToMAXIntermediate (String fileNamePrefix, TestSuiteUtils.TestSuiteResults data) { 
		String outFileName = outputDir+fileNamePrefix+"-lengthToMAXIntermediate.data";
		System.out.println("Writing table data ["+outFileName+"]");
		try
		{ 
			// Initialize the data output stream
			PrintStream out = new PrintStream(
				new BufferedOutputStream(
				new FileOutputStream( outFileName )));
			// Write the file header	
			out.println("length \t MAX");
			// Iterate over the length
			Iterator lengthIter = data.finalToLengthAVG.keySet().iterator();
			while (lengthIter.hasNext()) { 
				Integer length = (Integer) lengthIter.next();
				Vector maxV = (Vector) data.positionAnalysesToLengthMAX.get(length);
				Iterator maxIter = maxV.iterator();
				double MAX = 0;
				while (maxIter.hasNext()) { 
					double max = ((Double) maxIter.next()).doubleValue();
					if (max > MAX) { MAX = max; }
				} // end over maxima
				out.println(length+" \t "+MAX);
			} // end while over length clusters
			out.close();
	   } catch (IOException iox ) {
			System.out.println("Problem writing " + outFileName );
	   } // end try..catch		
	} // end writeRTableLengthToMAXIntermediate

	private void writeRTableLengthToFinalTime (String fileNamePrefix, TestSuiteUtils.TestSuiteResults data) { 
		String outFileName = outputDir+fileNamePrefix+"-lengthToFinalTime.data";
		System.out.println("Writing table data ["+outFileName+"]");
		try
		{ 
			// Initialize the data output stream
			PrintStream out = new PrintStream(
				new BufferedOutputStream(
				new FileOutputStream( outFileName )));
			// Write the file header	
			out.println("length \t AVG \t MAX \t SD");
			// Iterate over the length
			Iterator lengthIter = data.finalToLengthAVG.keySet().iterator();
			while (lengthIter.hasNext()) { 
				Integer length = (Integer) lengthIter.next();
				Long avgL = (Long) data.timeToLengthAVG.get(length);
				Long maxL = (Long) data.timeToLengthMAX.get(length);			
				Double sdD = (Double) data.timeToLengthSD.get(length);		
				out.println(length+" \t "+avgL+" \t "+maxL+" \t "+sdD);				
			} // end while over length clusters
			out.close();
	   } catch (IOException iox ) {
			System.out.println("Problem writing " + outFileName );
	   } // end try..catch		
	} // end writeRTableLengthToFinalTime
	

	// =================================================================
	// MAIN METHOD
	// =================================================================

	public static void main (String[] args) { 
		TestSuiteUtils ts = new TestSuiteUtils();
		Properties pl = ts.cLineArgsToConfiguration(args);
		ts.configure(pl);
		ts.init();
		try { 
			ts.runTests();
		} catch (IOException e) { 
			e.printStackTrace();
		} 


	} // end main

} // end class
