package comsys.processing.parseselection;

import java.io.File;
import java.io.FileReader;
import java.util.Hashtable;
import java.util.Properties;
import java.util.StringTokenizer;
import java.util.Vector;
import java.util.Enumeration;
import java.util.Iterator;

import opennlp.ccg.grammar.AbstractRule;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

import com.sun.org.apache.xerces.internal.parsers.DOMParser;

import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.comsysEssentials.PhonStringLFPair;
import comsys.processing.parse.PackedLFParseResults;
import comsys.processing.parse.examplegeneration.CFG;
import comsys.processing.parse.examplegeneration.GenerationUtils;
import comsys.processing.parse.examplegeneration.SemanticCorpusGenerator;
import comsys.utils.ComsysUtils;

// import de.dfki.lt.mary.client.MaryClient;
import java.util.Arrays;

public class DiscriminativeLearner {

	SemanticCorpusGenerator generator;
	CCGParser parser;
	int nbIterations;
	ParameterVector params;
	AveragedPerceptron perceptron;
	Decoder decoder;
	CFG cfg;
	String outputParamFile;
	boolean getStatistics = false;
	String outputStatisticsFile;
	int nbGoodExamples = 0;
	String genCorpusFile;

	static boolean logging = true;

	String outputTranscriptionsFile;
	String wavfilesDir;

	String learningType;

	public DiscriminativeLearner(Properties properties) {
		initialization(properties);
	}


	public void startLearning() {
		if (learningType.equals("online")) {
			log("Learning process: ONLINE\n");			
			onlineLearning();
			storeFinalOutput();
		}

		else if (learningType.equals("batch")) {
			log("Learning process: BATCH\n");
			batchLearning();
			storeFinalOutput();
		}

		else if (learningType.equals("generateSynthesizedCorpus")) {
			log("Learning process: NONE, only generate and synthesize corpus\n");
			generateCorpus();
			storeFinalOutput();
		}

		else if (learningType.equals("generateParses")) {
			log("Learning process: NONE, only generate new input parses\n");
			generateCorrectParses();
		} 

		else {
			log("FATAL ERROR: learning type " + learningType + 
			" doesn't exist, aborting!\n");
		}
	}


	public static String bold(String str) {
		return "\033[1;34m"+ str + "\033[0;34m";
	}


	public void generateCorrectParses() {

		DOMParser parser = new DOMParser();
		File f = new File("tests/data/utterancesToSelect.xml");
		String text = GenerationUtils.readFile("tests/data/utterancesToSelect.xml");

		try {
			FileReader reader = new FileReader(f);
			InputSource source = new InputSource(reader);
			parser.parse(source);
			Document testDoc = parser.getDocument();
			NodeList childNodes = testDoc.getChildNodes().item(0).getChildNodes();
			String topCategory = testDoc.getChildNodes().item(0).getNodeName();

			if(topCategory.equals("utterancesToSelect")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						NodeList parameterNodes = uttNode.getChildNodes();
						for (int j = 0; j < parameterNodes.getLength(); j++) {
							Node parameter = parameterNodes.item(j);

							if (parameter.getNodeName().equals("transcription")) {
								String str = parameter.getFirstChild().getNodeValue();
								String str2 = GenerationUtils.modifyCase2(str);
								str2 = GenerationUtils.removeDisfluencies(str2);
								String semantics = cfg.getSemantics(str2);

								semantics = semantics.replace("<", "/").replace(">", "\\");
								String transcriptiontag = "<transcription>"+str+"</transcription>";
								String semanticstag = "\t\t<parse>"+semantics+"</parse>";
								text = text.replace(transcriptiontag, transcriptiontag+"\n"+semanticstag);
							}
						}
					}
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		GenerationUtils.writeFile(text, "tests/data/utterancesToSelect.xml");
	}

	public static String bold(int value) {
		return bold(""+value);
	}


	private void initialization(Properties properties) {
		log("STEP 0: Initializing the semantic corpus generator...");
		generator = new SemanticCorpusGenerator(properties);
		log("Initialization:\t\t\t" + bold("OK\n"));


		log("STEP 1: Retrieving the context-free grammar...");
		cfg = generator.retrieveCFG();
		log("Grammar retrieval:\t\t\t" + bold("OK\n"));

		if (cfg != null) {
			log("STEP 2: Weights assignment");
			generator.assignWeights(cfg);
			GenerationUtils.weightHeuristics(cfg);
			log("Weights assignment:\t\t\t" + bold("OK\n"));

			generator.genCorpusSize = 1;

			log("STEP 3: Parameter learning");

			parser = new CCGParser();
			if (properties.get("--CCGGrammarFile") != null) {
				parser.defaultGrammar = (String)properties.get("--CCGGrammarFile");
				log("- Using CCG grammar specified in file " + parser.defaultGrammar);
			}
			else {
				log("FATAL ERROR: CCG grammar not specified\n");
				System.exit(0);
			}

			if (properties.get("--inputParamFile") != null) { 
				String inputParamFile  = properties.getProperty("--inputParamFile");
				log("- Extracting existing parameter values from input file " + inputParamFile);
				Vector<String> lines = LearningUtils.readFile(inputParamFile);
				params = LearningUtils.buildParameterVector(lines);
				log("  Extraction successful, number of features: " + bold(params.size()));
			}
			else {
				log("- Constructing new parameter vector...");
				params = new ParameterVector();
				log("  Construction of new parameter vector successful");
			}


			if (properties.get("--chartScoring") != null) 
				parser.chartScoring = new Boolean((String)properties.get("--chartScoring")).booleanValue();				
			else 
				parser.chartScoring = true;
			log("- Chart scoring set to: " + bold(""+parser.chartScoring));

			nbIterations = 1;
			if (properties.get("--nbIterations") != null) 
				nbIterations = (new Integer((String)properties.get("--nbIterations"))).intValue();
			log("- Total number of iterations to consider: " + bold(nbIterations));

			if (properties.get("--paramInitValue") != null) 
				params.initialValue  = (new Double((String)properties.get("--paramInitValue"))).doubleValue();
			else
				params.initialValue = 5;
			log("- Initial value set for new features in parameter vector: " + bold(""+params.initialValue) + "\n");

			if (properties.get("--outputParamFile") != null) {
				outputParamFile = (String)properties.get("--outputParamFile");
			}

			if (properties.get("--getStatistics") != null) {
				getStatistics = (new Boolean((String)properties.get("--getStatistics"))).booleanValue();
			}

			if (properties.get("--outputStatisticsFile") != null) {
				outputStatisticsFile = (String)properties.get("--outputStatisticsFile");
			}

			if (properties.get("--genCorpusFile") != null) {
				genCorpusFile = (String)properties.get("--genCorpusFile");
			}

			if (properties.get("--outputTranscriptionsFile") != null) {
				outputTranscriptionsFile = (String)properties.get("--outputTranscriptionsFile");
			}

			if (properties.get("--wavfilesDir") != null) {
				wavfilesDir = (String)properties.get("--wavfilesDir");
			}

			if (properties.get("--learning") != null) {
				learningType = (String)properties.get("--learning");
			}

			perceptron = new AveragedPerceptron(params);
			decoder = new Decoder(params);
			perceptron.setDecoder(decoder);
			decoder.setParser(parser);
			decoder.logging = true;

			if (properties.get("--extractSemanticFeatures") != null) {
				decoder.extractSemanticFeatures = (new Boolean((String)properties.get("--extractSemanticFeatures"))).booleanValue();
			}
			log("Extraction of semantic features: " + bold(""+decoder.extractSemanticFeatures));

			if (properties.get("--extractSyntacticFeatures") != null) {
				decoder.extractSyntacticFeatures = (new Boolean((String)properties.get("--extractSyntacticFeatures"))).booleanValue();
			}
			log("Extraction of syntactic features: " + bold(""+decoder.extractSyntacticFeatures));

			if (properties.get("--extractAcousticFeatures") != null) {
				decoder.extractAcousticFeatures = (new Boolean((String)properties.get("--extractAcousticFeatures"))).booleanValue();
			}
			log("Extraction of acoustic features: " + bold(""+decoder.extractAcousticFeatures));

			if (properties.get("--extractContextualFeatures") != null) {
				decoder.extractContextualFeatures = (new Boolean((String)properties.get("--extractContextualFeatures"))).booleanValue();
			}
			log("Extraction of contextual features: " + bold(""+decoder.extractContextualFeatures));

			if (properties.get("--extractNoParseFeature") != null) {
				decoder.extractNoParseFeature = (new Boolean((String)properties.get("--extractNoParseFeature"))).booleanValue();
			}
			log("Extraction of \"no parse\" feature: " + bold(""+decoder.extractNoParseFeature));

			log("");
			
			if (properties.get("--asr-correction") != null) {
				AbstractRule.asrcorrectionRules = (new Boolean((String)properties.get("--asr-correction"))).booleanValue();
			}
			log("- Use of ASR correction rules: " + bold("" + AbstractRule.asrcorrectionRules));
			
			if (properties.get("--discourse-level-composition") != null) {
				AbstractRule.disclevelcompositionRules = (new Boolean((String)properties.get("--discourse-level-composition"))).booleanValue();
			}
			log("- Use of discourse-level composition rules: " + bold("" + AbstractRule.disclevelcompositionRules));
			
			if (properties.get("--disfluency-correction") != null) {
				AbstractRule.disfluencycorrectionRules = (new Boolean((String)properties.get("--disfluency-correction"))).booleanValue();
			}
			log("- Use of disfluency correction rules: " + bold("" + AbstractRule.disfluencycorrectionRules));
			
			log("");
			
			if (properties.get("--beamwidth") != null) {
				int beamwidth = new Integer((String)properties.get("--beamwidth")).intValue();
				parser.setBeamwidth(beamwidth, params);
				log("- Beam width for incremental chart scoring: " + bold(""+beamwidth));
			}
			log("");
		}
	}

	/**
	 * Online perceptron learning
	 * @param properties
	 */
	public void onlineLearning() {	

		nbGoodExamples = 0;
		for (int i = 0; i < nbIterations; i++) {
			log("\t-----------");
			log("\tIteration number:" + (i+1));

			String exampleString = generator.retrieveRandomSentence(cfg);
			TrainingExample example = new TrainingExample(exampleString);

			log("\tUtterance to learn: \"" + example.getFormattedUtterance() +"\"");
			log("\tVerifying semantics using CCG grammar...\t" );

			int phonIncr = 0;
			
			if (parser.verifyExample(example, true)) {

				phonIncr++;
				boolean hasBeenUpdated = false;
				
				while (!hasBeenUpdated && 
						example.expectedUtteranceStringPos < example.getExpectedUtterance().length) {

					PackedLFParseResults results = parser.parse(example.getExpectedUtterance(), example.expectedUtteranceStringPos);
					example.expectedUtteranceStringPos++;
					
					log("\tCurrent incremental analysis: " + example.getFormattedUtterance());
					
					if (results != null) {
					NonStandardRulesAppliedForLF[] nonStandardRulesForLf = 
						ComsysUtils.convertNonStandardRulesDataStructs(results);

					PackedLFs plf = new PackedLFs("bla",new PhonStringLFPair[0], new PhonString[0], results.stringPos, 
							results.plf, results.finalized, "interpretation", nonStandardRulesForLf);

					params = LearningUtils.incrInitPV(example, params, plf);

					try {
						params = perceptron.learn(example);
						hasBeenUpdated = perceptron.hasBeenUpdated();
					}
					catch (Exception ee) {
						ee.printStackTrace();
					}

					nbGoodExamples++;
					}
					else {
						log("Not parsable: " + example.getIncrementalUtterance());
					}
				}
				log("\t--> Example successfully learned\n");
			}
			else {
				log("\t--> Example discarded (too long, not parsable, or pruned)\n");
			}
			
		      Runtime r = Runtime.getRuntime();
		      r.gc();
		}
		parser.phonIncr++;
	}


	public void generateCorpus() {

		String transcriptionsText = "";

	//	MaryClient mary;
		String serverHost = "localhost";
		String voiceName = "jmk-arctic";
		int serverPort = 59125;

		int nbexistingfiles = 0;

		if (wavfilesDir != null) {
			File dir = new File(wavfilesDir);
			File[] files = dir.listFiles();

			for (int i = 0; i < files.length ; i++) {
				if (files[i].getName().contains("utt")) {
					int curIncr = (new Integer(files[i].getName().replace("utt", "").replace(".wav", ""))).intValue();
					if (curIncr > nbexistingfiles) {
						nbexistingfiles = curIncr;
					}
				}
			}
			log("Existing files already in " + wavfilesDir + ", starting increment at " + nbexistingfiles);
		}

		for (int i = nbexistingfiles; i < nbIterations + nbexistingfiles ; i++) {
			log("\t-----------");
			log("\tIteration number:" + (i+1));
			String exampleString = generator.retrieveRandomSentence(cfg);
			TrainingExample example = new TrainingExample(exampleString);

			log("\tInitial utterance: \"" + example.getExpectedUtterance() +"\"");
			if (!example.getExpectedUtterance().equals(example.getExpectedUtterance())) {
				log("\tUtterance + random ASR errors: \"" + example.getExpectedUtterance() +"\"");
			}

			log("\tVerifying semantics using CCG grammar...\t" );
			if (parser.verifyExample(example)) {

				try {
		//			mary = new MaryClient(serverHost, serverPort);
		//			TTSLocal tts = new TTSLocal(mary, voiceName, false, "WAVE");			
		//			tts.saveToFile(example.getExpectedUtterance().wordSequence, wavfilesDir+"utt"+i+".wav");
					log("\t--> Utterance successfully synthesized with MARY");
				}
				catch (Exception e) {
					e.printStackTrace();
				}

				transcriptionsText += wavfilesDir+"utt"+i+".wav " + example.getExpectedUtterance() + "\n";
				log("\t--> Transcription successfully written\n");

			}
			else {
				log("\t--> Example discarded (too long, not parsable, or pruned)\n");
			}
		}

		if (outputTranscriptionsFile != null) {
			log("STEP 4: output example vector to file" + outputTranscriptionsFile);
			GenerationUtils.appendToFile(transcriptionsText, outputTranscriptionsFile);
			log("Output example vector to file:\t\t\t" + bold("OK\n"));
		}
	}


	public void batchLearning() {	

		Vector<TrainingExample> examples = new Vector<TrainingExample>();
		Vector<String> lines = LearningUtils.readFile(genCorpusFile);
		for (Enumeration<String> e = lines.elements(); e.hasMoreElements(); ) {
			String line = e.nextElement();		
			int colonIndex = line.indexOf(":");	
			if (colonIndex >0) {
				String initUtterance = line.substring(0, colonIndex);
				String recogUtterance = line.substring(colonIndex+1, line.length());
				TrainingExample example = new TrainingExample(initUtterance, recogUtterance);
				String semantics = cfg.getSemantics(GenerationUtils.modifyCase2(example.getExpectedUtterance().wordSequence));
				if (semantics.length() == 0 | 
						semantics.contains("@1") | 
						semantics.contains("@2") |
						semantics.contains("@3") |
						semantics.contains("@4") |
						semantics.contains("@5")) {
					log("Warning: no correct semantics found for \"" + example.getExpectedUtterance().wordSequence +"\"");
				}
				else {
					example.setSemantics(semantics);
				}
				examples.add(example);		
			}
		}

		nbGoodExamples = 0;
		int increment = 0;
		for (int k = 0 ; k < nbIterations; k++) {
			for (Enumeration<TrainingExample> e = examples.elements(); e.hasMoreElements(); ) {
				log("\t-----------");
				log("\tExample number:" + (increment+1));
				increment++;
				TrainingExample example = e.nextElement();
				log("\tInitial utterance: \"" + example.getExpectedUtterance().wordSequence +"\"");
				log("\tRecognized utterance: \"" + example.getExpectedUtterance().wordSequence +"\"");
				log("\tVerifying semantics using CCG grammar...\t" );
				if (parser.verifyExample(example)) {	
					PackedLFs plf = parser.getLastParse();
					params = LearningUtils.incrInitPV(example, params, plf);
					params = perceptron.learn(example);
					nbGoodExamples++;
					log("\t--> Example successfully learned\n");
				}
				else {
					log("\t--> Example discarded (too long, not parsable, or pruned)\n");
				}
			}	
		}
	}


	public void storeFinalOutput() {
		System.out.println("");
		log("Number of examples examined: " + bold(nbGoodExamples + "/ " + nbIterations + "*avglen"));
		log("Number of parameter updates: "  + bold(perceptron.countUpdates + "/" + nbGoodExamples));
		log("Learning of parameter values with perceptron:\t\t" + bold("OK\n"));

		if (outputParamFile != null) {
			log("STEP 4: output parameter vector to file" + outputParamFile);
			GenerationUtils.writeFile(params.toString(), outputParamFile);
			log("Output parameter vector to file:\t\t\t" + bold("OK\n"));
		}
		else {
			log("output file not specified, redirecting to standard output:");
			log("Parameter vector values:");
			log(params.toString());
		}	

		if (getStatistics) {
			String statistics = LearningUtils.statisticsToString(perceptron.getStatistics());
			if (outputStatisticsFile != null) {
				log("STEP 5: output statistics to file" + outputStatisticsFile);
				GenerationUtils.appendToFile(statistics, outputStatisticsFile);
				log("Output statistics to file:\t\t\t" + bold("OK\n"));
			}
			else {
				log("output statistics file not specified, redirecting to standard output:");
				log("Perceptron statistics:");
				log(statistics);
			}	
		}
	}


	public static void main(String[] args) {

		System.out.println("");
		log("Starting discriminative learning tool for parse selection...");

		if (args.length == 0) {
			log("FATAL ERROR: no configuration file specified on the command line, aborting!\n");
			System.exit(0);
		}

		Properties properties = LearningUtils.readProperties(args);

		if (properties.get("--learning") != null) {
			DiscriminativeLearner learner = new DiscriminativeLearner(properties);
			learner.startLearning();
		}

		else {
			log("FATAL ERROR: configuration file does not specify learning type, aborting!\n");
		}
	}


	private static void log(String str) {
		if (logging)
			System.out.println("\033[34m[Learner] " + str + "\033[0m");
	}
}
