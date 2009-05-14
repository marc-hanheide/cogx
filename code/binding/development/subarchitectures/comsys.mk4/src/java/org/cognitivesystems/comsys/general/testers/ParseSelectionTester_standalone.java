package org.cognitivesystems.comsys.general.testers;

import java.util.Properties;
import java.util.Vector;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.components.Controler;
import org.cognitivesystems.comsys.components.UtteranceInterpretation;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.data.SelectedLogicalForm;
import org.cognitivesystems.comsys.data.WordRecognitionLattice;
import org.cognitivesystems.comsys.data.testData.TestData;
import org.cognitivesystems.comsys.general.ComsysUtils;
import org.cognitivesystems.comsys.general.testers.AbstractParseSelectionTester.AbstractParseSelectionTest;
import org.cognitivesystems.comsys.general.testers.ParseSelectionTester.ParseSelectionTest;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;
import org.cognitivesystems.comsys.processing.parseselection.Decoder;
import org.cognitivesystems.comsys.processing.parseselection.DiscriminativeLearner;
import org.cognitivesystems.comsys.processing.parseselection.LearningUtils;
import org.cognitivesystems.comsys.processing.parseselection.ParameterVector;
import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.core.data.CASTData;

public class ParseSelectionTester_standalone extends AbstractParseSelectionTester {

	UtteranceInterpretation inter;
	Controler controler;
	
	public ParseSelectionTester_standalone(Properties properties) {
		super("parseselectiontester");

		log("Initialization...");
		initialization(properties);
		inter = new UtteranceInterpretation("inter");
		inter.grammarFile = grammarFile;
		inter.params = params;
		inter.beamwidth = beamwidth;
		inter.init();
		inter.setDecoder(decoder);
		
		controler = new Controler("bla");
		
		GenerationUtils.logging = false;
		
		XMLTestReader reader = new XMLTestReader();
		reader.NbNBests = NbNBests;
		parseSelectionTests = reader.readTest(testFile);
		log("Extraction of XML test file successful\n");
		log("---------------");
	}
	
	public void startTests() {
		long initTime = System.currentTimeMillis();
		

		inter.parser.maxUtteranceLength = maxUtteranceLength;
		
		
		for (int i = 0 ; parseSelectionTests != null && i < parseSelectionTests.size(); i++) {
			ParseSelectionTest_standalone test = 
				new ParseSelectionTest_standalone(parseSelectionTests.elementAt(i));
			log("Test: "+(i+1)+":");
			test.startTestDirect();
		
			log("----------------------------------------------");
		}
		long finalTime = System.currentTimeMillis();
		log("==> Total processing time (in seconds): " + (finalTime-initTime)/1000.0);
	}
	
	public class ParseSelectionTest_standalone 
	extends AbstractParseSelectionTest  {
	
	public String recogString;
	public double lastscore = 0.0;
	
	public ParseSelectionTest_standalone(TestData test) {
		super(test);
	}
	
	public void startTest() {}
	
	protected void startTestDirect() {

		if (!discardItemsWithoutCorrectParse || 
				!test.getCorrectParse().equals("")) {
			
		log("Initial string: " + test.getTranscription());
		
		if (test.getCorrectParse().equals(""))
			log("No correct parse is given for the utterance");
		
		WordRecognitionLattice lattice = new WordRecognitionLattice(test.getASRInputs(), "id1");
		CASTData<WordRecognitionLattice> data = 
			new CASTData<WordRecognitionLattice>("idw", "org::cognitivesystems::comsys::WordRecognitionLattice", lattice);
		
		if (lattice.getMaximumLength() > 0) {
			try {
				
				long initTime = System.currentTimeMillis();
				
				//	addToWorkingMemory(id, lattice);
				ProcessingData pd = new ProcessingData("blabla");
				pd.add(data);

				inter.executeParseTask(pd);
				PackedLFs plf = inter.lastAddedPLf;
				
				decoder.logging = true;
				LogicalForm lf = decoder.getBestParse(plf);
				decoder.logging = false;
				
				long finalTime = System.currentTimeMillis();
				log("\tTotal interpretation time (in seconds): " + bold(""+(finalTime-initTime)/1000.0));

				boolean parsed = lf.root != null;			
				String bestLFStr;
				
				if (parsed) {
				bestLFStr = LFUtils.lfToString(lf);
				}
				else {
					bestLFStr = "(not parsable)";
				}

				log("\tBest logical form according to current model: \n\t\t" + specialfont(bestLFStr));
				if (bestLFStr == null) {
					recogString = test.getASRInputs().elementAt(0).wordSequence;
					appendWERResultsToFile(recogString);
				}
				else {
					String scoreStr = new Double(decoder.getLastMaxScore()).toString();
					if (scoreStr.length() >10)
						scoreStr = scoreStr.substring(0,10);
					log("\tDecoding score: " + scoreStr);

					recogString = ComsysUtils.getPhonStringFromPair(plf, lf.logicalFormId).wordSequence;
					log("\tPhonological string: \"" + recogString);
					recogString += decoder.recogError;
					decoder.recogError = "";
					lastscore = decoder.getLastMaxScore();
					appendWERResultsToFile(recogString);
					String correctparse = test.getCorrectParse();
					
					boolean hadReferenceParse = !correctparse.equals("");

					boolean comparison = false;
					int partialMatchCount = 0;
					int totalCount = 0;
		
					if (hadReferenceParse) {
						LogicalForm lf2 = LFUtils.convertFromString(correctparse);
						lf2 = GenerationUtils.removeDuplicateNominalsAndFeatures(lf2);
						totalCount = countSubstructures(lf2);
	                	
						if (parsed) {
							lf = GenerationUtils.removeDuplicateNominalsAndFeatures(lf);
							comparison = compareResults(lf, lf2);
							partialMatchCount = partialMatch(lf2, lf);
							if (partialMatchCount > totalCount) 
								partialMatchCount = totalCount - partialMatchCount;
							float pmatch = ((partialMatchCount+0.0f)/totalCount);
							log("\tPartial match: " +  bold(""+ partialMatchCount + "/" + totalCount) + " ==> " + pmatch);
					
		                	PhonString phon = ComsysUtils.getPhonStringFromPair(plf, lf.logicalFormId);
		                	SelectedLogicalForm selectedLF = new SelectedLogicalForm();
		                	selectedLF.lf = lf;
		                	selectedLF.score = lastscore;
		                	selectedLF.phon = phon;
		                	selectedLF.scoreOfSecondBest = decoder.getLast2ndMaxScore();
		                	selectedLF.exactMath = comparison;
		                	selectedLF.partialMatch = pmatch;
		          //      	controler.verifySelectedLF(selectedLF);
		                	
						}
					}

					appendAccuracyResultsToFile(hadReferenceParse, parsed, comparison, lastscore, 
							partialMatchCount, totalCount);
				}
				
			}
			catch(Exception e) {
				e.printStackTrace();
			}
		}
		else {
			appendWERResultsToFile("");
			testComplete(true);
		}      
		//	initParseSelectionTest(test.getASRInputs());
	}
	}
	}

	
	public static void main (String[] args) {

		System.out.println("");
		log("Starting testing tool for parse selection...");
		
		Properties properties = LearningUtils.readProperties(args);
		
		if (properties.size() > 0) {
			ParseSelectionTester_standalone tester = 
				new ParseSelectionTester_standalone(properties);
			tester.startTests();
		}
		else {
			log("ERROR: must specify configuration file");
			log("Usage: --configFile blablafile\n");
			System.exit(0);
		}
		
		System.out.println("");
	}

}
