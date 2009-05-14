package org.cognitivesystems.comsys.general.testers;

import java.util.Properties;
import java.util.Vector;

import opennlp.ccg.grammar.AbstractRule;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.components.ParseSelection;
import org.cognitivesystems.comsys.data.testData.ParseSelectionTestData;
import org.cognitivesystems.comsys.data.testData.TestData;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;
import org.cognitivesystems.comsys.processing.examplegeneration.SemanticCorpusGenerator;
import org.cognitivesystems.comsys.processing.parseselection.AveragedPerceptron;
import org.cognitivesystems.comsys.processing.parseselection.CCGParser;
import org.cognitivesystems.comsys.processing.parseselection.Decoder;
import org.cognitivesystems.comsys.processing.parseselection.LearningUtils;
import org.cognitivesystems.comsys.processing.parseselection.ParameterVector;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFRelation;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.core.data.CASTData;

abstract public class AbstractParseSelectionTester extends AbstractComsysTester {

	Vector<TestData> parseSelectionTests;
	String testFile = "tests/data/utterancesToSelect.xml";
	String grammarFile = "subarchitectures/comsys.mk4/grammars/openccg/robustmoloko/grammar.xml";
	String WERFileName = "WER-data.txt";
	String AccuracyFileName = "accuracy.txt";

	int NbNBests = 5;
	boolean discardItemsWithoutCorrectParse = false;

	int maxUtteranceLength = 10;
	
	boolean useFakeSaliency = false;
	
	static int count = 1;
	
	protected ParameterVector params;
	protected Decoder decoder;
	
	public int beamwidth = 2;

	/**
	 * @param _id
	 */
	public AbstractParseSelectionTester(String _id) {
		super(_id);
	}



	abstract public class AbstractParseSelectionTest extends AbstractComsysTest  {

		ParseSelectionTestData test ;

		public AbstractParseSelectionTest(TestData test) {
			this.test = (ParseSelectionTestData) test;
		}

		public void appendWERResultsToFile(String recogString) {

			String line = "\"" + test.getTranscription() + "\" --> \"" + 
			recogString + "\"\n";
			GenerationUtils.appendToFile(line, WERFileName);
		}
		
		public void appendAccuracyResultsToFile(boolean hadCorrectParse, boolean parsed, boolean totalmatch, double score, int matchCount, int totalCount) {

			String scoreStr = new Double(score).toString();
			if (scoreStr.length() >5)
				scoreStr = scoreStr.substring(0,5);

			String emResult = "";
			String pmResult = "";
			if (hadCorrectParse) {
				if (totalmatch) {
					emResult = "TP (parsed, 100% match)";
				}
				else {
					if (parsed) 
						emResult = "FP (parsed, not exact match)";
					else
						emResult = "FN (no parse found)";
				}
				if (parsed)
				pmResult = " ("+matchCount+ " TP & " + (totalCount-matchCount) + " FP)";
				else
					pmResult = " ("+matchCount+ " TN & " + (totalCount-matchCount) + " FN)";
			}
			else {
				emResult = "TN (no reference parse given)";
			}


			String line = "Test " + count + "--> exact-match result: " + emResult + 
			", score: " + scoreStr + 
			", partial-match result: "+matchCount+"/"+totalCount + pmResult + "\n";
			GenerationUtils.appendToFile(line, AccuracyFileName);
			count++;
		}


		public boolean compareResults(WorkingMemoryChange _wmc, String expectedLF) {
			try {
				log("got added LF");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData lfWM = new CASTData(id,
						getWorkingMemoryEntry(id));

				LogicalForm lf1 = (LogicalForm) lfWM.getData();
				//	log("LF: " + LFUtils.lfToString(lf1));

				LogicalForm lf2 = LFUtils.convertFromString(expectedLF);
				lf2 = GenerationUtils.removeDuplicateNominalsAndFeatures(lf2);
				return compareResults(lf1, lf2);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
			return true;
		}


		public boolean compareResults (LogicalForm lf1, LogicalForm lf2) {
			if (lf1 != null && lf2 != null) {
				boolean comparison = LFUtils.compareLFs(lf1, lf2) && LFUtils.compareLFs(lf2, lf1);
				log("\tComparison result: " + bold(""+ comparison));
				if(!comparison) {
					log("\tCorrect parse: \n\t\t" + specialfont(LFUtils.lfToString(lf2)));
				}
				return comparison;
			}		
			return true;
		}
	}

	/**
	 * Verify the equivalence of two logical forms
	 */
	public int partialMatch (LogicalForm lf1, LogicalForm lf2) {

		int matchCount = 0;

		for (int i = 0 ; i < lf1.noms.length ; i++) {
			boolean foundEquiv = false;
			LFNominal nom1 = lf1.noms[i];
			for (int j = 0 ; j < lf2.noms.length && !foundEquiv; j++) {
				LFNominal nom2 = lf2.noms[j];
				if (!foundEquiv && nom1.prop.prop.equals(nom2.prop.prop) &&
						nom1.sort.equals(nom2.sort)) {

					boolean isExactMatch = true;
					for (int k = 0 ; k < nom1.feats.length ; k++) {
						boolean foundEquivFeat = false;
						Feature feat1 = nom1.feats[k];
						for (int l = 0 ; l < nom2.feats.length && !foundEquivFeat ; l++) {
							Feature feat2 = nom2.feats[l];
							if (feat1.feat.equals(feat2.feat) && 
									feat1.value.equals(feat2.value)) {
								foundEquivFeat = true;
							}
						}

						if (foundEquivFeat) {
							matchCount++;
				//			log("equiv found for : " + feat1.feat + ":" + feat1.value);
						}
						else {
				//			log("feat: " + feat1.feat + "-" + feat1.value + " not found");
							isExactMatch = false;
						}
					}

					for (int k = 0 ; k < nom1.rels.length ; k++) {
						boolean foundEquivRel = false;
						LFRelation rel1 = nom1.rels[k];
						for (int l = 0 ; l < nom2.rels.length && !foundEquivRel ; l++) {
							LFRelation rel2 = nom2.rels[l];
							if (rel1.mode.equals(rel2.mode)) {
								foundEquivRel = true;
							}
						}

						if (foundEquivRel) {
							matchCount++;
				//			log("equiv found for : " + rel1.head + "->" + rel1.dep + "[" + rel1.mode + "]");
						}
						else {
				//			log("rel: " + rel1.mode + " not found");
							isExactMatch = false;
						}
					}
					if (isExactMatch)
						foundEquiv = true;
					matchCount++;
			//		log("equiv found for : " + nom1.nomVar + ":" + nom1.sort + " ^ " + nom1.prop.prop);
				}
			}
		}		

		return matchCount;

	}
	
	
	/**
	 * 
	 */
	public int countSubstructures (LogicalForm lf1) {

		int totalCount = 0;

		for (int i = 0 ; i < lf1.noms.length ; i++) {
			LFNominal nom1 = lf1.noms[i];
			totalCount++;

					for (int k = 0 ; k < nom1.feats.length ; k++) {
							totalCount++;
					}

					for (int k = 0 ; k < nom1.rels.length ; k++) {
						totalCount++;

					}
				}

		return totalCount;
	}


	protected void initialization(Properties properties) {

		if (properties.get("--inputParamFile") != null) { 
			String inputParamFile  = properties.getProperty("--inputParamFile");
			log("- Extracting existing parameter values from input file " + inputParamFile);
			Vector<String> lines = LearningUtils.readFile(inputParamFile);
			params = LearningUtils.buildParameterVector(lines);
			log("  Extraction successful, number of features: " + bold(""+params.size()));
			decoder = new Decoder(params);
		}
		
		else {
			log("ERROR, must specify a parameter vector");
			System.exit(0);
		}

		if (properties.get("--CCGGrammarFile") != null) {
			grammarFile = (String)properties.get("--CCGGrammarFile");
			log("- CCG grammar filename: " +  bold(grammarFile));
		}
		
		if (properties.get("--testFile") != null) {
			testFile = (String)properties.get("--testFile");
			log("- Parse selection tests filename: " +  bold(testFile));
		}

		if (properties.get("--useFakeSaliency") != null) {
			useFakeSaliency = (new Boolean((String)properties.get("--useFakeSaliency"))).booleanValue();
		}
		log("- Use fake saliency: " + bold(""+useFakeSaliency));

		if (useFakeSaliency && properties.get("--contextdependentwordsFilename") != null) {
			decoder.contextdependentwordsFilename = (String)properties.get("--contextdependentwordsFilename");
		}
		log("- Context-dependent words filename: " + bold(decoder.contextdependentwordsFilename));
		
		if (useFakeSaliency && properties.get("--salientwordsFilename") != null) {
			decoder.salientwordsFilename = (String)properties.get("--salientwordsFilename");
		}
		log("- Salient words filename: " +  bold(decoder.salientwordsFilename));

		log("");

		if (properties.get("--extractSemanticFeatures") != null) {
			decoder.extractSemanticFeatures = (new Boolean((String)properties.get("--extractSemanticFeatures"))).booleanValue();
		}
		log("- Extraction of semantic features: " + bold(""+decoder.extractSemanticFeatures));

		if (properties.get("--extractSyntacticFeatures") != null) {
			decoder.extractSyntacticFeatures = (new Boolean((String)properties.get("--extractSyntacticFeatures"))).booleanValue();
		}
		log("- Extraction of syntactic features: " + bold(""+decoder.extractSyntacticFeatures));

		if (properties.get("--extractAcousticFeatures") != null) {
			decoder.extractAcousticFeatures = (new Boolean((String)properties.get("--extractAcousticFeatures"))).booleanValue();
		}
		log("- Extraction of acoustic features: " + bold(""+decoder.extractAcousticFeatures));

		if (properties.get("--extractContextualFeatures") != null) {
			decoder.extractContextualFeatures = (new Boolean((String)properties.get("--extractContextualFeatures"))).booleanValue();
		}
		log("- Extraction of contextual features: " + bold(""+decoder.extractContextualFeatures));

		if (properties.get("--extractNoParseFeature") != null) {
			decoder.extractNoParseFeature = (new Boolean((String)properties.get("--extractNoParseFeature"))).booleanValue();
		}
		log("- Extraction of \"no parse\" feature: " + bold(""+decoder.extractNoParseFeature));	
			
		log("");
		
		if (properties.get("--maxUtteranceLength") != null) {
			maxUtteranceLength = new Integer((String)properties.get("--maxUtteranceLength")).intValue();
		}
		log("- Max utterance length to consider all NBest hypotheses: " + bold(""+maxUtteranceLength));
		
		if (properties.get("--beamwidth") != null) {
			beamwidth = new Integer((String)properties.get("--beamwidth")).intValue();
		}
		log("- Beam width for incremental chart scoring: " + bold(""+beamwidth));

		if (properties.get("--NbNBests") != null) {
			NbNBests = new Integer((String)properties.get("--NbNBests")).intValue();
		}
		log("- Number of NBest hypotheses to consider: " + bold(""+NbNBests));

		if (properties.get("--discardItemsWithoutCorrectParse") != null) {
			discardItemsWithoutCorrectParse = (new Boolean((String)properties.get("--discardItemsWithoutCorrectParse"))).booleanValue();
		}
		log("- Discard test items without any specified correct parse: " + bold(""+discardItemsWithoutCorrectParse));

		if (properties.get("--WERFileName") != null) {
			WERFileName = (String)properties.get("--WERFileName");
		}
		log("- Word Error Rate results written to file: " + bold(""+WERFileName));
			
		if (properties.get("--AccuracyFileName") != null) {
			AccuracyFileName = (String)properties.get("--AccuracyFileName");
		}
		
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
		
		log("- Accuracy (exact-match, partial-match) results written to file: " + bold(""+AccuracyFileName));
	
		log("");
	}


	public static String specialfont(String str) {
		return "\033[2;34m"+ str + "\033[0;34m";
	}
	
	public static String bold(String str) {
		return "\033[1;34m"+ str + "\033[0;34m";
	}

	public static void log(String str) {
		System.out.println("\033[34m[Tester] " + str + "\033[0m");
	}
}
