
package comsys.processing.parse.parseselection;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Random;

import comsys.processing.parse.parseselection.FeatureExtractionFromPLF;
import comsys.processing.parse.parseselection.FeatureVector;
import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;
import comsys.lf.utils.LFPacking;
import comsys.processing.parse.parseselection.ParameterVector;
import comsys.datastructs.comsysEssentials.*;


public class Decoder {

	PackedLFs lastPLF;
	Hashtable<String, FeatureVector> lastFVs;
	double lastMaxScore;
	
	ParameterVector params;
	CCGParser parser;
	
	public boolean extractSemanticFeatures = true;
	public boolean extractSyntacticFeatures = true;
	public boolean extractAcousticFeatures = true;
	public boolean extractContextualFeatures = true;
	public boolean extractNoParseFeature = true;
	
	public String contextdependentwordsFilename = "./subarchitectures/comsys/learning/contextdependentwords.txt";
	public String salientwordsFilename = "./subarchitectures/comsys/learning/salientwords.txt";
	
	
	LFPacking packingTool;
	
	
	public Decoder(ParameterVector params) {
		packingTool = new LFPacking();
		this.params = params;
		parser = new CCGParser();
	}
	
	public void setParser(CCGParser parser) {
		this.parser = parser;
	}
	
	public LogicalForm getBestParse(String utterance) {
		
		PackedLFs plf = new PackedLFs();
		if (parser.getLastParse() != null) {
		 plf = parser.getLastParse();
		}
		else {
			plf.packedLF = parser.parse(utterance);
		}

		return getBestParse(plf);
	}
	
	public String recogError ;
	
	public LogicalForm getBestParse(PackedLFs plf) {
		
		lastPLF = plf;
		
		FeatureExtractionFromPLF extraction = new FeatureExtractionFromPLF(plf);
		extraction.extractSemanticFeatures = extractSemanticFeatures;
		extraction.extractSyntacticFeatures = extractSyntacticFeatures;
		extraction.extractAcousticFeatures = extractAcousticFeatures;
		extraction.extractNoParseFeature = extractNoParseFeature;
		extraction.extractContextualFeatures = extractContextualFeatures;
		extraction.contextdependentwordsFilename = contextdependentwordsFilename;
		extraction.salientwordsFilename = salientwordsFilename;
		
		Hashtable<String, FeatureVector> featureVectors = extraction.extractFeatures();
		lastFVs = featureVectors;
		
		log("Number of alternative interpretations: " + featureVectors.keySet().size());
		
		double maxScore = Double.NEGATIVE_INFINITY;
		String lfIdMaxScore = "";
		for (Enumeration<String> f = featureVectors.keys(); f.hasMoreElements();) {
			
			String lfId = f.nextElement();
			FeatureVector fv = featureVectors.get(lfId);
			double result = params.innerProduct(fv);
		
		//	log("result of inner product: " + result);
			if (result > maxScore) {
				maxScore = result;
				lfIdMaxScore = lfId;
			}
			else if (result == maxScore) {
				Random rand = new Random();
				int choice = rand.nextInt(2);
				if (choice == 1) {
					maxScore = result;
					lfIdMaxScore = lfId;
				}
			}
		}
		
	//	log("final result of inner product: " + maxScore);
		lastMaxScore = maxScore;
		if (lfIdMaxScore.contains("nonparsable-")) {
			LogicalForm fakeLF = new LogicalForm();
			fakeLF.logicalFormId = lfIdMaxScore;
			return fakeLF;		
		}
		
		else {
		LogicalForm bestLf = packingTool.extractLogicalForm(plf.packedLF, lfIdMaxScore);
		showInfoOnSyntacticFeatures(plf, lfIdMaxScore);
		return bestLf;
		}

	}
	
	
	private void showInfoOnSyntacticFeatures(PackedLFs plf, String lfIdMaxScore) {
		recogError = "";
		for (int i = 0; i < plf.nonStandardRulesForLF.length ; i++) {
			NonStandardRulesAppliedForLF nonStandardRules = plf.nonStandardRulesForLF[i];
			if (nonStandardRules.logicalFormId.equals(lfIdMaxScore)) {
				if (nonStandardRules.nonStandardRules.length > 0) {
					String combinators = "";
					for (int j = 0; j < nonStandardRules.nonStandardRules.length; j++) {
						combinators += "[" + nonStandardRules.nonStandardRules[j].rulename + "]";
						if (nonStandardRules.nonStandardRules[j].rulename.contains("recogError")) {
							recogError = "[" + nonStandardRules.nonStandardRules[j].rulename + "]";
						}
						if (j < (nonStandardRules.nonStandardRules.length -1))
							combinators += ", ";
					}
					log("Decoded best parse includes the following non-standard combinator(s): " +combinators);
				}
			}
		}
	}
	
	public double getLastMaxScore() {
		return lastMaxScore;
	}
	
	public LogicalForm getBestParseWithSemantics(String utterance, LogicalForm lf) {
		
		Enumeration<String> lfIds = parser.getSimilarLFs(lastPLF.packedLF, lf);
		
		double maxScore = Double.NEGATIVE_INFINITY; ;
		String lfIdMaxScore = "";
		for (Enumeration<String> f = lfIds; f.hasMoreElements();) {
			
			String lfId = f.nextElement();
			FeatureVector fv = lastFVs.get(lfId);
			
			double result = params.innerProduct(fv);
			
			if (result >= maxScore) {
				maxScore = result;
				lfIdMaxScore = lfId;
			}
		}
			
		if (!lfIdMaxScore.equals("")) {
			LogicalForm bestLf = packingTool.extractLogicalForm(lastPLF.packedLF, lfIdMaxScore);
			return bestLf;
		}
		else {
			log("Warning: expected LF cannot be generated with CCG grammar");
			return null;
		}
	}
	
	public FeatureVector getFV(String lfId) {
		return lastFVs.get(lfId);
	}
	
	
	private static void log(String str) {
		System.out.println("\033[32m[Decoder]\t" + str  + "\033[0m");
	}
}
