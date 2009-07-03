
package comsys.processing.parseselection;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Random;

import java.util.Vector;
import java.util.Arrays;

import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.lf.LogicalForm;
import comsys.datastructs.lf.PackedLogicalForm;
import comsys.lf.utils.LFPacking;
import comsys.lf.utils.LFUtils;

public class Decoder {

	public boolean logging = true;
	
	PackedLFs lastPLF;
	Hashtable<String, FeatureVector> lastFVs;
	double lastMaxScore;
	double last2ndMaxScore;
	
	public ParameterVector params;
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
//		parser = new CCGParser();
	}

	public void setParser(CCGParser parser) {
		this.parser = parser;
	} 

	public LogicalForm getBestParse(PhonString utterance, int stringPos) {

		PackedLFs plf = new PackedLFs();
		if (parser.getLastParse() != null) {
			plf = parser.getLastParse();
		}
		else {
			plf.packedLF = parser.parse(utterance, stringPos).plf;
		}

		return getBestParse(plf);
	}

	public String recogError ;


	public LogicalForm getBestParse(PackedLFs plf) {
				
		String LFId = getBestParses(plf, 1).elementAt(0);
				
		if (LFId.contains("nonparsable-")) {
			LogicalForm fakeLF = new LogicalForm();
			fakeLF.logicalFormId = LFId;
			return fakeLF;
		}
		else {
			LogicalForm bestLf = packingTool.extractLogicalForm
			(plf.packedLF, LFId);
			showInfoOnSyntacticFeatures(plf, LFId);
			return bestLf;
		}
	}

	public Vector<String> getWorstParses(PackedLFs plf, int beamwidth) {
		
		Vector<String> worstParsesIds = new Vector<String>();
		
		String[] LFs = LFUtils.plfGetPackingNode(plf.packedLF,plf.packedLF.root).lfIds;
		Vector<String> selectedLFs = getBestParses(plf, beamwidth);

		for (int i = 0; i < LFs.length ; i++) {
			String lfId = LFs[i];
			if (!selectedLFs.contains(lfId)) {
				worstParsesIds.add(lfId);
			}
		}
		return worstParsesIds;
	}
	
	
	public Vector<String> getBestParses(PackedLFs plf, int beamwidth) {

		Vector<String> results = new Vector<String>();
	
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

		int incr = 0;
		DecodingResult[] dresults = new DecodingResult[featureVectors.size()];

		for (Enumeration<String> f = featureVectors.keys(); f.hasMoreElements();) {

			String lfId = f.nextElement();
			FeatureVector fv = featureVectors.get(lfId);
			double result = params.innerProduct(fv);
			DecodingResult dresult = new DecodingResult (result, lfId);
			dresults[incr] = dresult;
			incr++;
		}

		Arrays.sort(dresults);

		//	log("final result of inner product: " + maxScore);

		for (int j = 0 ; j < beamwidth && j <= dresults.length - 1 ; j++) {

			DecodingResult dresult = dresults[dresults.length-1-j];
			String lfIdMaxScore = dresult.getLfId();
			results.add(lfIdMaxScore);
		}
		
		if (dresults.length > 0)
		lastMaxScore = dresults[0].getScore();
		if (dresults.length > 1)
			last2ndMaxScore = dresults[1].getScore();

		return results;
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

	
	public double getLast2ndMaxScore() {
		return last2ndMaxScore;
	}
	
	public LogicalForm getBestParseWithIncludedSemantics(PhonString utterance, LogicalForm expectedFullLF) {

		LFPacking packingTool = new LFPacking();
		
		PackedLogicalForm plf = lastPLF.packedLF;

		double maxScore = Double.NEGATIVE_INFINITY; ;
		String lfIdMaxScore = "";
		log("Number of logical forms to consider: " + lastFVs.keySet().size());
		for (String lfId : lastFVs.keySet()) {

			LogicalForm lf = packingTool.extractLogicalForm(plf, lfId);
			
			if (LFUtils.isLf1IncludedInLf2(lf, expectedFullLF)) {
				FeatureVector fv = lastFVs.get(lfId);
			
				double result = params.innerProduct(fv);

				if (result >= maxScore) {
					maxScore = result;
					lfIdMaxScore = lfId;
				}
			}
		}

		if (!lfIdMaxScore.equals("")) {
			LogicalForm bestLf = packingTool.extractLogicalForm(lastPLF.packedLF, lfIdMaxScore);
			return bestLf;
		}
		else {
			log("Warning: A golden standard partial parse cannot be found");
			return null;
		}
	}

	public FeatureVector getFV(String lfId) {
		return lastFVs.get(lfId);
	}


	private void log(String str) {
		if (logging)
			System.out.println("\033[32m[Decoder]\t" + str  + "\033[0m");
	}

	public final class DecodingResult implements Comparable {

		double score;
		String lfId;

		public DecodingResult(double score, String lfId) {
			this.score = score;
			this.lfId = lfId;
		}

		public int compareTo(Object result) {
			DecodingResult result2 = (DecodingResult) result;
			return (int)(score-result2.getScore());
		}

		public double getScore() {
			return score;
		}

		public String getLfId() {
			return lfId;
		}
	}
}
