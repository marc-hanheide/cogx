package comsys.processing.parseselection;

import java.util.Enumeration;
import java.util.Vector;

import comsys.arch.ComsysException;
import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.comsysEssentials.PhonStringLFPair;
import comsys.datastructs.lf.LogicalForm;
import comsys.datastructs.lf.PackedLogicalForm;
import comsys.lf.utils.LFPacking;
import comsys.lf.utils.LFUtils;
import comsys.processing.parse.ActiveCCGLexicon;
import comsys.processing.parse.ActiveIncrCCGParser;
import comsys.processing.parse.PackedLFParseResults;
import comsys.utils.ComsysUtils;

import opennlp.ccg.grammar.AbstractRule;
import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;

public class CCGParser {

	public boolean logging = true;
	
	public boolean chartScoring = false;
	
	public String defaultGrammar = 
		"subarchitectures/comsys.mk4/grammars/openccg/moloko.v6/grammar.xml";
		
	private PackedLFs lastPLF;
	
	int phonIncr= 0;
	
	ActiveIncrCCGParser parser;
		
	public CCGParser() {
		try{
		parser = new ActiveIncrCCGParser();
		//	parser.setLogLevel(0);
		ActiveCCGLexicon grammar = new ActiveCCGLexicon();
		grammar.setGrammar(defaultGrammar);
		parser.registerGrammarAccess(grammar);

		if (chartScoring) {
			CategoryChartScorer cshs = new CategoryChartScorer();
			cshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			parser.registerChartScorer(cshs);
		}

		FrontierCatFilter cfilter = new FrontierCatFilter();
		parser.registerFrontierFilter(cfilter);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		AbstractRule.disclevelcompositionRules = true;
		AbstractRule.asrcorrectionRules= true;
	}
	
	public void setBeamwidth(int beamwidth, ParameterVector params) {
	if (beamwidth > 0 && params != null) {
		parser.beamwidth = beamwidth;
		parser.params = params;
		parser.incrementalPruning = true;
		}
	}
	
	
	public boolean verifyExample(TrainingExample example) {
		return verifyExample(example, false);
	}
	

	
	public boolean verifyExample(TrainingExample example, boolean incremental) {

		boolean foundEquivalentParse = false;

		try{
			LogicalForm lf = example.getSemantics();

			if (example != null && example.getExpectedUtterance() != null &&
					example.getSemantics() != null &&
					example.getExpectedUtterance().wordSequence.length() > 1 &&  
					example.getExpectedUtterance().wordSequence.length() < 200 && 
					LFUtils.lfToString(example.getSemantics()).length() < 1200 &&
					!example.getExpectedUtterance().wordSequence.contains("right")) {

				log("(full) expected semantics: " + LFUtils.lfToString(example.getSemantics()));
	
				LogicalForm[] lfs2;
				PhonString phon = example.getExpectedUtterance();
				PackedLFParseResults plf = parse(phon, (int)phon.length);
				phonIncr++;
				
				if (plf != null) {
					//	LFUtils.plfToGraph(results.plf, "testPLF");

					LFPacking packingTool = new LFPacking();
					lfs2 = packingTool.unpackPackedLogicalForm(plf.plf);
					log("Number of CCG analyses: " + lfs2.length);
					for (int j=0 ; lfs2 != null && j < lfs2.length && !foundEquivalentParse ; j++) {
						LogicalForm lf2 = lfs2[j];
					//	lf2 = GenerationUtils.removeDuplicateNominalsAndFeatures(lf2);
					//		log("CCG Utterance: " + LFUtils.lfToString(lf2));
						if (LFUtils.isLf1IncludedInLf2(lf2, lf)) {
							foundEquivalentParse = true;
							log("==> The LF associated to the utterance is a valid one");
						}
					}

					if (!foundEquivalentParse) {
						if (lfs2.length > 0) {
						//	log("-------------------");
						//	log("--> Utterance: " + gr.getUtterance());
						//	log("--> Wrong semantics: " + gr.getSemantics());
						}
						log("==>  The LF associated to the utterance is not a valid one according to the CCG grammar");
						//	log("initial semantics: " + initSemantics);
					}
				}
				else {
					log("Unable to parse");
				}
			}
		}

		catch (Exception e) {
			try {
			log("Semantics: " + LFUtils.lfToString(example.getSemantics()));
			}
			catch (Exception f) {
				log("WARNING: problem printing the parse!");
			}
			e.printStackTrace();	
		}

		return foundEquivalentParse;
	}



	public PackedLFParseResults parse(PhonString input, int stringPos) {		

		try {
			PackedLFParseResults results = (PackedLFParseResults)parser.incrParse(input);

			if (results != null && results.plf != null) {
				
				NonStandardRulesAppliedForLF[] nonStandardRulesForLf = 
					ComsysUtils.convertNonStandardRulesDataStructs(results);
				PhonString[] nonParsablePhonStrings;
				if (results.nonParsablePhonStrings != null) {
					nonParsablePhonStrings = new PhonString[results.nonParsablePhonStrings.size()];
					int i = 0;
					for (Enumeration<PhonString> g = results.nonParsablePhonStrings.elements(); g.hasMoreElements();) {
						nonParsablePhonStrings[i] = g.nextElement();
						i++;
					}
				}
				else {
					nonParsablePhonStrings = new PhonString[0];
				}
				
				
				PhonStringLFPair[] pairs = ComsysUtils.convertPhonString2LFPairs(results);
				lastPLF = new PackedLFs("id" , pairs, nonParsablePhonStrings, results.stringPos, results.plf,results.finalized, "interpretation", nonStandardRulesForLf);          	
 				
				/**
				log("number of keys: " + results.nonStandardRulesApplied.size());
				for (Enumeration<String> e = results.nonStandardRulesApplied.keys() ; e.hasMoreElements();) {
					String lfId = e.nextElement();
					Hashtable<String,Integer> hash = results.nonStandardRulesApplied.get(lfId);
					log("number of non-standard rules applied: " + hash.size());
					for (Enumeration<String> f = hash.keys() ; f.hasMoreElements();) {
						String rulename = f.nextElement();
						log ("-- rule " + rulename + " applied " + hash.get(rulename) + " times");
					}
				}*/
				return results;
			}
			else {
				log("Unable to parse");
			}
		}
		catch (ComsysException e) {
			log(e.getMessage());
		}	
		catch (Exception e) {
			e.printStackTrace();
		}	
		return null;
	}
	
	public PackedLFs getLastParse() {
		return lastPLF;
	}

	public Vector<String> getSimilarLFs(PackedLogicalForm plf, LogicalForm lf) {

		LFPacking packingTool = new LFPacking();
		LogicalForm[] lfs = packingTool.unpackPackedLogicalForm(plf);
		
		Vector<String> correctLfIds = new Vector<String>();
		
		for (int i=0 ; i < lfs.length ; i++) {
			if (LFUtils.compareLFs(lfs[i], lf)) {
				correctLfIds.add(lfs[i].logicalFormId);
			}
		}
		
		return correctLfIds;
	}
	
	private void log(String str) {
		if (logging)
		System.out.println("\033[33m[Parser]\t" + str+ "\033[0m");
	}
	
	public static void main (String[] args) {
		String utterance = "what size is the laptop now";
		String semantics = "(@q10:ascription(be ^ <Mood>int ^ <Tense>pres ^  <Cop-Restr>(o14:thing ^ laptop ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific) ^  <Cop-Scope>(c1_0:quality ^ size) ^  <Subject>(o14:thing ^ laptop ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific) ^  <Wh-Restr>(w1_0:specifier ^ what ^  <Scope>c1_0:quality) ^  <Modifier>(t1:m-time-point ^ now))";
		TrainingExample example = new TrainingExample (utterance + ": " + semantics);
		
		for (int i=0; i < 100 ; i++) {
		CCGParser parser = new CCGParser();
		System.out.println("final result: " + parser.verifyExample(example));
		}
	}

}
