package comsys.processing.parse.parseselection;

import java.util.Enumeration;
import java.util.Vector;

import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;

import comsys.datastructs.comsysEssentials.*;
import comsys.processing.parse.PackedLFParseResults;
import comsys.arch.ComsysException;
import comsys.utils.ComsysUtils;
import comsys.processing.parse.ActiveCCGLexicon;
import comsys.processing.parse.ActiveIncrCCGParser;
import comsys.datastructs.lf.*;
import comsys.lf.utils.LFPacking;
import comsys.lf.utils.LFUtils;
import java.util.Hashtable;

public class CCGParser {

	public boolean logging = true;
	
	public boolean chartScoring = false;
	
	public String defaultGrammar = 
		"subarchitectures/comsys.mk4/grammars/openccg/robustmoloko/grammar.xml";
		
	private PackedLFs lastPLF;
	

	public boolean verifyExample(TrainingExample example) {

		boolean foundEquivalentParse = false;

		try{
			LogicalForm lf = example.getSemantics();

			if (example != null && example.getUtterance() != null &&
					example.getSemantics() != null &&
					example.getUtterance().length() > 1 &&  
					example.getUtterance().length() < 90 && 
					LFUtils.lfToString(example.getSemantics()).length() < 600 &&
					!example.getUtterance().contains("right")) {

		//		log("Utterance: " + example.getUtterance());
				log("Expected semantics: " + LFUtils.lfToString(example.getSemantics()));
	
				LogicalForm[] lfs2;
				PackedLogicalForm plf = parse(example.getRecognizedUtterance());
				
				if (plf != null) {
					//	LFUtils.plfToGraph(results.plf, "testPLF");

					LFPacking packingTool = new LFPacking();
					lfs2 = packingTool.unpackPackedLogicalForm(plf);
					log("Number of CCG analyses: " + lfs2.length);
					for (int j=0 ; lfs2 != null && j < lfs2.length && !foundEquivalentParse ; j++) {
						LogicalForm lf2 = lfs2[j];
					//	lf2 = GenerationUtils.removeDuplicateNominalsAndFeatures(lf2);
					//		log("CCG Utterance: " + LFUtils.lfToString(lf2));
						if (LFUtils.compareLFs(lf, lf2)) {
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



	public PackedLogicalForm parse(String input) {		

		try {
			ActiveIncrCCGParser parser = new ActiveIncrCCGParser();
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

			PhonString phon = new PhonString();
			phon.wordSequence = input ;
			phon.id ="bla";

			PackedLFParseResults results = (PackedLFParseResults)parser.parse(phon);

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
				else 
					nonParsablePhonStrings = new PhonString[0];

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
				return results.plf;
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

	public Enumeration<String> getSimilarLFs(PackedLogicalForm plf, LogicalForm lf) {

		LFPacking packingTool = new LFPacking();
		LogicalForm[] lfs = packingTool.unpackPackedLogicalForm(plf);
		
		Vector<String> correctLfIds = new Vector<String>();
		
		for (int i=0 ; i < lfs.length ; i++) {
			if (LFUtils.compareLFs(lfs[i], lf)) {
				correctLfIds.add(lfs[i].logicalFormId);
			}
		}
		
		return correctLfIds.elements();
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
