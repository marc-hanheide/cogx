package org.cognitivesystems.comsys.processing;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Stack;
import java.util.Vector;

import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.data.PackedLFParseResults;
import org.cognitivesystems.comsys.data.WordRecognitionLattice;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFRelation;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;

public class ActiveWordLatticeParser extends ActiveIncrCCGParser {


	public int maxUtteranceLength = 10;

	Hashtable<String,PackedLFParseResults> alreadyFinishedParses ;

	int stringPos = 0;
	

	/**
	 * Complete parsing for word recognition lattices
	 */
	public ParseResults parse (WordRecognitionLattice lattice) 
	throws ComsysException, ParseException {
	//	logging = true;
		log("parsing word lattice...");
		//	setLogLevel(1);
		// initialize return result
		PackedLFParseResults results = new PackedLFParseResults();
		// tokenize the phonString
		// Run incremental parse steps over the length of the sentence

		alreadyFinishedParses = new Hashtable<String,PackedLFParseResults>();

		for (int i = 0; i < lattice.getMaximumLength() ; i++) { 
			log("Incremental parse, step " + (i+1));
			results = (PackedLFParseResults) this.incrParse(lattice);
			recordAmbiguity(results);
		} // end for over the length of the sentence
		
		stringPos = 0;

		chartHistories = new Hashtable<String,Stack>();
		phonStringPositions = new Hashtable<String,Integer>();
		removedSigns = new Hashtable<String,Vector<PackedLFParseResults.SignInChart>>();
		
		return (ParseResults)results;
	} // end parse


	private void recordAmbiguity(PackedLFParseResults results) {
		
		int stringPos = results.stringPos;
		int nbLFs = LFUtils.plfGetPackingNode(results.plf, results.plf.root).lfIds.length;
		
		String text = stringPos + ": " + nbLFs + "\n";
		
		GenerationUtils.appendToFile(text, "data/ambiguities.txt");
	}
	

	
	/**
	 * Incremental parsing for word recognition lattices
	 */
	public ParseResults incrParse(WordRecognitionLattice lattice) 
	throws ComsysException, ParseException{

		log("-----------------------------");
	
		PackedLFParseResults results = new PackedLFParseResults();
		results.finalized = this.NOTFINISHED;

		Vector<PhonString> recogResults;
		
	//	log(lattice.getMaximumLength());
		if (lattice.getMaximumLength() <= maxUtteranceLength) {
			recogResults = lattice.getAllResults();
		}
		else {
			recogResults = lattice.getFirstResult();
		} 

		Vector<LogicalForm> totalLFs = new Vector<LogicalForm>();
		Vector<String> totalLFsStrs = new Vector<String>();
		
		boolean areParsesFinished = true;

		int incrStr = 1;
		String uniformNumbering = "";
		Hashtable<String,Hashtable<String,Integer>> nonStandardRules = 
			new Hashtable<String,Hashtable<String,Integer>>();

		results.phon2LFsMapping = new Hashtable<PhonString,Vector<String>>();
		results.nonParsablePhonStrings = new Vector<PhonString>();
		
		incrementalPruning = false;
		results.lfIdToSignMapping = 
			new Hashtable<String,PackedLFParseResults.SignInChart>();
		
		for (Enumeration<PhonString> e = recogResults.elements() ; e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			
			log(">>>>> Parsing path " + incrStr + ": \"" + phon.wordSequence + "\"");
			
			Vector<String> LFsForPhon = new Vector<String>();
			results.phon2LFsMapping.put(phon, LFsForPhon);

			try {
				PackedLFParseResults result;

				result = (PackedLFParseResults) incrParse(phon);

				if (result != null && result.plf != null) {
					
					LFPacking packingTool2 = new LFPacking();
					
					LogicalForm[] unpackedLFs = packingTool2.unpackPackedLogicalForm(result.plf);
		
					for (int i = 0; i < unpackedLFs.length; i++) {		
					
						String newLfId = unpackedLFs[i].logicalFormId + "-" + incrStr;
												
						PackedLFParseResults.SignInChart sign = 
							result.lfIdToSignMapping.get(unpackedLFs[i].logicalFormId);
						results.lfIdToSignMapping.put(newLfId, sign);

						unpackedLFs[i].logicalFormId = newLfId;

						for (int j = 0; j < unpackedLFs[i].noms.length ; j++) {
							LFNominal nom = unpackedLFs[i].noms[j];
							String[] split = nom.nomVar.split("_");
							if (split.length == 2) {
								if (uniformNumbering.equals("")) {
									uniformNumbering = split[1];
								}
								else {
									nom.nomVar = split[0] + "_" + uniformNumbering;
								}
							}
							for (int k = 0; k < nom.rels.length; k++) {
								LFRelation rel = nom.rels[k];
								String[] split2 = rel.head.split("_");
								rel.head = split2[0] + "_" + uniformNumbering;
								String[] split3 = rel.dep.split("_");
								rel.dep = split3[0] + "_" + uniformNumbering;
							}
						}
					}

					Vector<LogicalForm> lfs = LFArrayToVector(unpackedLFs);
					
					for (LogicalForm lf : lfs) {
						String lfStr = LFUtils.lfToString(lf); 
						if (!totalLFsStrs.contains(lfStr)) {
							totalLFs.add(lf);
							totalLFsStrs.add(lfStr);
							LFsForPhon.add(lf.logicalFormId);
							String oldLfId = lf.logicalFormId.replace("-"+incrStr, "");
							if (result.nonStandardRulesApplied.containsKey(oldLfId)) {
								nonStandardRules.put((lf.logicalFormId), 
										result.nonStandardRulesApplied.get(oldLfId));
							}
						}
					}

					if (result.finalized == this.FINAL_PARSE) {
						alreadyFinishedParses.put(phon.id, result);
					}
					else {
						areParsesFinished = false;
					}
					incrStr++;
				}
				
				// phonological string is not parsable
				else {
					results.nonParsablePhonStrings.add(phon);
				}
			}
			catch (ParseException f) {
				log(f.getMessage());
			}
		}

		LogicalForm[] totalLFsArray = new LogicalForm[totalLFs.size()];
		totalLFsArray = totalLFs.toArray(totalLFsArray);

		log("now packing " + totalLFsArray.length + " logical forms for the whole lattice");

		PackedLogicalForm plf = packingTool.packLogicalForms(totalLFsArray);
		
		log("packing done");
			
		results.plf = plf;
		results.nonStandardRulesApplied = nonStandardRules;
		results.plf.packedLFId = "plf"+ utteranceIncrement;
		results.finalized = this.FINAL_PARSE;
		utteranceIncrement++;

		results.stringPos = stringPos;
		stringPos++;
		
		pruneChartAnalyses(results);
		
		log("Total number of logical forms for lattice: " + totalLFsArray.length);

		return results;
	}

	private Vector<LogicalForm> LFArrayToVector(LogicalForm[] lfs) {
		Vector<LogicalForm> result = new Vector<LogicalForm>();
		for (int i = 0; i < lfs.length ; i++) {
			result.add(lfs[i]);
		}		
		return result;
	}
}
