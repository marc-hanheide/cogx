package comsys.processing.parse;

import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;

import comsys.datastructs.comsysEssentials.PhonString;
import comsys.processing.parse.PackedLFParseResults;
import comsys.processing.parse.ActiveIncrCCGParser;
import comsys.processing.asr.WordRecognitionLattice;
import comsys.arch.ComsysException;
import comsys.datastructs.lf.*;
import comsys.lf.utils.LFPacking;

public class ActiveWordLatticeParser extends ActiveIncrCCGParser {


	public int maxUtteranceLength = 10;

	Hashtable<String,PackedLFParseResults> alreadyFinishedParses ;


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
		} // end for over the length of the sentence
		return (ParseResults)results;
	} // end parse


	
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
		boolean areParsesFinished = true;

		int incrStr = 1;
		String uniformNumbering = "";
		Hashtable<String,Hashtable<String,Integer>> nonStandardRules = 
			new Hashtable<String,Hashtable<String,Integer>>();

		results.phon2LFsMapping = new Hashtable<PhonString,Vector<String>>();
		results.nonParsablePhonStrings = new Vector<PhonString>();
		
	//	log(lattice.toString());
		for (Enumeration<PhonString> e = recogResults.elements() ; e.hasMoreElements();) {
			//	log(">>>>> Parsing path " + incrStr + "...");
			PhonString phon = e.nextElement();
	//		log("word recognition path currently parsed: " + phon.wordSequence);

			results.phon2LFsMapping.put(phon, new Vector<String>());
			try {
				PackedLFParseResults result;

				result = (PackedLFParseResults) incrParse(phon);

				if (result != null && result.plf != null) {
					for (Enumeration<String> f = result.nonStandardRulesApplied.keys();
					f.hasMoreElements();) {
						String ID = f.nextElement();
						nonStandardRules.put((ID+ "-" + incrStr), 
								result.nonStandardRulesApplied.get(ID));
					}
					LFPacking packingTool2 = new LFPacking();
					LogicalForm[] unpackedLFs = packingTool2.unpackPackedLogicalForm(result.plf);

					Vector<String> vec = results.phon2LFsMapping.get(phon);

					for (int i = 0; i < unpackedLFs.length; i++) {		

						unpackedLFs[i].logicalFormId = 
							unpackedLFs[i].logicalFormId + "-" + incrStr;

						vec.add(unpackedLFs[i].logicalFormId);

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
					totalLFs.addAll(lfs);

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

		/**if (areParsesFinished) {
		for (int i = 0; i < totalLFsArray.length; i++) {
		log("lf"+i);
		LFUtils.lfToGraph(totalLFsArray[i], "lf"+i, true);
		}
		} */

		log("now packing " + totalLFsArray.length + " logical forms for the whole lattice");

		PackedLogicalForm plf = packingTool.packLogicalForms(totalLFsArray);
		results.plf = plf;
		results.nonStandardRulesApplied = nonStandardRules;
		results.plf.packedLFId = "plf"+ utteranceIncrement;
//		if (areParsesFinished) {
		results.finalized = this.FINAL_PARSE;
		utteranceIncrement++;
//		}
		results.stringPos = -1;

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
