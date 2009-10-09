
package comsys.utils;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Vector;

import opennlp.ccg.parse.DerivationHistory;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.Sign;

import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.comsysEssentials.PhonStringLFPair;
import comsys.processing.parse.*;
import comsys.datastructs.lf.LogicalForm;
import comsys.datastructs.lf.PackedLogicalForm;
import comsys.datastructs.lf.PackingNode;
import comsys.lf.utils.LFPacking;
import comsys.lf.utils.LFUtils;

public class ParsingUtils {


	public static PackedLFs createDummyPLF(PackedLFParseResults results) {

		PhonStringLFPair[] pairs = new PhonStringLFPair[0];
		
		if (results.plf != null && results.plf.root != null) {
		PackingNode pn = LFUtils.plfGetPackingNode(results.plf,results.plf.root);

		if (pn != null) {
			String[] LFs = pn.lfIds;

			Vector<String> LFsV = new Vector<String>();
			for (int i = 0; i < LFs.length; i++) {
				LFsV.add(LFs[i]);
			}

			pairs = new PhonStringLFPair[LFs.length];
			int i = 0;
			if (results.phon2LFsMapping != null) {
				for (PhonString phon : results.phon2LFsMapping.keySet()) {
					Vector<String> lfs = results.phon2LFsMapping.get(phon);
					for (String LFId : lfs) {
						if (LFsV.contains(LFId)) {
							PhonStringLFPair pair = new PhonStringLFPair();
							pair.phonStr = phon;
							pair.logicalFormId = LFId;
							pairs[i] = pair;
							i++;
						}
					}
				}
			}

		}  }
		
		PackedLFs plf = new PackedLFs("", pairs, new PhonString[0], results.stringPos, 
				results.plf, results.finalized,  "", new NonStandardRulesAppliedForLF[0]);	

		return plf;
	}

	/**
	 * create a mapping between the phonString and the logical forms
	 */ 
	public static PackedLFParseResults createPhon2LFsMapping (
			PackedLFParseResults results, 
			PhonString str) {

		results.phon2LFsMapping = new Hashtable<PhonString,Vector<String>>();
		results.phon2LFsMapping.put(str, new Vector<String>());
		if (results != null && results.plf != null) {
			LFPacking packingTool2 = new LFPacking();
			LogicalForm[] unpackedLFs = packingTool2.unpackPackedLogicalForm(results.plf);			
			Vector<String> vec = results.phon2LFsMapping.get(str);
			for (int i = 0; i < unpackedLFs.length; i++) {		
				vec.add(unpackedLFs[i].logicalFormId);
			}
		}
		return results;
	}


	public static Hashtable<String,Integer> getNonStandardRules(Sign sign) {
		Hashtable<String,Integer> nonStandardRules = 
			new Hashtable<String,Integer>();

		if (sign.getWords().size() == 1 && 
				sign.getDerivationHistory().NbLexicalCorrectionRulesApplied > 0) {
			nonStandardRules.put("recogError-"+sign.getOrthography(), 1);
		}

		DerivationHistory dh = sign.getDerivationHistory();
		Sign[] inputsigns = dh.getInputs();

		if (dh != null && dh.getRule() != null && 
				dh.getRule().name().contains("disclevelcomp")) {
			String rulename = dh.getRule().name();
			if (nonStandardRules.containsKey(rulename)) {
				Integer oldValue = nonStandardRules.get(rulename);
				nonStandardRules.put(rulename, 
						new Integer(oldValue.intValue() + 1));
			}
			else {
				nonStandardRules.put(rulename, new Integer(1));
			}
		}
		
		else if (dh != null && dh.getRule() != null && 
				dh.getRule().name().contains("x2du")) {
			String rulename = dh.getRule().name();
			if (nonStandardRules.containsKey(rulename)) {
				Integer oldValue = nonStandardRules.get(rulename);
				nonStandardRules.put(rulename, 
						new Integer(oldValue.intValue() + 1));
			}
			else {
				nonStandardRules.put(rulename, new Integer(1));
			}
		}
		
		else if (dh != null && dh.getRule() != null && 
				dh.getRule().name().contains("correction")) {
			String rulename = dh.getRule().name();
			if (nonStandardRules.containsKey(rulename)) {
				Integer oldValue = nonStandardRules.get(rulename);
				nonStandardRules.put(rulename, 
						new Integer(oldValue.intValue() + 1));
			}
			else {
				nonStandardRules.put(rulename, new Integer(1));
			}
		}

		// recursion
		for (int i=0; inputsigns != null && i < inputsigns.length; i++) {
			Hashtable<String,Integer> hash = getNonStandardRules(inputsigns[i]);
			for (Enumeration<String> e = hash.keys(); e.hasMoreElements();) {
				String rulename = e.nextElement();
				Integer newValue = hash.get(rulename);
				if (nonStandardRules.containsKey(rulename)) {
					Integer oldValue = nonStandardRules.get(rulename);
					nonStandardRules.put(rulename, 
							new Integer(oldValue.intValue() + newValue.intValue()));
				}
				else {
					nonStandardRules.put(rulename, 
							new Integer(newValue.intValue()));
				}
			}
		}

		return nonStandardRules;
	}


	public static List<Sign> collectSignInDerivationHistory(Sign topSign) {
		List<Sign> result = new ArrayList<Sign>();
		Sign[] signs = topSign.getDerivationHistory().getInputs();
		if (signs != null) {
		for (int i = 0 ; i < signs.length ; i++) {
			Sign sign = signs[i];
			result.add(sign);
			result.addAll(collectSignInDerivationHistory(sign));
		}
		}
		return result;
	}
	
	public static void lockSignsInDerivationHistory(Sign topSign, boolean lock) {
		Sign[] signs = topSign.getDerivationHistory().getInputs();
		if (signs != null) {
		for (int i = 0 ; i < signs.length ; i++) {
			Sign sign = signs[i];
			sign.locked = lock;
			lockSignsInDerivationHistory(sign, lock);
		}
		}
	}
	

	/**
		The method <i>visualize</i> visualizes the given packed logical form. 
		@param msg The log message to be printed 
	 */

	static void visualize (PackedLogicalForm plf) { 

	} // end visualize


}
