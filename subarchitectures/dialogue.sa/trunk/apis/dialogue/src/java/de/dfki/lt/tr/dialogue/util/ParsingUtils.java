//=================================================================
//Copyright (C) 2007-2010 Pierre Lison (plison@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.util;

//=================================================================
//IMPORTS

//Java
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Vector;

//OpenNLP
import opennlp.ccg.parse.DerivationHistory;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.Sign;

//Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.parse.NonStandardRulesAppliedForLF;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.parse.PhonStringLFPair;
import de.dfki.lt.tr.dialogue.parse.*;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.PackedLogicalForm;
import de.dfki.lt.tr.dialogue.slice.lf.PackingNode;
import de.dfki.lt.tr.dialogue.util.LFPacking;
import de.dfki.lt.tr.dialogue.util.LFUtils;

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
				results.plf, results.finalized,  "", new NonStandardRulesAppliedForLF[0], 1.0f, false, null, "");

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

	public static boolean arePLFsFinalized(PackedLFs plfs) {
		return plfs.finalized == 1;
	}

}
