//
//  ComsysUtils.java
//  
//
//  Created by Geert-Jan Kruijff on 6/12/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.utils;


import java.util.*;
import comsys.processing.parse.*;
import comsys.datastructs.comsysEssentials.*;

public class ComsysUtils {

	boolean LOGGING = true;
	
	public static PhonStringLFPair[] convertPhonString2LFPairs (PackedLFParseResults results) {
		
		if (results.phon2LFsMapping != null) {
		Hashtable<PhonString,Vector<String>> hash = results.phon2LFsMapping;
		Vector<PhonStringLFPair> pairs = new Vector<PhonStringLFPair>();
		for (Enumeration<PhonString> e = hash.keys(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			Vector<String> LFs = hash.get(phon);
			for (Enumeration<String> f = LFs.elements(); f.hasMoreElements();) {
				PhonStringLFPair pair = new PhonStringLFPair();
				pair.phonStr = phon;
				pair.logicalFormId = f.nextElement();
				pairs.add(pair);
			}
		}
		
		PhonStringLFPair[] pairsArray = new PhonStringLFPair[pairs.size()];
		pairsArray = pairs.toArray(pairsArray);
		return pairsArray;
		}
		else {
			log("No phon2LFMapping!");
			Vector<PhonStringLFPair> pairs = new Vector<PhonStringLFPair>();
			return null;
		}
		
	}
	
	
	public static PhonString getPhonStringFromPair (PackedLFs results, String lfId) {
		
		PhonStringLFPair[] pairs = results.phonStringLFPairs;
		
		for (int i = 0; i < pairs.length ; i++) {
			PhonStringLFPair pair = pairs[i];
			if (pair.logicalFormId.equals(lfId)) {
				return pair.phonStr;
			}
		}
		
		if (lfId.contains("nonparsable-")) {
			int index = new Integer(lfId.replace("nonparsable-", "")).intValue();
			PhonString phon = results.nonParsablePhonStrings[index];
			return phon;
		}
		
		return null;
	}
	

	public static NonStandardRulesAppliedForLF[] convertNonStandardRulesDataStructs (PackedLFParseResults results) {
		Hashtable<String,Hashtable<String,Integer>> hash = results.nonStandardRulesApplied;
		NonStandardRulesAppliedForLF[] nonStandardRulesForLf = new NonStandardRulesAppliedForLF[hash.size()];
		int count = 0;
		for (Enumeration<String> e = hash.keys(); e.hasMoreElements();) {
			String lfId = e.nextElement();
			Hashtable<String, Integer> minihash = hash.get(lfId);
			NonStandardRule[] nonStandardRules = new NonStandardRule[minihash.size()];
			
			int count2 = 0;
			for (Enumeration<String> f = minihash.keys(); f.hasMoreElements();) {
				String rulename = f.nextElement();
				int number = minihash.get(rulename).intValue();
				NonStandardRule nonStandardRule = new NonStandardRule(rulename, number);
				nonStandardRules[count2] = nonStandardRule;
				count2++;
			}
			NonStandardRulesAppliedForLF nonStandardRulesForLF = new NonStandardRulesAppliedForLF(lfId, nonStandardRules);
			nonStandardRulesForLf[count] = nonStandardRulesForLF;
			count++;
		}
		return nonStandardRulesForLf;
	}
	
	
	private static void log(String s) {
		System.out.println ("[ComsysUtils] " + s);
	}
	
	
}
