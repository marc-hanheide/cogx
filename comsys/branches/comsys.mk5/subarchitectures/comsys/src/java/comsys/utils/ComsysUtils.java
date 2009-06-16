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

	public static PhonStringLFPair[] convertPhonString2LFPairs (PackedLFParseResults results) {
		
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
	
	
	
}
