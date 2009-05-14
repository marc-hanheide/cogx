//=================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package org.cognitivesystems.comsys.general;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// COMSYS IMPORTS
//-----------------------------------------------------------------
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.CacheMapping;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.IndexAssociation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRule;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.NonStandardRulesAppliedForLF;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonStringLFPair;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.RecogResult;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SpokenOutputItem;
import org.cognitivesystems.comsys.data.RecognitionResult;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;
import org.cognitivesystems.comsys.data.CacheElement;
import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller.GREAttribute;
import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller;
import org.cognitivesystems.repr.lf.utils.LFUtils;
import org.omg.CORBA.Any;
import org.omg.CORBA.ORB;

import java.util.TreeMap;

import org.cognitivesystems.comsys.data.PackedLFParseResults;
import java.util.Arrays;

// import com.sun.corba.se.internal.iiop.ORB;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>ComsysUtils</b> implements an initialization method
which registers the ComsysOntology data types with the CAST data
type manager. Furthermore, it provides constructor methods for 
the data types to ensure that their variables are properly initialized 
(i.e. non-null), and several accessor/update methods for these 
data types.  
   
 
@version 070905 (started 061020) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
@author	 Nick A. Hawes (n.a.hawes@cs.bham.ac.uk)
 
*/
 
public class ComsysUtils {


	// ----------------------------------------------------------------
	// COMSYSESSENTIALS CONSTRUCTOR METHODS
	// ----------------------------------------------------------------	

	/** Returns a properly initialized ContentPlanningGoal */ 
	
	public static ContentPlanningGoal newContentPlanningGoal () { 
		ContentPlanningGoal result = new ContentPlanningGoal();
		result.cpgid = "";
		result.lf = LFUtils.newLogicalForm();
		return result;
	} // end newProductionLF		
	
	/** Returns a properly initialized ProductionLF */ 
	
	public static ProductionLF newProductionLF () { 
		ProductionLF result = new ProductionLF();
		result.plfid = "";
		result.lf = LFUtils.newLogicalForm();
		return result;
	} // end newProductionLF	
	
	/** Returns a properly initialized SpokenOutputItem */ 
	
	public static SpokenOutputItem newSpokenOutputItem () { 
		SpokenOutputItem result = new SpokenOutputItem("", "", ORB.init().create_any());
		result.inputStream = ORB.init().create_any();
		return result;
	} // end newSpokenOutputItem
	
	public static RecogResult newRecogResult(RecognitionResult recResult) {
		RecogResult recResultIDL = new RecogResult() ;
		recResultIDL.recString = recResult.getRecString() ; 
		recResultIDL.isRecognized = recResult.isRecognized() ; 
		recResultIDL.isConnectionClosed = recResult.isConnectionClosed() ; 
		recResultIDL.probability = recResult.getProbability() ; 
		recResultIDL.confidence = recResult.getConfidence() ; 
		recResultIDL.ipAddress = recResult.getOrigin() ;
		return recResultIDL ;
	}
	
//	
//	public static DiscRefBindings newDiscRefBindings () { 
//		DiscRefBindings result = new DiscRefBindings ();
//		result.drbid = "";
//		result.lfCollectionId = "";
//		result.bindings = new LFBindings[0];
//		return result;
//	} // end newDiscRefBindings
	
	
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	// ----------------------------------------------------------------
	// COMSYSESSENTIALS ACCESSOR METHODS
	// ----------------------------------------------------------------	
	
	/** Reallocates an array with a new size, and copies the contents
		of the old array to the new array.
		@param oldArray  the old array, to be reallocated.
		@param newSize   the new array size.
		@return          A new array with the same contents.
	*/
	private static Object resizeArray (Object oldArray, int newSize) {
	   int oldSize = java.lang.reflect.Array.getLength(oldArray);
	   Class elementType = oldArray.getClass().getComponentType();
	   Object newArray = java.lang.reflect.Array.newInstance(
			 elementType,newSize);
	   int preserveLength = Math.min(oldSize,newSize);
	   if (preserveLength > 0) System.arraycopy (oldArray,0,newArray,0,preserveLength);
	   return newArray; 
	}
	
	/** BINDING: Adds a discourse referent to the list of referents in a Binding object 
	
	@param discRefs The list of discourse referents to be updated
	@param discRef	The discourse referent to be added
	@return String[] The updated list of discourse referents
*/

public static String[] bindingAddDiscRef (String[] discRefs, String discRef) {
	int newSize = 1+java.lang.reflect.Array.getLength(discRefs);
	String[] updDiscRefs = (String[]) resizeArray(discRefs,newSize);
	updDiscRefs[newSize-1] = discRef; 
	return updDiscRefs;
} // end bindingAddDiscRef


	/**
	 The method <i>retrieveReferentsByContent</i> creates a map with discourse referents, as stored in the SDRS. In the map, 
	 each discourse referent is stored wth a "profile", represented by a template of attributes. Only those refererents are 
	 retrieved which have the given value for the specific attribute. 
	 
	 @returns TreeMap	The map, indexed by discourse referent id's, and with the profile for a discourse referent as value. 
	 
	*/
	
	public static TreeMap<String,GREAttributeTemplateFiller> retrieveReferentsByContent(GREAttribute attribute, String value)	{
		TreeMap<String,GREAttributeTemplateFiller> resultMap = new TreeMap<String,GREAttributeTemplateFiller>();
		
		
		
		return resultMap;
	} // end retrieveReferentsByContent

	
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

public static void log(String str) {
	System.out.println("[Comsys utils] " + str);
}

public static PhonStringLFPair[] convertPhonString2LFPairs (PackedLFParseResults results) {
	
	Hashtable<PhonString,Vector<String>> hash = results.phon2LFsMapping;
	Vector<PhonStringLFPair> pairs = new Vector<PhonStringLFPair>();
	for (Enumeration<PhonString> e = hash.keys(); e.hasMoreElements();) {
		PhonString phon = e.nextElement();
		Vector<String> LFs = hash.get(phon);
		for (Enumeration<String> f = LFs.elements(); f.hasMoreElements();) {
			PhonStringLFPair pair = new PhonStringLFPair();
			pair.phon = phon;
			pair.logicalFormId = f.nextElement();
			pairs.add(pair);
		}
	}
	
	PhonStringLFPair[] pairsArray = new PhonStringLFPair[pairs.size()];
	pairsArray = pairs.toArray(pairsArray);
	return pairsArray;
	
}



public static PhonString getPhonStringFromPair (PackedLFs results, String lfId) {
	
	PhonStringLFPair[] pairs = results.phonStringLFPairs;
	
	for (int i = 0; i < pairs.length ; i++) {
		PhonStringLFPair pair = pairs[i];
		if (pair.logicalFormId.equals(lfId)) {
			return pair.phon;
		}
	}
	
	if (lfId.contains("nonparsable-")) {
		int index = new Integer(lfId.replace("nonparsable-", "")).intValue();
		PhonString phon = results.nonParsablePhonStrings[index];
		return phon;
	}

	return null;
}

///** BINDINGS: clone the Bindings object */ 
//
//public static Binding[] bindingsClone (Binding[] bindings) { 
//	Binding[] result = new Binding[java.lang.reflect.Array.getLength(bindings)];
//	for (int i=0; i < java.lang.reflect.Array.getLength(bindings); i++) { 
//		Binding binding = (Binding) bindings[i];
//		Binding cloneBinding = new Binding();
//		cloneBinding.lfRef = binding.lfRef;
//		cloneBinding.bindingRelation = binding.bindingRelation;
//		cloneBinding.discRefs = binding.discRefs.clone();
//		result[i] = cloneBinding;
//	} // end for over Binding objects
//	return result;
//} //end bindingsClone
//
///** DISCREFBINDINGS: returns an updated list with LFBindings objects, given the existing 
//	list and a new LFBindings object to be added
//*/
//
//public static LFBindings[] drbAddLFBindings (LFBindings[] bindings, LFBindings bObj) { 
//	int newSize = 1+java.lang.reflect.Array.getLength(bindings);
//	LFBindings[] updBindings = (LFBindings[]) resizeArray(bindings,newSize);
//	updBindings[newSize-1] = bObj; 
//	return updBindings;		
//} // end drbAddLFBindings

	// ----------------------------------------------------------------
	// COMSYSESSENTIALS CAST INITIALIZATION METHOD
	// ----------------------------------------------------------------	




    //=================================================================
    // TOSTRING METHODS
    //=================================================================



} // end class
