//=================================================================
// Copyright (C) 2005-2008 Geert-Jan M. Kruijff
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

package org.cognitivesystems.comsys.processing;

//=================================================================
// IMPORTS
//=================================================================

import BindingFeatures.Concept;
import BindingFeatures.Colour;
import BindingFeatures.Name;
import BindingFeatures.Shape;
import BindingFeatures.Size;

//-----------------------------------------------------------------
// COMSYS Imports
//-----------------------------------------------------------------

import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller.GREAttribute;
import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller;
import org.cognitivesystems.comsys.general.ComsysUtils;
import org.cognitivesystems.comsys.processing.saliency.SalienceModel;

//-----------------------------------------------------------------
// JAVA Imports
//-----------------------------------------------------------------
import java.util.Collection;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.TreeSet;

//=================================================================
// JAVA DOCUMENTATION
//=================================================================

/**
 The class <b>ObjectGREAlgorithm</b> implements an algorithm for 
 incrementally generating a referring expression for an object, 
 based on the original algorithm by Dale & Reiter. 

 @started	080903
 @version	080904
 @author	Geert-Jan M. Kruijff (gj@dfki.de)
 
*/  
 
//=================================================================
// CLASS 
//=================================================================


public class ObjectGREAlgorithm {

	
	//=================================================================
	// CLASS-GLOBAL DATA STRUCTURES
	//=================================================================
	

	/** logging flag, default is false, set by the command line option --log */
	boolean m_logging; 
	
	/** root feature used for computing the distractor set, default is GREAttribute.TYPE, set by the method setDistractorFeature */	
	GREAttribute m_distractorFeature; 
	
	/** distance in salience between the intended referent and any distractor. default is 10, set by method setPronominalizationDistance */ 
	int m_pronominalizationDistance;
	
	/** index starting at which the nominal variable indices are generated. default is 0, set by method setNominalStartIndex */ 
	int m_nomIndex; 
	
	/** treeset with GREAttribute attributes which should not be used when generating a referring expression */
	TreeSet<GREAttribute> m_excludedFeats;
	
	/** Salience model with salience scores (float) per discourse referent */
	SalienceModel m_salienceModel;
	
	
	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================
	
	public ObjectGREAlgorithm () { 
		init();
	} // end constructor
	
	private void init() { 
		m_nomIndex = 0;
		m_distractorFeature = GREAttribute.TYPE;
		m_pronominalizationDistance = 10;
		m_excludedFeats = new TreeSet<GREAttribute>();
		m_salienceModel = null;
	} // end init
		
	//=================================================================
	// ACCESSOR METHODS
	//=================================================================
	
	public int getNomIndex () { return m_nomIndex; } 
	
	public void setLogging (boolean l) { m_logging = l; } 
	
	public void setDistractorFeature (GREAttribute a) { m_distractorFeature = a; }
	
	public void setExcludedFeatures (TreeSet<GREAttribute> xfs) { m_excludedFeats = xfs; }
	
	public void setNominalStartIndex (int s) { m_nomIndex = s; }
	
	public void setPronominalizationDistance (int d) { m_pronominalizationDistance = d; }
	
	public void setSalienceModel (SalienceModel sm) { m_salienceModel = sm; }
	
	//=================================================================
	// COMPUTATION MAIN METHOD
	//=================================================================
	
	/** 
	 The method <i>generateRFX</i> produces a referring expression for
	 an intended referent. 
	 */ 
	
	public String generateRFX (GREAttributeTemplateFiller profile, String intdDiscRef) { 
		String resultRFX = "";
		// Initialize the TreeSet with discourse referent id's which have been ruled out
		TreeSet ruledOutRefs = new TreeSet();
		log("Generating referring expression for intended referent ["+intdDiscRef+"] with profile \n"+profile.toString());
		// Create the context distractor set, going by the profile and the feature defined by m_distractorFeature
		TreeMap<String,GREAttributeTemplateFiller> contextSet = createContextSet(profile,intdDiscRef,m_distractorFeature);
		// Compute the minimal salience distance between the intended referent and any distractor in the set
		float minSalienceDistance = computeMinimalSalienceDistance(m_salienceModel, intdDiscRef,contextSet);
		log("Minimum salience distance between the intended referent and distractors is ["+minSalienceDistance+"]");		
		// If the distance is large enough, pronominalize; else generate a nominal expression
		if (minSalienceDistance >= m_pronominalizationDistance) {
			log("The minimum salience distance ["+minSalienceDistance+"] is larger than the pronominalization minimum distance ["+m_pronominalizationDistance+"], so pronominalize ");
			//resultRFX = "@it"+ m_nomIndex++ +":thing(it ^ <Num>sg)";
			resultRFX = "@it"+ m_nomIndex++ +":thing(it ^ <Num>sg)";
		} else { 
			log("Distractors within salience range, so generate a referring expression");
			for (GREAttribute attr : profile.getAttributes()) { 
				// make sure the attribute has not been excluded from consideration
				if (!m_excludedFeats.contains(attr)) { 
					GREAttributeTemplateFiller bestValue = findBestValue(attr,profile,contextSet,ruledOutRefs);
					// If there is a restricting value for this attribute, use it in the rfx
					String value = (String)bestValue.getAttributeValue(attr);
					if (!value.equals("")) { 
						// Update the logical form for the rfx
						resultRFX = resultRFX + "^ "+bestValue.fillTemplate(attr,value);
						// Update the set with ruled out discourse referents
						ruledOutRefs = rulesOut(attr,value,contextSet,ruledOutRefs); // bit suboptimal, as rulesOut is already once computed in findBestValue ... 
					} // end check for non-empty values
				} // end if.. check for 	
			} // end foreach over attributes
			// Check whether we have been able to rule out all the distractors
			log("Comparing #ruled outs ["+ruledOutRefs.size()+"] against #distractors in context set ["+contextSet.size()+"]");
			if (ruledOutRefs.size() < contextSet.size()) {
				// If we have not yet ruled out everything, indicate the rfx is not unique
				resultRFX = resultRFX + "^ <Unique>false";
			} else { 	
				// else indicate it is unique
				resultRFX = resultRFX + "^ <Unique>true";				
			} // end if.. check whether we've 
		} // end if..else check whether to pronominalize
		// Return the resulting referring 
		log("Returning the following RFX: ["+resultRFX+"]");
		return resultRFX; 
	} // end generateRFX	
		
	//=================================================================
	// COMPUTATION HELPER METHODS
	//=================================================================
	
	/** 
	 The method <i>computeMinimalSalienceDistance</i> takes a map from discourse referent to saliency, 
	 and computes the minimal distance between the salience of the intended referent for a GRE, and a 
	 context set of discourse referents (provided through an iterator)
	
	 @param salienceMap		A salience model for the current dialogue context
	 @param intdDiscRef		The discourse referent for the intended referent
	 @param contextSet		The distractor discourse referents
	 @param salienceMap		The map from discourse referents to salience scores
	 @returns int			The minimal distance
	*/ 
	
	public float computeMinimalSalienceDistance (SalienceModel salienceMap, String intdDiscRef, TreeMap<String,GREAttributeTemplateFiller> distractors) { 
		float result = 1000.00f;
		float iddSalience = salienceMap.getSaliencyOfDiscRef(intdDiscRef);
		for (String distractor : distractors.keySet()) { 
			float distrSalience = salienceMap.getSaliencyOfDiscRef(distractor);
			if (iddSalience - distrSalience < result) { result = iddSalience - distrSalience; }
		} // end for
		return result;
	} // end computeMinimalSalienceDistance	
		
	/**
	 The method <i>createContextSet</i> creates a set of distractor 
	 objects given the intended referent. 
	 
	*/ 
	
	public TreeMap<String,GREAttributeTemplateFiller> createContextSet (GREAttributeTemplateFiller profile, String intdDiscRef, GREAttribute distractorRoot) { 
		TreeMap<String,GREAttributeTemplateFiller> resultMap = new TreeMap<String,GREAttributeTemplateFiller>();
		// Retrieve discourse referents from the SDRS, by the given distractor root and the value it has in the intended referent profile
		String rootValue = (String) profile.getAttributeValue(distractorRoot); 
		resultMap = ComsysUtils.retrieveReferentsByContent(distractorRoot,rootValue);
		// Remove the intended referent from the context set
		resultMap.remove(intdDiscRef);
		// Return the result
		return resultMap;
	} // end createContextSet
	
	
	/** 
	 The method <i>findBestValue</i> returns the best value for the 
	 specified attribute, given the intended referent and the context
	 set, as part of a template. If there is no such value, getting the value for 
	 the attribute will return an empty String. 
	 <p>
	 At the moment, findBestValue just checks whether including the given 
	 property in the description for the referent makes any difference with 
	 regard to what distractors we can exclude. 
	 <p> 
	 The value for the feature stored in <tt>m_distractorFeature</tt> is always
	 added, without further checks on ruled-out distractors.
	 
	*/ 
	
	public GREAttributeTemplateFiller findBestValue (GREAttribute attr, GREAttributeTemplateFiller profile, TreeMap<String,GREAttributeTemplateFiller> contextSet, TreeSet<String> ruledOutRefs) {
		GREAttributeTemplateFiller bestValue = null; 
		// Get the value for the attribute from the profile for the intended referent
		String value = (String) profile.getAttributeValue(attr);
		// If we are looking at the root feature, add it without further checks
		if (attr.equals(m_distractorFeature)) { 
			bestValue.setAttributeValue(attr,value);
		} else { 
			// Get the size of the set with ruled-out distractors when including this attribute and value
			TreeSet<String> potentiallyRuledOut = rulesOut(attr,value,contextSet,ruledOutRefs);
			// If the size gets larger, include the attribute and value in the return result
			if (potentiallyRuledOut.size() > ruledOutRefs.size()) { 
				bestValue.setAttributeValue(attr,value);
			} // end if.. check for size increase
		} // end if..else check for root feature
		// Return the result
		return bestValue;
	} // end findBestValue	
	
	

 	
	
	/** 
	 The method <i>rulesOut</i> creates an updated set of distractor
	 objects, given the assignment of a specific value to a given 
	 parameter in the GRE attribute template. The check is based on 
	 retrieving the value for the given attribute for a discourse referent, 
	 and checking whether it is different from the provided value. It is 
	 different, then the referent is excluded. 
	 
	 @param	attr		The attribute to be used for checking exclusion
	 @param	value		The value for the attribute, to be compared against
	 @param	contextSet	The set of discourse referents to be checked
	 @param	ruledOutRefs The set of referents already excluded
	 @returns TreeSet<String> A treeset with IDs for the discourse referent to be excluded. 
	 */ 
	 
	public TreeSet<String> rulesOut (GREAttribute attr, String value, TreeMap<String,GREAttributeTemplateFiller> contextSet, TreeSet<String> ruledOutRefs) { 
		TreeSet<String> result = new TreeSet<String>();
		for (String discRef : contextSet.keySet() ) { 
			// If the discref has already been excluded, keep it this way
			if (ruledOutRefs.contains(discRef)) { 
				result.add(discRef);
			} else { 
				// Get the profile for this discourse referent
				GREAttributeTemplateFiller profile = contextSet.get(discRef);
				// Get the value for the given attribute as stored in this profile
				String profileValue = (String) profile.getAttributeValue(attr);
				// If the stored value is different from the given value, exclude this referent
				if (!profileValue.equals(value)) {
					result.add(discRef);
				} // if.. check for unequal values, unequality means exclusion
			} // end if..else check whether already excluded	
		} // end 
		// Return the result
		return result;
	} // end rulesOut
	
	//=================================================================
	// MISC METHODS
	//=================================================================
	
	public void log(String m) { if (m_logging) { System.out.println("[ObjectGREAlgorithm] "+m); } }
	
} // end class
