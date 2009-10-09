/** 
 The factory specifies possible readings for a cognition construction. 
 The restrictor and scope are derived from the construction over 
 which the cognition predicate scopes. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	091003
 @version	091003
 */ 

// -------------------------------------------------------
// PACKAGE
// -------------------------------------------------------

package comsys.processing.reference.readingfactories;

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.comsysEssentials.RefReading;
import comsys.datastructs.comsysEssentials.RefReadings;


import comsys.processing.reference.ReadingFactory;
import comsys.processing.reference.ReadingFactoryResults;

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.Vector; 

// -------------------------------------------------------
// LF imports
// -------------------------------------------------------

import comsys.lf.utils.LFUtils; 
import comsys.datastructs.lf.LFNominal;
import comsys.datastructs.lf.LFRelation;
import comsys.datastructs.lf.LogicalForm;

// -------------------------------------------------------
// CLASS
// -------------------------------------------------------

public class CognitionReadings 
	implements ReadingFactory 
{

	/** Cognition constructions have root sort: cognition.*/ 
	
	private String sort = "cognition"; 	
	
	public String getSort() { return sort; }	
	
	public ReadingFactoryResults constructReadings (LogicalForm lf) { 
		// initialize the results
		ReadingFactoryResults results = new ReadingFactoryResults();
		// initialize the structures to be examined
		LFNominal root = lf.root; 		
		// Get to the embedded construction, under the Patient
		LFRelation patientR = LFUtils.lfNominalGetRelation(root,"Patient");
		assert patientR != null; 
		LFNominal patientRoot = LFUtils.lfGetNominal(lf, patientR.dep);
		assert patientRoot != null; 
		LFRelation pscopeR = LFUtils.lfNominalGetRelation(patientRoot,"Scope-in");
		assert pscopeR != null; 
		LFNominal pscopeRoot = LFUtils.lfGetNominal(lf,pscopeR.dep);
		assert pscopeRoot != null; 
		// Examine the embedded construction
		if (pscopeRoot.sort.equals("ascription")) { 
			LogicalForm scopeInLF = LFUtils.lfConstructSubtree(pscopeRoot,lf); 
			CopulaReadings factory = new CopulaReadings();
			results = factory.constructReadings(scopeInLF);
		} else { 
			
			
		} // end if..else
		// return the result
		return results;
	} // end method
	
	
} // end class
