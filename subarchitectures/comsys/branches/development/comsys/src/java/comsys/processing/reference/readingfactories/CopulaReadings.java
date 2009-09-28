/** 
 The factory specifies possible readings for a copula construction. 
 The restrictor is given a restrictive reading, whereas the scope 
 has an attributive reading. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	090921
 @version	090921
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

public class CopulaReadings implements ReadingFactory {
	
	/** Copula constructions have root sort: ascription.*/ 
	
	private String sort = "ascription"; 

	/** 

	 
	 */ 
	
	public ReadingFactoryResults constructReadings (LogicalForm lf) { 
		// initialize the results
		ReadingFactoryResults results = new ReadingFactoryResults();
		// initialize the structures to be examined
		LFNominal root = lf.root; 
		LFRelation restrictor = LFUtils.lfNominalGetRelation(root,"Cop-Restr");
		LFRelation scope = LFUtils.lfNominalGetRelation(root,"Cop-Scope");	
		assert restrictor != null;
		assert scope != null; 
		LFNominal restrRoot = LFUtils.lfGetNominal(lf, restrictor.dep); 
		LFNominal scopeRoot = LFUtils.lfGetNominal(lf, scope.dep);
		assert restrRoot != null;
		assert scopeRoot != null; 
		// structures have now been properly initialized
	
		Vector<String> restrNoms = LFUtils.lfCollectNomvars (restrRoot,lf);
		restrNoms.add(restrRoot.nomVar);
		
		Vector<String> scopeNoms = LFUtils.lfCollectNomvars (scopeRoot,lf);
		Vector excludes = new Vector();
		excludes.addAll(restrNoms);
		excludes.addAll(scopeNoms);
		// initialize the results
		results.setExcludes(excludes); 
		// construct the (single) reading
		RefReading reading = new RefReading();
		reading.restrictiveTrees = new String[restrNoms.size()]; 
		reading.restrictiveTrees = restrNoms.toArray(reading.restrictiveTrees);
		reading.attributiveTrees = new String[scopeNoms.size()]; 
		reading.attributiveTrees = (String[])scopeNoms.toArray(reading.attributiveTrees);
		// add the (single) reading to the readings
		RefReadings readings = new RefReadings();
		readings.refRdngs = new RefReading[1];
		readings.refRdngs[0] = reading;
		
		readings.lform = lf;
		
		// return the results
		results.setReadings(readings);
		return results; 
	} // end method
	
	/** Returns the sort on which this factory operates. Copula constructions have root sort: ascription.*/ 
	
	public String getSort () { return sort; } 
	
	
	
} // end class
