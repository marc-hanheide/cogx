//
//  ReferentialReadings.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference;

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.comsysEssentials.RefReading;
import comsys.datastructs.comsysEssentials.RefReadings;

import comsys.processing.reference.readingfactories.CopulaReadings;

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.HashMap; 
import java.util.Iterator;
import java.util.Vector;

// -------------------------------------------------------
// LF imports
// -------------------------------------------------------

import comsys.datastructs.lf.LFNominal;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;




/** 
 The class <b>ReferentialReadings</b> determines, for a given logical form, the different referential readings it might have. 

 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	090921
 @version	090921
 */ 
 
public class ReferentialReadings { 
	
	
	private HashMap readingFactories = null; 
	
	public RefReadings constructReadings (LogicalForm lf) { 
		assert readingFactories != null; // make sure we have run through init and have factories to work with
		RefReadings results = new RefReadings();
		Vector excludes = new Vector();
		// cycle over the nominals in the logical form, examine the non-yet-excluded ones
		Iterator<LFNominal> lfnomsIter = LFUtils.lfGetNominals(lf);
		while (lfnomsIter.hasNext()) {
			LFNominal nom = lfnomsIter.next();
			if (!excludes.contains(nom.nomVar)) { 
				// construct the readings
				assert readingFactories.containsKey(nom.sort);
				ReadingFactory factory = (ReadingFactory) readingFactories.get(nom.sort);
				ReadingFactoryResults fresults = factory.constructReadings(lf);
				// get the results: update the readings, update the excludes
				excludes.addAll(fresults.getExcludes());
				RefReadings readings = fresults.getReadings();
				results.refRdngs = addRefReadingArrays(results.refRdngs,readings.refRdngs);
			} // end if.. 
		} // end while
		// return results
		return results;
	} // end method constructReadings

	private RefReading[] addRefReadingArrays (RefReading[] array1, RefReading[] array2) { 
		RefReading[] result = new RefReading[array1.length+array2.length];
		int i;
		for (i=0; i<array1.length; i++) { result[i] = array1[i]; } 
		i = i+1; // move one beyond the array1 length
		for (int j=0; j < array2.length; j++) { result[i+j] = array2[j]; }
		return result;
	} // end addReadingArrays
	
	
	public void init () { 
		readingFactories = new HashMap();
		readingFactories.put("ascription",new CopulaReadings());
		
	} // end method init
	
	
} // end class
