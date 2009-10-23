/** 
 The factory specifies possible readings for an attitude construction. 
 It returns a single reading, with all nominals as attributive. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	091013
 @version	091013
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

public class QAttitudeReadings 
	implements ReadingFactory
{

	/** QAttitude constructions have root sort: q-attitude.*/ 
	
	private String sort = "q-attitude"; 	
	
	public String getSort() { return sort; }
	
	public ReadingFactoryResults constructReadings (LogicalForm lf) { 
		// initialize the results
		ReadingFactoryResults results = new ReadingFactoryResults();
		// initialize the structures to be examined
		LFNominal root = lf.root; 	
		// construct the (single) reading; no restrictive readings, attributive is the rest
		RefReading reading = new RefReading();
		reading.restrictiveTrees = new String[0]; 
		Vector nomVars = LFUtils.lfCollectNomvars(root,lf);
		reading.attributiveTrees = new String[nomVars.size()]; 
		reading.attributiveTrees = (String[])nomVars.toArray(reading.attributiveTrees);
		// add the (single) reading to the readings
		RefReadings readings = new RefReadings();
		readings.refRdngs = new RefReading[1];
		readings.refRdngs[0] = reading;
		readings.lform = lf;
		// return the results
		results.setReadings(readings);
		return results; 		
	} // end method
	
	
	
} // end class
