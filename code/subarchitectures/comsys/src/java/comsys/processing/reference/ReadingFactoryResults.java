//
//  ReadingFactoryResults.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference;


// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.comsysEssentials.RefReadings;


// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.Vector;

// -------------------------------------------------------
// LF imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm;




/** 
 The datastructure contains the possible readings, and a Vector with identifiers of nominals no longer to be considered. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	090921
 @version	090921
 */ 


public class ReadingFactoryResults {

	
	private RefReadings readings = null; 
	
	private Vector excludes = null; 
	
	
	public Vector getExcludes () { return excludes; } 
	
	public RefReadings getReadings () { return readings; } 
	
	public void setExcludes (Vector ex) { excludes = ex; }
	
	public void setReadings (RefReadings r) { readings = r; }
	
	
} // end class
