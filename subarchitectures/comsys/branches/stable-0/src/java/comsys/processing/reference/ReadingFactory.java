//
//  ReadingFactory.java
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

// -------------------------------------------------------
// LF imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm;

/** 
 The interface specifies what methods a reading factory needs to specify. 
 The sort provides the sort of the root of the logical form on which this
 factory can operate. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	090921
 @version	090921
*/ 

public interface ReadingFactory {

	public	ReadingFactoryResults constructReadings (LogicalForm lf); 
	
	public  String getSort(); 
	
	
} // end interface
