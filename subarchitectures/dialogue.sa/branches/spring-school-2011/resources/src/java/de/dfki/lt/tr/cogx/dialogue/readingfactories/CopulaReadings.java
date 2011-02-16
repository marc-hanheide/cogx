
// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.cogx.dialogue.readingfactories;

//=================================================================
// IMPORTS

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.ref.RefReading;
import de.dfki.lt.tr.dialogue.slice.ref.RefReadings;

// Dialogue API 
import de.dfki.lt.tr.dialogue.ref.ReadingFactory;
import de.dfki.lt.tr.dialogue.ref.ReadingFactoryResults;
import de.dfki.lt.tr.dialogue.util.LFUtils; 

// JAVA 
import java.util.Vector; 

/** 
The factory specifies possible readings for a copula construction. 
The restrictor is given a restrictive reading, whereas the scope 
has an attributive reading. 

@author	Geert-Jan Kruijff
@email		gj@dfki.de
@started	090921
@version	090921
*/ 
public class CopulaReadings 
implements ReadingFactory 
{
	
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
		Vector restrNoms = LFUtils.lfCollectNomvars (restrRoot,lf);
		Vector scopeNoms = LFUtils.lfCollectNomvars (scopeRoot,lf);
		Vector excludes = new Vector();
		excludes.addAll(restrNoms);
		excludes.addAll(scopeNoms);
		// initialize the results
		results.setExcludes(excludes); 
		// construct the (single) reading
		RefReading reading = new RefReading();
		reading.restrictiveTrees = new String[restrNoms.size()]; 
		reading.restrictiveTrees = (String[])restrNoms.toArray(reading.restrictiveTrees);
		reading.attributiveTrees = new String[scopeNoms.size()]; 
		reading.attributiveTrees = (String[])scopeNoms.toArray(reading.attributiveTrees);
		// add the (single) reading to the readings
		RefReadings readings = new RefReadings();
		readings.refRdngs = new RefReading[1];
		readings.refRdngs[0] = reading;
		// return the results
		return results; 
	} // end method
	
	/** Returns the sort on which this factory operates. Copula constructions have root sort: ascription.*/ 
	
	public String getSort () { return sort; } 
	
	
	
} // end class
