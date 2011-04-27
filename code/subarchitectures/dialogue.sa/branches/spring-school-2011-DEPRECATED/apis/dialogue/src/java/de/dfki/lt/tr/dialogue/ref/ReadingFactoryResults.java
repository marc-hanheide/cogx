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
package de.dfki.lt.tr.dialogue.ref;

//=================================================================
// IMPORTS

// Java
import java.util.Vector;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.ref.RefReadings;

// Dialogue API 
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;


/** 
 * The datastructure contains the possible readings, and a Vector with identifiers of 
 * nominals no longer to be considered. 
 * 
 * @author	Geert-Jan Kruijff
 * @email	gj@dfki.de
 * @started	090921
 * @version	100608
*/ 

public class ReadingFactoryResults {

	// Slice data structure for storing readings
	private RefReadings readings = null; 
	// Vector with nominals no longer to be considered
	private Vector excludes = null; 
	
	/**
	 * Returns a list (Vector) of nominals which are no longer to be considered for readings
	 * @return Vector 	The vector with nominals
	 */
	public Vector getExcludes () { return excludes; } 
	
	/**
	 * Returns the constructed referential readings
	 * @return RefReadings	The readings
	 */
	
	public RefReadings getReadings () { return readings; } 
	
	/**
	 * Sets the excludes to the provided vector
	 * @param ex	The vector with nominals to be excluded from further consideration
	 * @throws DialogueMissingValueException Thrown if the vector is null
	 */
	
	public void setExcludes (Vector ex) 
	throws DialogueMissingValueException 
	{ 
		if (ex == null)
		{
			throw new DialogueMissingValueException("Cannot set excludes for reading factory: Vector is null");
		}
		excludes = ex; 
	} // end setExcludes
	
	/**
	 * Sets the stored readings to the provided object
	 * @param r		The computed readings
	 * @throws DialogueMissingValueException 	Thrown if the object is null
	 */
	
	public void setReadings (RefReadings r) 
	throws DialogueMissingValueException 
	{ 
		if (r == null)
		{
			throw new DialogueMissingValueException("Cannot set readings for reading factory: Object is null");
		}
		readings = r; 
	} // end setReadings
	
	
} // end class
