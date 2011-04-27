// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (plison@dfki.de)
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
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

/**
 * Object for storing basic saliency information for a visual object
 * 
 * @author 	Pierre Lison
 * @version	100608
 */


public class VisualSalientEntity extends SalientEntity{
 
	int[] location ;
	String colour  = "" ;
	
	float DEFAULT_SCORE = 1.0f;
	
	public VisualSalientEntity (String concept, float score) {
		super(concept);
		this.score = score;
	}
	
	public VisualSalientEntity (String concept) {
		super(concept);
		score = DEFAULT_SCORE;
	}
	
	/**
	 * Sets the 3D location of the visual entity 
	 * @param 	location	An int[] array of size 3 representing the (x,y,z) location of the entity 
	 * @throws 	DialogueMissingValueException Thrown if the array is not of size 3
	 */
	
	public void setLocation(int[] location) 
	throws DialogueMissingValueException 
	{
		if (location.length != 3)
		{
			throw new DialogueMissingValueException("Cannot set location: int array must be of size 3");
		}
		this.location = location;
	} // end setLocation
		
	/**
	 * Returns the 3D location of the visual entity, as an int[] array representing the (x,y,z) location
	 * @return int[] 	The 3D location
	 */
	
	public int[] getLocation () { return location ; }
	
	public String toString() {
		String result = concept + ": coordinates = (" + location[0] + "," 
						+ location[1] + "," + location[2] + ")";
		if (!colour.equals("")) {
			result += ", colour = " + colour;
		}
		return result;
	}
	
	public boolean isWellFormed() {
		return ((!concept.equals("")) && (location != null)); 
	}
}