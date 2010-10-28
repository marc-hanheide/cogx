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

import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

/**
 * Computes the saliency as a function over recency of mention. This function is 
 * discrete: 
 * <ul>
 * <li> recency of 0 maps to the highest saliency, of 1.0f;
 * <li> recency of 1 maps to a saliency of 0.8f;
 * <li> recency of 2 maps to a saliency of 0.6f;
 * <li> recency of 3 maps to a saliency of 0.4f;
 * <li> any other recency score maps to a defaulting saliency of 0.1f. 
 * </ul> 
 * 
 * Saliency thus ranges between 0.1f (lowest) and 1.0f (highest). 
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @author 	Pierre Lison (plison@dfki.de)
 * @version 100608
 */


public class RecencySalienceScorer 
implements SalienceScorer
{

	/** 
	 * Compute saliency simply on the basis of recency of occurrence 
	 * 
	 * @param	recency	The recency of occurrence
	 * @return  float 	The computed saliency
	 */ 
	
	public float computeSaliency (int recency) 
	{
		float score = 0.0f; 
		switch (recency) {
			case 0: score = 1.0f; break;
			case 1: score = 0.8f; break;
			case 2: score = 0.6f; break;
			case 3: score = 0.4f; break;
			default: score = 0.1f; break;
		}
		return score;	
	} // end computeSaliency
	
	/** 
	 * <b>Not implemented</b>. Compute saliency simply on the basis of recency of occurrence 
	 * 
	 * @param  	id		The identifier of the object
	 * @param	recency	The recency of occurrence
	 * @return  float 	The computed saliency
	 * @throws	DialogueMissingValueException Thrown if the object identifier is not provided
	 */ 
	
	public float computeSaliency (String id, int recency)
	throws DialogueMissingValueException, UnsupportedOperationException
	{
		throw new UnsupportedOperationException("Cannot compute saliency: method not implemented");
	}
	
	/** 
	 * <b>Not implemented</b>. Compute saliency simply on the basis of recency of occurrence 
	 * 
	 * @param  	id		The identifier of the object
	 * @param	meaning	The meaning of the object
	 * @param	recency	The recency of occurrence
	 * @return  float 	The computed saliency
	 * @throws	DialogueMissingValueException Thrown if the object identifier or meaning is not provided
	 */ 
	
	public float computeSaliency (String id, Object meaning, int recency)
	throws DialogueMissingValueException,UnsupportedOperationException
	{
		throw new UnsupportedOperationException("Cannot compute saliency: method not implemented");
	}
	
	/** 
	 * <b>Not implemented</b>. Compute saliency simply on the basis of recency of occurrence 
	 * 
	 * @param  	form	The form of the object
	 * @param	recency	The recency of occurrence
	 * @return  float 	The computed saliency
	 * @throws	DialogueMissingValueException Thrown if the object identifier is not provided
	 */ 
	
	public float computeSaliency (int recency, Object form)
	throws DialogueMissingValueException, UnsupportedOperationException
	{
		throw new UnsupportedOperationException("Cannot compute saliency: method not implemented");
	}
	
	
} // end class
