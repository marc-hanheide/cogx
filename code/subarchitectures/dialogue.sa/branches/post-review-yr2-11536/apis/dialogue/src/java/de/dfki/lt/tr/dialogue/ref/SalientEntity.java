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
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

public abstract class SalientEntity {

	String concept = "" ;
	float score;
	SalienceScorer _salienceScorer; 
	
	public SalientEntity (String concept) {
		this.concept = concept;
	}
	
	public void setConcept (String concept) {
		this.concept = concept;
	}
	
	public String getConcept() { return concept; }
	
	public float getScore() { return score; }
		
	/**
	 * Sets the salience scorer to be used
	 * @param scorer	The scorer
	 * @throws DialogueMissingValueException Thrown if the provided argument is null
	 */
	
	public void setSalienceScorer (SalienceScorer scorer)
	throws DialogueMissingValueException 
	{
		if (scorer == null)
		{
			throw new DialogueMissingValueException("Cannot set salience scorer: Provided scorer is null");
		}
		_salienceScorer = scorer;
	}
	
	/**
	 * Returns the salience scorer used 
	 * @return	SalienceScorer	The scorer
	 */
	
	public SalienceScorer getSalienceScorer () 
	{
		return _salienceScorer; 
	}

	
} // end class
